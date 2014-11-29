
#include "NavMesh.h"

#include "rob/memory/LinearAllocator.h"
#include "rob/Assert.h"
#include "rob/Log.h"

#include <clipper.hpp>
#include <poly2tri.h>
#include <polypartition.h>

namespace sneaky
{

    class PointTest : public b2QueryCallback
    {
    public:
        PointTest() : m_hit(false) { }

        bool ReportFixture(b2Fixture *fixture) override
        {
            m_hit = m_hit || (fixture->GetBody()->GetType() == b2_staticBody);
            return m_hit ? false : true;
        }

        bool ShouldQueryParticleSystem(const b2ParticleSystem* particleSystem) override
        { return false; }

        bool m_hit;
    };


    NavMesh::NavMesh()
        : m_faceCount(0)
        , m_faces(nullptr)
        , m_vertexCount(0)
        , m_vertices(nullptr)
    { }

    NavMesh::~NavMesh()
    { }

    void NavMesh::Allocate(rob::LinearAllocator &alloc)
    {
        m_faces = alloc.AllocateArray<Face>(MAX_FACES);
        m_vertices = alloc.AllocateArray<Vert>(MAX_VERTICES);
    }

    void ClassifyPaths(const ClipperLib::Paths &paths, ClipperLib::Paths &solids, ClipperLib::Paths &holes)
    {
        solids.reserve(paths.size() / 2);
        holes.reserve(paths.size() / 2);
        for (size_t pl = 0; pl < paths.size(); pl++)
        {
            const ClipperLib::Path &path = paths[pl];

            int orientation = 0;
            for (size_t p = 1; p < path.size(); p++)
            {
                const ClipperLib::IntPoint &ip0 = path[p - 1];
                const ClipperLib::IntPoint &ip1 = path[p];
                orientation += (ip1.X - ip0.X) * (ip1.Y + ip0.Y);
            }
            const ClipperLib::IntPoint &ip0 = path[path.size() - 1];
            const ClipperLib::IntPoint &ip1 = path[0];
            orientation += (ip1.X - ip0.X) * (ip1.Y + ip0.Y);

            const bool isHole = (orientation > 0);
            if (isHole)
                holes.push_back(path);
            else
                solids.push_back(path);
        }
    }

    static inline float TriArea(const vec2f &a, const vec2f &b, const vec2f &c)
    {
        const vec2f ab = b - a;
        const vec2f ac = c - a;
        return (ac.x * ab.y - ab.x * ac.y);
    }

    void AddPolyPath(TPPLPoly &poly, const ClipperLib::Path &path, const float clipperScale)
    {
        const float steinerStep = 12.0f;
        size_t pathSize = 0;
        {
            for (size_t p = 1; p < path.size(); p++)
            {
                const ClipperLib::IntPoint &ip0 = path[p - 1];
                const ClipperLib::IntPoint &ip1 = path[p];
                const vec2f p0 = vec2f(ip0.X, ip0.Y) / clipperScale;
                const vec2f p1 = vec2f(ip1.X, ip1.Y) / clipperScale;

                vec2f v = p1 - p0;
                const int cnt = int(v.Length() / steinerStep) + 1;
                pathSize += cnt;
            }
            const ClipperLib::IntPoint &ip0 = path[path.size() - 1];
            const ClipperLib::IntPoint &ip1 = path[0];
            const vec2f p0 = vec2f(ip0.X, ip0.Y) / clipperScale;
            const vec2f p1 = vec2f(ip1.X, ip1.Y) / clipperScale;

            vec2f v = p1 - p0;
            const int cnt = int(v.Length() / steinerStep) + 1;
            pathSize += cnt;
        }

        poly.Init(pathSize);

        size_t n = 0;
        for (size_t p = 1; p < path.size(); p++)
        {
            const ClipperLib::IntPoint &ip0 = path[p - 1];
            const ClipperLib::IntPoint &ip1 = path[p];
            const vec2f p0 = vec2f(ip0.X, ip0.Y) / clipperScale;
            const vec2f p1 = vec2f(ip1.X, ip1.Y) / clipperScale;

            vec2f v = p1 - p0;
            const int cnt = int(v.Length() / steinerStep) + 1;
            v /= cnt;
            for (int i = 0; i < cnt; i++)
            {
                const vec2f sp = p0 + v * i;
                poly[n].x = sp.x;
                poly[n].y = sp.y;
                n++;
            }
        }
        const ClipperLib::IntPoint &ip0 = path[path.size() - 1];
        const ClipperLib::IntPoint &ip1 = path[0];
        const vec2f p0 = vec2f(ip0.X, ip0.Y) / clipperScale;
        const vec2f p1 = vec2f(ip1.X, ip1.Y) / clipperScale;

        vec2f v = p1 - p0;
        const int cnt = int(v.Length() / steinerStep) + 1;
        v /= cnt;
        for (int i = 0; i < cnt; i++)
        {
            const vec2f sp = p0 + v * i;
            poly[n].x = sp.x;
            poly[n].y = sp.y;
            n++;
        }
    }

    void NavMesh::TriangulatePath(const ClipperLib::Path &path, const ClipperLib::Paths &holes, const float clipperScale)
    {
        TPPLPartition partition;
        std::list<TPPLPoly> polys;
        polys.resize(holes.size() + 1);

        std::list<TPPLPoly>::iterator polyIt = polys.begin();
        TPPLPoly &poly = *polyIt;
        AddPolyPath(poly, path, clipperScale);
//        poly.Init(path.size());
//        for (size_t i = 0; i < path.size(); i++)
//        {
//            const ClipperLib::IntPoint &ip = path[i];
//            poly[i].x = ip.X / clipperScale;
//            poly[i].y = ip.Y / clipperScale;
//        }

        polyIt++;
        for (size_t h = 0; h < holes.size(); h++, ++polyIt)
        {
            const ClipperLib::Path &holePath = holes[h];
            TPPLPoly &hole = *polyIt;
            hole.Init(holePath.size());
            hole.SetHole(true);
//            AddPolyPath(hole, holePath, clipperScale);
            for (size_t i = 0; i < holePath.size(); i++)
            {
                const ClipperLib::IntPoint &ip = holePath[i];
                hole[i].x = ip.X / clipperScale;
                hole[i].y = ip.Y / clipperScale;
            }
        }

        std::list<TPPLPoly> tris;
        partition.Triangulate_EC(&polys, &tris);

        std::list<TPPLPoly>::iterator it;
        for (it = tris.begin(); it != tris.end(); ++it)
        {
            TPPLPoly &tri = *it;

            const vec2f p0(tri[0].x, tri[0].y);
            const vec2f p1(tri[1].x, tri[1].y);
            const vec2f p2(tri[2].x, tri[2].y);

//            if (TriArea(vec2f(p0->x, p0->y), vec2f(p1->x, p1->y), vec2f(p2->x, p2->y)) > 0.0f) continue;

            index_t i0, i1, i2;
            GetVertex(p0.x, p0.y, &i0);
            GetVertex(p1.x, p1.y, &i1);
            GetVertex(p2.x, p2.y, &i2);
            AddFace(i0, i1, i2);
        }
    }

    void NavMesh::TriangulatePath2(const ClipperLib::Path &path, const ClipperLib::Paths &holes, const float clipperScale)
    {
        using namespace ClipperLib;
        const float steinerStep = 12.0f;
        std::vector<p2t::Point*> allPoints;
        std::vector<p2t::Point*> holePoints;

        std::vector<p2t::Point*> polyLine;
        {
            for (size_t p = 1; p < path.size(); p++)
            {
                const IntPoint &ip0 = path[p - 1];
                const IntPoint &ip1 = path[p];
                const vec2f p0 = vec2f(ip0.X, ip0.Y) / clipperScale;
                const vec2f p1 = vec2f(ip1.X, ip1.Y) / clipperScale;

                vec2f v = p1 - p0;
                const int cnt = int(v.Length() / steinerStep) + 1;
                v /= cnt;
                for (int i = 0; i < cnt; i++)
                {
                    const vec2f sp = p0 + v * i;
                    p2t::Point *point = new p2t::Point(sp.x, sp.y);
                    polyLine.push_back(point);
                }
            }
            const IntPoint &ip0 = path[path.size() - 1];
            const IntPoint &ip1 = path[0];
            const vec2f p0 = vec2f(ip0.X, ip0.Y) / clipperScale;
            const vec2f p1 = vec2f(ip1.X, ip1.Y) / clipperScale;

            vec2f v = p1 - p0;
            const int cnt = int(v.Length() / steinerStep) + 1;
            v /= cnt;
            for (int i = 0; i < cnt; i++)
            {
                const vec2f sp = p0 + v * i;
                p2t::Point *point = new p2t::Point(sp.x, sp.y);
                polyLine.push_back(point);
            }
        }

        p2t::CDT cdt(polyLine);

        for (size_t i = 0; i < holes.size(); i++)
        {
            const Path &hole = holes[i];
            polyLine.clear();
            for (size_t p = 0; p < hole.size(); p++)
            {
                const IntPoint &ip = hole[p];
                p2t::Point *point = new p2t::Point(ip.X / clipperScale, ip.Y / clipperScale);
                polyLine.push_back(point);
                holePoints.push_back(point);
            }
            cdt.AddHole(polyLine);
        }

        cdt.Triangulate();

        std::vector<p2t::Triangle*> tris = cdt.GetTriangles();
        std::vector<p2t::Triangle*>::iterator it;
//        std::list<p2t::Triangle*> tris = cdt.GetMap();
//        std::list<p2t::Triangle*>::iterator it;
        for (it = tris.begin(); it != tris.end(); ++it)
        {
            p2t::Triangle *tri = *it;

            const p2t::Point *p0 = tri->GetPoint(0);
            const p2t::Point *p1 = tri->GetPoint(1);
            const p2t::Point *p2 = tri->GetPoint(2);

//            bool isHole0 = false;
//            bool isHole1 = false;
//            bool isHole2 = false;
//            for (size_t i = 0; i < holePoints.size() && !(isHole0 && isHole1 && isHole0); i++)
//            {
//                if (p0 == holePoints[i]) isHole0 = true;
//                if (p1 == holePoints[i]) isHole1 = true;
//                if (p2 == holePoints[i]) isHole2 = true;
//            }
//            if (isHole0 && isHole1 && isHole2) continue;

            if (TriArea(vec2f(p0->x, p0->y), vec2f(p1->x, p1->y), vec2f(p2->x, p2->y)) > 0.0f) continue;

            index_t i0, i1, i2;
            GetVertex(p0->x, p0->y, &i0);
            GetVertex(p1->x, p1->y, &i1);
            GetVertex(p2->x, p2->y, &i2);
            AddFace(i0, i1, i2);
        }
    }

    void NavMesh::Create(const b2World *world, const float halfW, const float halfH, const float agentRadius)
    {
        m_gridSz = agentRadius;
        m_halfW = halfW;
        m_halfH = halfH;

        for (size_t i = 0; i < MAX_VERTICES; i++)
            m_vertCache.m_v[i] = nullptr;

        const float scale = 10.0f;
        const float wW = halfW * scale;
        const float wH = halfH * scale;

        using namespace ClipperLib;
        Clipper clipper;
        Path worldRegion;
        worldRegion.push_back(IntPoint(-wW, -wH));
        worldRegion.push_back(IntPoint(wW, -wH));
        worldRegion.push_back(IntPoint(wW, wH));
        worldRegion.push_back(IntPoint(-wW, wH));
        clipper.AddPath(worldRegion, ptSubject, true);

        const b2Body *body = world->GetBodyList();
        while (body)
        {
            if (body->GetType() == b2_staticBody)
            {
                Path &path = worldRegion; // Re-use worldRegion path
                path.clear();

                // NOTE: Assumes that there is only one fixture per static body.
                const b2Fixture *fixture = body->GetFixtureList();
                const b2Shape *shape = fixture->GetShape();
                switch (shape->GetType())
                {
                case b2Shape::e_polygon:
                    {
                        const b2PolygonShape *polyShape = (const b2PolygonShape*)shape;
                        for (int i = 0; i < polyShape->m_count; i++)
                        {
                            const b2Vec2 &v = polyShape->m_vertices[i];
                            const b2Vec2 wp = body->GetWorldPoint(v);
                            path.push_back(IntPoint(wp.x * scale, wp.y * scale));
                        }
                        clipper.AddPath(path, ptClip, true);
                    } break;
                default:
                    rob::log::Debug("NavMesh: Unsupported shape type");
                    break;
                }
            }
            body = body->GetNext();
        }

        ClipperLib::Paths paths;
        clipper.Execute(ctDifference, paths, pftNonZero, pftNonZero);

        ClipperOffset clipperOfft(2.0, agentRadius);
        clipperOfft.AddPaths(paths, jtRound, etClosedPolygon);
        paths.clear();
        clipperOfft.Execute(paths, -agentRadius * scale);

        Paths solids, holes;
        ClassifyPaths(paths, solids, holes);
        m_solids = solids;
        m_holes = holes;

        for (size_t s = 0; s < solids.size(); s++)
        {
            const Path &solid = solids[s];
            size_t start = m_faceCount;

            TriangulatePath(solid, holes, scale);

            for (size_t i = start; i < m_faceCount; i++)
            {
                Face &face = m_faces[i];
                const index_t i0 = face.vertices[0];
                const index_t i1 = face.vertices[1];
                const index_t i2 = face.vertices[2];
                for (size_t j = i + 1; j < m_faceCount; j++)
                {
                    Face &other = m_faces[j];
                    if (FaceHasEdge(other, i0, i1))
                    {
                        SetNeighbour(i, 0, j, i0, i1);
                    }
                    else if (FaceHasEdge(other, i1, i2))
                    {
                        SetNeighbour(i, 1, j, i1, i2);
                    }
                    else if (FaceHasEdge(other, i2, i0))
                    {
                        SetNeighbour(i, 2, j, i2, i0);
                    }
                }
            }
        }
        Refine();

//        for (size_t i = 0; i < m_faceCount; i++)
//        {
//            Face &face = m_faces[i];
//            const index_t i0 = face.vertices[0];
//            const index_t i1 = face.vertices[1];
//            const index_t i2 = face.vertices[2];
//            for (size_t j = i + 1; j < m_faceCount; j++)
//            {
//                Face &other = m_faces[j];
//                if (FaceHasEdge(other, i0, i1))
//                {
//                    SetNeighbour(i, 0, j, i0, i1);
//                }
//                else if (FaceHasEdge(other, i1, i2))
//                {
//                    SetNeighbour(i, 1, j, i1, i2);
//                }
//                else if (FaceHasEdge(other, i2, i0))
//                {
//                    SetNeighbour(i, 2, j, i2, i0);
//                }
//            }
//        }
    }

    void NavMesh::Refine()
    {
        static const int prevV[] = { 2, 0, 1 };
        static const int nextV[] = { 1, 2, 0 };
        for (index_t f = 0; f < m_faceCount; f++)
        {
            Face &face = m_faces[f];
            for (int ni = 0; ni < 3; ni++)
            {
                if (face.neighbours[ni] != InvalidIndex)
                {
                    //      v2i
                    //      /\          |
                    //     /  \         |
                    //v0i /    \ v1i
                    //    ------
                    //nv1i\    / nv0i
                    //     \  /         |
                    //      \/          |
                    //     nv2i

                    const index_t n = face.neighbours[ni];
                    Face &neighbour = m_faces[n];

                    const int nni = (neighbour.neighbours[0] == f ? 0 : (neighbour.neighbours[1] == f ? 1 : 2));

                    const int v0i = ni;
                    const int v1i = nextV[v0i];
                    const int v2i = nextV[v1i];

                    const int nv0i = nni;
                    const int nv1i = nextV[nv0i];
                    const int nv2i = nextV[nv1i];

                    const index_t v0 = face.vertices[v0i];
                    const index_t v1 = face.vertices[v1i];
                    const vec2f vec0(m_vertices[v0].x, m_vertices[v0].y);
                    const vec2f vec1(m_vertices[v1].x, m_vertices[v1].y);
                    const float edgeLen = rob::Distance(vec0, vec1);

                    const index_t v2 = face.vertices[v2i];
                    const index_t nv2 = neighbour.vertices[nv2i];
                    const vec2f vec2(m_vertices[v2].x, m_vertices[v2].y);
                    const vec2f nvec2(m_vertices[nv2].x, m_vertices[nv2].y);
                    const float altLen = rob::Distance(vec2, nvec2);

                    const float area0 = TriArea(vec0, nvec2, vec2);
                    const float area1 = TriArea(vec1, vec2, nvec2);

                    if (area0 > 0.0f && area1 > 0.0f && edgeLen > 1.1f * altLen)
                    {
                        face.flags = 1;
                        neighbour.flags = 1;
//                        continue;

                        const index_t fn_p = face.neighbours[prevV[ni]];
                        const index_t fn_n = face.neighbours[nextV[ni]];
                        const index_t nn_p = neighbour.neighbours[prevV[nni]];
                        const index_t nn_n = neighbour.neighbours[nextV[nni]];

                        face.vertices[0] = v0;
                        face.vertices[1] = nv2;
                        face.vertices[2] = v2;

                        neighbour.vertices[0] = v1;
                        neighbour.vertices[1] = v2;
                        neighbour.vertices[2] = nv2;

                        SetNeighbour(f, 0, nn_n, v0, nv2);
                        SetNeighbour(f, 1, n, nv2, v2);
                        SetNeighbour(f, 2, fn_p, v2, v0);

                        SetNeighbour(n, 0, fn_n, v1, v2);
                        SetNeighbour(n, 1, f, v2, nv2);
                        SetNeighbour(n, 2, nn_p, nv2, v1);
                        continue;
                    }
                }
            }
        }
    }

    void NavMesh::SetNeighbour(index_t fi, int ni, index_t fj, index_t v0, index_t v1)
    {
        m_faces[fi].neighbours[ni] = fj;

        int nj = 0;
        Face &f = m_faces[fj];
        if (v0 == f.vertices[0])
        {
            if (v1 == f.vertices[1])
                nj = 0;
            else if (v1 == f.vertices[2])
                nj = 2;
        }
        else if (v0 == f.vertices[1])
        {
            if (v1 == f.vertices[2])
                nj = 1;
            else if (v1 == f.vertices[0])
                nj = 0;
        }
        else if (v0 == f.vertices[2])
        {
            if (v1 == f.vertices[0])
                nj = 2;
            else if (v1 == f.vertices[1])
                nj = 1;
        }
        f.neighbours[nj] = fi;
    }

    bool NavMesh::FaceHasEdge(const Face &face, index_t v0, index_t v1)
    {
        return (face.vertices[0] == v0 || face.vertices[1] == v0 || face.vertices[2] == v0)
            && (face.vertices[0] == v1 || face.vertices[1] == v1 || face.vertices[2] == v1);
    }

    void NavMesh::SetGrid(const b2World *world, const float halfW, const float halfH, const float agentRadius)
    {
        ROB_ASSERT(agentRadius > 0.0f);
        const float gridSz = agentRadius;

        const size_t sideVertsX = size_t(2.0f * halfW / gridSz) + 1;
        const size_t sideVertsY = size_t(2.0f * halfH / gridSz) + 1;
        const size_t vertCount = sideVertsX * sideVertsY;
        rob::log::Debug("NavMesh: ", vertCount, " vertices");
        ROB_ASSERT(vertCount <= MAX_VERTICES);

        const size_t sideFacesX = sideVertsX - 1;
        const size_t sideFacesY = sideVertsY - 1;
        const size_t faceCount = sideFacesX * sideFacesY * 2;
        rob::log::Debug("NavMesh: ", faceCount, " faces");
        ROB_ASSERT(faceCount <= MAX_FACES);

        float vY = -halfH;
        for (size_t y = 0; y < sideVertsY; y++, vY += gridSz)
        {
            float vX = -halfW;
            for (size_t x = 0; x < sideVertsX; x++, vX += gridSz)
            {
//                if (!TestPoint(world, vX, vY))
//                    AddVertex(vX, vY);
                const bool active = !TestPoint(world, vX, vY);
                AddVertex(vX, vY, active);
            }
        }

        for (size_t y = 0; y < sideFacesY; y++)
        {
            for (size_t x = 0; x < sideFacesX; x++)
            {
                // The faces: 3-2
                //            |/|
                //            0-1
                const size_t i0 = y * sideVertsX + x;
                const size_t i1 = i0 + 1;
                const size_t i2 = (y + 1) * sideVertsX + x + 1;
                const size_t i3 = i2 - 1;
                Vert& v0 = m_vertices[i0];
                Vert& v1 = m_vertices[i1];
                Vert& v2 = m_vertices[i2];
                Vert& v3 = m_vertices[i3];

                bool face0Active = (v0.flags & v1.flags & v2.flags & VertActive);
                bool face1Active = (v0.flags & v2.flags & v3.flags & VertActive);
                if (face0Active)
                {
                    const vec2f c = vec2f(v0.x + v1.x + v2.x, v0.y + v1.y + v2.y) / 3.0f;
                    face0Active &= !TestPoint(world, c.x, c.y);
                }
                if (face1Active)
                {
                    const vec2f c = vec2f(v0.x + v2.x + v3.x, v0.y + v2.y + v3.y) / 3.0f;
                    face1Active &= !TestPoint(world, c.x, c.y);
                }

                const index_t fi0 = AddFace(i0, i1, i2); //, face0Active);
                const index_t fi1 = AddFace(i2, i3, i0); //, face1Active);

                const index_t fi = (y * sideFacesX + x) * 2;

                Face &f0 = m_faces[fi0];
                Face &f1 = m_faces[fi1];

                if (y > 0)
                    f0.neighbours[0] = fi - sideFacesX * 2 + 1;
                if (x < sideFacesX - 1)
                    f0.neighbours[1] = fi + 3;
                f0.neighbours[2] = fi + 1;

                if (y < sideFacesY - 1)
                    f1.neighbours[0] = fi + sideFacesX * 2;
                if (x > 0)
                    f1.neighbours[1] = fi - 2;
                f1.neighbours[2] = fi;
            }
        }

        m_gridSz = gridSz;
        m_halfW = halfW;
        m_halfH = halfH;
    }

    size_t NavMesh::GetFaceCount() const
    { return m_faceCount; }

    const NavMesh::Face& NavMesh::GetFace(size_t index) const
    {
        ROB_ASSERT(index < m_faceCount);
        return m_faces[index];
    }

    size_t NavMesh::GetVertexCount() const
    { return m_vertexCount; }

    const NavMesh::Vert& NavMesh::GetVertex(size_t index) const
    {
        ROB_ASSERT(index < m_vertexCount);
        return m_vertices[index];
    }

    int Side(const vec2f &v0, const vec2f &v1, const vec2f &v)
    {
        const vec2f e = v1 - v0;
        const vec2f n(-e.y, e.x);
        return e.Dot(v - v0);
    }

    bool NavMesh::FaceContainsPoint(const Face &face, const vec2f &v) const
    {
        const Vert &vert0 = m_vertices[face.vertices[0]];
        const Vert &vert1 = m_vertices[face.vertices[1]];
        const Vert &vert2 = m_vertices[face.vertices[2]];
        const vec2f v0(vert0.x, vert0.y);
        const vec2f v1(vert1.x, vert1.y);
        const vec2f v2(vert2.x, vert2.y);
        return Side(v0, v1, v) >= 0 && Side(v1, v2, v) >= 0 && Side(v2, v0, v) >= 0;
    }

    index_t NavMesh::GetFaceIndex(const vec2f &v) const
    {
        for (size_t i = 0; i < m_faceCount; i++)
        {
            Face &face = m_faces[i];
            if (FaceContainsPoint(face, v))
                return i;
        }
        return InvalidIndex;
//        const int gridW = (m_halfW * 2.0f) / m_gridSz;
//        const int gridH = (m_halfH * 2.0f) / m_gridSz;
//        const vec2f p = v + vec2f(m_halfW, m_halfH);
//        int x = (p.x + 0.5f) / m_gridSz;
//        int y = (p.y + 0.5f) / m_gridSz;
//        x = rob::Clamp(x, 0, gridW - 1);
//        y = rob::Clamp(y, 0, gridH - 1);
//        int index = (y * gridW + x) * 2;
//        if (p.y > p.x) return index + 1;
//        return index;
    }

    vec2f GetClosestPoint(const vec2f &v0, const vec2f &v1, const vec2f &v2, const vec2f &p)
    {
        const vec2f e0 = v1 - v0;
        const vec2f e1 = v2 - v0;
        const vec2f p0 = p - v0;
        const float d0 = e0.Dot(p0);
        const float d1 = e1.Dot(p0);
        if (d0 <= 0.0f && d1 <= 0.0f)
            return v0;

        const vec2f p1 = p - v1;
        const float d2 = e0.Dot(p1);
        const float d3 = e1.Dot(p1);
        if (d2 >= 0.0f && d3 <= d2)
            return v1;

        const float vc = d0 * d3 - d2 * d1;
        if (vc <= 0.0f && d0 >= 0.0f && d2 <= 0.0f)
        {
            const float t = d0 / (d0 - d2);
            return v0 + t * e0;
        }

        const vec2f p2 = p - v2;
        const float d4 = e0.Dot(p2);
        const float d5 = e1.Dot(p2);
        if (d5 >= 0.0f && d4 <= d5)
            return v2;

        const float vb = d4 * d1 - d0 * d5;
        if (vb <= 0.0f && d1 >= 0.0f && d5 <= 0.0f)
        {
            const float t = d1 / (d1 - d5);
            return v0 + t * e1;
        }

        const float va = d2 * d5 - d4 * d3;
        if (va <= 0.0f && (d3 - d2) >= 0.0f && (d4 - d5) >= 0.0f)
        {
            const float t = (d3 - d2) / ((d3 - d2) - (d4 - d5));
            return v1 + t * (v2 - v1);
        }

        const float denom = 1.0f / (va + vb + vc);
        const float t = vb * denom;
        const float u = vc * denom;
        return v0 + e0 * t + e1 * u;
    }

    vec2f NavMesh::GetClosestPointOnFace(const Face &face, const vec2f &p) const
    {
        const Vert &v0 = m_vertices[face.vertices[0]];
        const Vert &v1 = m_vertices[face.vertices[1]];
        const Vert &v2 = m_vertices[face.vertices[2]];
        return GetClosestPoint(vec2f(v0.x, v0.y), vec2f(v1.x, v1.y), vec2f(v2.x, v2.y), p);
    }

    index_t NavMesh::GetClampedFaceIndex(vec2f *v) const
    {
//        index_t index = GetFaceIndex(*v);
//        if (index != InvalidIndex) return index;
//        index = 0;

        index_t index = 0;
        vec2f clampedPos = GetClosestPointOnFace(m_faces[0], *v);
        float minDist = rob::Distance2(clampedPos, *v);

        for (size_t i = 1; i < m_faceCount; i++)
        {
            const vec2f pos = GetClosestPointOnFace(m_faces[i], *v);
            float dist = rob::Distance2(pos, *v);
            if (dist < minDist)
            {
                index = i;
                clampedPos = pos;
                minDist = dist;
            }
        }

        *v = clampedPos;
        return index;
    }

    vec2f NavMesh::GetFaceCenter(const Face &f) const
    {
        const Vert &v0 = GetVertex(f.vertices[0]);
        const Vert &v1 = GetVertex(f.vertices[1]);
        const Vert &v2 = GetVertex(f.vertices[2]);

        return vec2f(v0.x + v1.x + v2.x, v0.y + v1.y + v2.y) / 3.0f;
    }

    vec2f NavMesh::GetEdgeCenter(index_t f, int edge) const
    {
        static const int nextEdgeV[] = { 1, 2, 0 };
        const Face &face = GetFace(f);
        const Vert &v0 = GetVertex(face.vertices[edge]);
        const Vert &v1 = GetVertex(face.vertices[nextEdgeV[edge]]);
        return vec2f(v0.x + v1.x, v0.y + v1.y) / 2.0f;
    }

    float NavMesh::GetDist(index_t fi0, index_t fi1) const
    {
        const Face &f0 = m_faces[fi0];
        const Face &f1 = m_faces[fi1];
        return (GetFaceCenter(f0) - GetFaceCenter(f1)).Length();
    }

    bool FaceHasVertex(const NavMesh::Face &face, const index_t v)
    {
        return face.vertices[0] == v || face.vertices[1] == v || face.vertices[2] == v;
    }

    float NavMesh::GetDist(index_t fi0, index_t fi1, int edge) const
    {
        static const int nextEdge[] = { 1, 2, 0 };
        const Face &f0 = m_faces[fi0];
        const Face &f1 = m_faces[fi1];
        if (FaceHasVertex(f1, f0.vertices[edge]) || FaceHasVertex(f1, f0.vertices[nextEdge[edge]]))
        {
            return 0.0f;
        }
        return GetDist(fi0, fi1);
    }

    bool NavMesh::TestPoint(const b2World *world, float x, float y)
    {
        b2AABB aabb;
        aabb.lowerBound.Set(x-0.001f, y-0.001f);
        aabb.upperBound.Set(x+0.001f, y+0.001f);

        PointTest pointTest;
        world->QueryAABB(&pointTest, aabb);
        return pointTest.m_hit;
    }

    NavMesh::Vert* NavMesh::AddVertex(float x, float y, bool active)
    {
        ROB_ASSERT(m_vertexCount < MAX_VERTICES);
        Vert &v = m_vertices[m_vertexCount++];
        v.x = x;
        v.y = y;
        v.flags = 0;
        v.flags |= active ? VertActive : 0x0;
//        for (size_t i = 0; i < 8; i++) v.faces[i] = InvalidIndex;
        return &v;
    }

    NavMesh::Vert* NavMesh::GetVertex(const float x, const float y, index_t *index)
    {
        const index_t HASH_MASK = MAX_VERTICES - 1;
        const float scale = 10.0f;
        const int hx = x * scale;
        const int hy = y * scale;
        const float vx = hx / scale;
        const float vy = hy / scale;

        index_t hash = (hx * 32786 + hy) & HASH_MASK;

        Vert *v = m_vertCache.m_v[hash];
        if (v)
        {
            const index_t hstart = hash;
            while (v && hash < MAX_VERTICES)
            {
                v = m_vertCache.m_v[hash];
                if (v && !vec2f::Equals(vec2f(v->x, v->y), vec2f(vx, vy), scale))
                {
                    hash++;
                }
                else
                    break;
            }
            while (v && hash < hstart)
            {
                v = m_vertCache.m_v[hash];
                if (v && !vec2f::Equals(vec2f(v->x, v->y), vec2f(vx, vy), scale))
                {
                    hash++;
                }
                else
                    break;
                if (hash == hstart)
                    break;
            }
        }

        if (!v)
        {
            v = AddVertex(vx, vy, true);
            m_vertCache.m_v[hash] = v;
        }

        *index = index_t(v - m_vertices);
        return v;
    }

    index_t NavMesh::AddFace(index_t i0, index_t i1, index_t i2)
    {
        ROB_ASSERT(m_faceCount < MAX_FACES);
        const index_t faceI = m_faceCount++;
        Face &f = m_faces[faceI];
        f.vertices[0] = i0;
        f.vertices[1] = i1;
        f.vertices[2] = i2;

        const Vert &v0 = m_vertices[i0];
        const Vert &v1 = m_vertices[i1];
        const Vert &v2 = m_vertices[i2];
        if (TriArea(vec2f(v0.x, v0.y), vec2f(v1.x, v1.y), vec2f(v2.x, v2.y)) < 0.0f)
        {
            f.vertices[0] = i0;
            f.vertices[1] = i2;
            f.vertices[2] = i1;
        }

        for (size_t i = 0; i < 3; i++) f.neighbours[i] = InvalidIndex;
        f.flags = 0;
        return faceI;
    }

    void NavMesh::GetPortalPoints(const index_t from, const index_t to, vec2f &left, vec2f &right) const
    {
        const Face &fromFace = m_faces[from];
//        const Face &toFace = m_faces[to];

        int edge = 0;
        for (; edge < 3; edge++)
        {
            if (fromFace.neighbours[edge] == to)
                break;
        }

        const index_t v0 = fromFace.vertices[edge];
        const index_t v1 = fromFace.vertices[(edge + 1) % 3];
        const Vert &vert0 = m_vertices[v0];
        const Vert &vert1 = m_vertices[v1];
        left = vec2f(vert0.x, vert0.y);
        right = vec2f(vert1.x, vert1.y);
    }

} // sneaky
