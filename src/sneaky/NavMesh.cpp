
#include "NavMesh.h"

#include "rob/memory/LinearAllocator.h"
#include "rob/Assert.h"
#include "rob/Log.h"

#include <clipper.hpp>
#include <poly2tri.h>

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

    void NavMesh::Create(const b2World *world, const float halfW, const float halfH, const float agentRadius)
    {
        m_gridSz = agentRadius;
        m_halfW = halfW;
        m_halfH = halfH;

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
        clipper.Execute(ctDifference, paths, pftPositive, pftPositive);

        const float steinerStep = 12.0f;

        std::vector<p2t::Point*> polyLine;
        const Path &path = paths[0];
        for (size_t p = 1; p < path.size(); p++)
        {
            const IntPoint &ip0 = path[p - 1];
            const IntPoint &ip1 = path[p];
            const vec2f p0 = vec2f(ip0.X, ip0.Y) / scale;
            const vec2f p1 = vec2f(ip1.X, ip1.Y) / scale;

            p2t::Point *point = new p2t::Point(ip0.X / scale, ip0.Y / scale);
            polyLine.push_back(point);

            vec2f v = p1 - p0;
            const int cnt = v.Length() / steinerStep;
            v /= cnt;
            for (int i = 1; i < cnt; i++)
            {
                const vec2f sp = p0 + v * i;
                p2t::Point *point = new p2t::Point(sp.x, sp.y);
                polyLine.push_back(point);
            }
        }
        const IntPoint &ip0 = path[path.size() - 1];
        const IntPoint &ip1 = path[0];
        const vec2f p0 = vec2f(ip0.X, ip0.Y) / scale;
        const vec2f p1 = vec2f(ip1.X, ip1.Y) / scale;

        p2t::Point *point = new p2t::Point(ip0.X / scale, ip0.Y / scale);
        polyLine.push_back(point);

        vec2f v = p1 - p0;
        const int cnt = v.Length() / steinerStep;
        v /= cnt;
        for (int i = 1; i < cnt; i++)
        {
            const vec2f sp = p0 + v * i;
            p2t::Point *point = new p2t::Point(sp.x, sp.y);
            polyLine.push_back(point);
        }

        p2t::CDT cdt(polyLine);

        for (size_t i = 1; i < paths.size(); i++)
        {
            Path &path = paths[i];
            polyLine.clear();
            for (size_t p = 0; p < path.size(); p++)
            {
                const IntPoint &ip = path[p];
                p2t::Point *point = new p2t::Point(ip.X / scale, ip.Y / scale);
                polyLine.push_back(point);
            }
            cdt.AddHole(polyLine);
        }

        const int ptsX = int(halfW * 2.0f / steinerStep);
        const int ptsY = int(halfH * 2.0f / steinerStep);
        for (int y = 1; y < ptsY; y++)
        {
            for (int x = 1; x < ptsX; x++)
                cdt.AddPoint(new p2t::Point(-halfW + steinerStep * x, -halfH + steinerStep * y));
        }

        cdt.Triangulate();

        for (size_t i = 0; i < MAX_VERTICES; i++)
            m_vertCache.m_v[i] = nullptr;

        std::vector<p2t::Triangle*> tris = cdt.GetTriangles();
        for (size_t t = 0; t < tris.size(); t++)
        {
            p2t::Triangle *tri = tris[t];
            p2t::Point *p0 = tri->GetPoint(0);
            p2t::Point *p1 = tri->GetPoint(1);
            p2t::Point *p2 = tri->GetPoint(2);

            index_t i0, i1, i2;
            GetVertex(p0->x, p0->y, &i0);
            GetVertex(p1->x, p1->y, &i1);
            GetVertex(p2->x, p2->y, &i2);
            AddFace(i0, i1, i2, true);
        }

        for (size_t i = 0; i < m_faceCount; i++)
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

                const index_t fi0 = AddFace(i0, i1, i2, face0Active);
                const index_t fi1 = AddFace(i2, i3, i0, face1Active);

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
        const Face &face = GetFace(f);
        const Vert &v0 = GetVertex(face.vertices[edge]);
        const Vert &v1 = GetVertex(face.vertices[((edge + 2) & 3) - 1]);
        return vec2f(v0.x + v1.x, v0.y + v1.y) / 2.0f;
    }

    float NavMesh::GetDist(index_t fi0, index_t fi1) const
    {
        const Face &f0 = m_faces[fi0];
        const Face &f1 = m_faces[fi1];
        return (GetFaceCenter(f0) - GetFaceCenter(f1)).Length();
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

    NavMesh::Vert* NavMesh::GetVertex(float x, float y, index_t *index)
    {
        const index_t HASH_MASK = MAX_VERTICES - 1;
        const float scale = 10.0f;
        int hx = x * scale;
        int hy = y * scale;
        x = hx / scale;
        y = hy / scale;

        const index_t hash = (hx * 32786 + hy) & HASH_MASK;

        Vert *v = m_vertCache.m_v[hash];
        if (v)
        {
            const Vert *vstart = v;
            while (v && v < m_vertices + MAX_VERTICES)
            {
                if (v->x != x || v->y != y)
                    v++;
                else
                    break;
                if (v == vstart)
                    break;
            }
        }

        if (!v)
        {
            v = AddVertex(x, y, true);
            m_vertCache.m_v[hash] = v;
        }

        *index = index_t(v - m_vertices);
        return v;
    }

    inline float TriArea(const vec2f a, const vec2f &b, const vec2f &c)
    {
        const vec2f ab = b - a;
        const vec2f ac = c - a;
        return (ac.x * ab.y - ab.x * ac.y);
    }

    index_t NavMesh::AddFace(index_t i0, index_t i1, index_t i2, bool active)
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
        f.flags = active ? FaceActive : 0;
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
