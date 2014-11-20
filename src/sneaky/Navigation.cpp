
#include "Navigation.h"

#include "rob/memory/LinearAllocator.h"
#include "rob/renderer/Renderer.h"
#include "rob/Assert.h"
#include "rob/Log.h"

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

    void NavMesh::SetGrid(const b2World *world, const float halfSize, const float agentRadius)
    {
        ROB_ASSERT(agentRadius > 0.0f);
        const float gridSz = agentRadius;

        const size_t sideVerts = size_t(halfSize / gridSz) * 2 + 1;
        const size_t vertCount = sideVerts * sideVerts;
        rob::log::Debug("NavMesh: ", vertCount, " vertices");
        ROB_ASSERT(vertCount <= MAX_VERTICES);

        const size_t sideFaces = sideVerts - 1;
        const size_t faceCount = sideFaces * sideFaces;
        rob::log::Debug("NavMesh: ", faceCount, " faces");
        ROB_ASSERT(faceCount <= MAX_FACES);

        float vY = -halfSize;
        for (size_t y = 0; y < sideVerts; y++, vY += gridSz)
        {
            float vX = -halfSize;
            for (size_t x = 0; x < sideVerts; x++, vX += gridSz)
            {
//                if (!TestPoint(world, vX, vY))
//                    AddVertex(vX, vY);
                const bool active = !TestPoint(world, vX, vY);
                AddVertex(vX, vY, active);
            }
        }

        for (size_t y = 0; y < sideFaces; y++)
        {
            for (size_t x = 0; x < sideFaces; x++)
            {
                // The faces: 3-2
                //            |/|
                //            0-1
                const size_t i0 = y * sideVerts + x;
                const size_t i1 = i0 + 1;
                const size_t i2 = (y + 1) * sideVerts + x + 1;
                const size_t i3 = i2 - 1;
                Vert& v0 = m_vertices[i0];
                Vert& v1 = m_vertices[i1];
                Vert& v2 = m_vertices[i2];
                Vert& v3 = m_vertices[i3];

                bool faceActive = (v0.flags & v1.flags & v2.flags & v3.flags & VertActive);

                const uint16_t fi0 = AddFace(i0, i1, i2, faceActive);
                const uint16_t fi1 = AddFace(i2, i3, i0, faceActive);

                const uint16_t fi = (y * sideFaces + x) * 2;

                Face &f0 = m_faces[fi0];
                Face &f1 = m_faces[fi1];

                if (y > 0)
                    f0.neighbours[0] = fi - sideFaces * 2 + 1;
                if (x < sideFaces - 1)
                    f0.neighbours[1] = fi + 3;
                f0.neighbours[2] = fi + 1;

                if (y < sideFaces - 1)
                    f1.neighbours[0] = fi + sideFaces * 2;
                if (x > 0)
                    f1.neighbours[1] = fi - 2;
                f1.neighbours[2] = fi;
            }
        }

        m_gridSz = gridSz;
        m_halfSize = halfSize;
    }

    uint16_t NavMesh::GetFaceIndex(const vec2f &v) const
    {
        const int gridW = (m_halfSize * 2) / m_gridSz;
        const vec2f p = v + vec2f(m_halfSize);
        int x = p.x / m_gridSz;
        int y = p.y / m_gridSz;
        x = rob::Clamp(x, 0, gridW - 1);
        y = rob::Clamp(y, 0, gridW - 1);
        int index = (y * gridW + x) * 2;
        if (p.y > p.x) return index + 1;
        return index;
    }

    vec2f NavMesh::GetFaceCenter(const Face &f) const
    {
        const NavMesh::Vert &v0 = GetVertex(f.vertices[0]);
        const NavMesh::Vert &v1 = GetVertex(f.vertices[1]);
        const NavMesh::Vert &v2 = GetVertex(f.vertices[2]);

        return vec2f(v0.x + v1.x + v2.x, v0.y + v1.y + v2.y) / 3.0f;
    }

    float NavMesh::GetDist(uint16_t fi0, uint16_t fi1) const
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

    void NavMesh::AddVertex(float x, float y, bool active)
    {
        ROB_ASSERT(m_vertexCount < MAX_VERTICES);
        Vert &v = m_vertices[m_vertexCount++];
        v.x = x;
        v.y = y;
        v.flags = 0;
        v.flags |= active ? VertActive : 0x0;
        for (size_t i = 0; i < 8; i++) v.faces[i] = InvalidIndex;
    }

    uint16_t NavMesh::AddFace(uint16_t i0, uint16_t i1, uint16_t i2, bool active)
    {
        ROB_ASSERT(m_faceCount < MAX_FACES);
        const uint16_t faceI = m_faceCount++;
        Face &f = m_faces[faceI];
        f.vertices[0] = i0;
        f.vertices[1] = i1;
        f.vertices[2] = i2;
        for (size_t i = 0; i < 3; i++) f.neighbours[i] = InvalidIndex;
        f.flags = active ? FaceActive : 0;
        return faceI;
    }


    NavPath::NavPath()
        : m_len(0)
    { }

    void NavPath::AppendVertex(float x, float y)
    {
        ROB_ASSERT(m_len < MAX_PATH_LEN);
        m_path[m_len++] = vec2f(x, y);
    }

    size_t NavPath::GetLength() const
    { return m_len; }

    const vec2f &NavPath::GetVertex(size_t index) const
    {
        ROB_ASSERT(index < m_len);
        return m_path[index];
    }


    Navigation::Navigation()
        : m_world(nullptr)
        , m_mesh()
    { }

    Navigation::~Navigation()
    { }

    bool Navigation::CreateNavMesh(rob::LinearAllocator &alloc, const b2World *world, const float worldHalfSize, const float agentRadius)
    {
        m_world = world;
        m_mesh.Allocate(alloc);
        m_mesh.SetGrid(world, worldHalfSize, agentRadius);
        m_nodes = alloc.AllocateArray<Node>(m_mesh.GetFaceCount());
        m_np.SetMemory(alloc.AllocateArray<NavPath>(16), rob::GetArraySize<NavPath>(16));
        return false;
    }

    NavPath *Navigation::ObtainNavPath()
    { return m_np.Obtain(); }

    void Navigation::ReturnNavPath(NavPath *path)
    { m_np.Return(path); }

    bool Navigation::FindNodePath(uint16_t startFace, uint16_t endFace)
    {
        m_path.len = 0;

        const float inf = 1e6f;
        const int nodeCount = m_mesh.GetFaceCount();

        int n = nodeCount;

        for(int i = 0; i < nodeCount; ++i)
        {
            m_nodes[i].dist = inf;
            m_nodes[i].prev = NavMesh::InvalidIndex;
            m_nodes[i].closed = false;
        }

        m_nodes[startFace].dist = 0.0f;

        while (n > 0)
        {
            int u = 0;
            float d = inf;
            for(int i = 0; i < nodeCount; ++i)
            {
                if (m_nodes[i].closed) continue;
                if (m_nodes[i].dist < d)
                {
                    u = i;
                    d = m_nodes[i].dist;
                }
            }

            m_nodes[u].closed = true;
            n--;

            if(u == endFace) break;
            if(d > inf - 100.0f) return false;

            for (int i = 0; i < 3; i++)
            {
                int v = m_mesh.GetFace(u).neighbours[i];
                if (v == NavMesh::InvalidIndex)
                    continue;
                if (m_nodes[v].closed) // TODO: This is bad if the navmesh is not a grid (maybe)
                    continue;
                if ((m_mesh.GetFace(v).flags & NavMesh::FaceActive) == 0)
                    continue;

                float alt = d + m_mesh.GetDist(u, v);

                if (alt < m_nodes[v].dist)
                {
                    m_nodes[v].dist = alt;
                    m_nodes[v].prev = u;
                }
            }
        }

        int v = endFace;
        while (m_nodes[v].prev != NavMesh::InvalidIndex)
        {
            v = m_nodes[v].prev;
            m_path.len++;
        }

        if (m_path.len > MAX_PATH_LEN)
            return false;

        int u = endFace;
        for (int i = m_path.len - 1; i >= 0; i--)
        {
            m_path.path[i] = u;
            u = m_nodes[u].prev;
        }

        return true;
    }

    struct PathRayCast : public b2RayCastCallback
    {
        bool hit;

        PathRayCast()
            : hit(false)
        { }

        float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float32 fraction) override
        {
            hit = (fixture->GetBody()->GetType() == b2_staticBody);
            return !(hit);
        }

        bool ShouldQueryParticleSystem(const b2ParticleSystem* particleSystem) override
        {
            B2_NOT_USED(particleSystem);
            return false;
        }
    };

    void Navigation::FindStraightPath(const vec2f &start, const vec2f &end, NavPath *path)
    {
        int num = 0;
        vec2f candidates[3];
        vec2f dest = start;
        vec2f current = start;

        path->AppendVertex(start.x, start.y);

        for (int i = 0; i < m_path.len; i++)
        {
            const NavMesh::Face &f = m_mesh.GetFace(m_path.path[i]);

            num = 0;
            for (int j = 0; j < 3; j++)
            {
                PathRayCast rayCast;
                const NavMesh::Vert &v = m_mesh.GetVertex(f.vertices[j]);
                candidates[num] = vec2f(v.x, v.y);
                m_world->RayCast(&rayCast, ToB2(current), ToB2(candidates[num]));
                if (!rayCast.hit) num++;
            }

            float minDist = 1e6f;

            if (num == 0)
            {
                path->AppendVertex(dest.x, dest.y);
                current = dest;
            }
            else
            {
                for (int j = 0; j < num; j++)
                {
                    float dist = (candidates[j] - current).Length2();
                    if (dist < minDist)
                    {
                        minDist = dist;
                        dest = candidates[j];
                    }
                }
            }
        }

        path->AppendVertex(dest.x, dest.y);
        path->AppendVertex(end.x, end.y);
    }

    bool Navigation::Navigate(const vec2f &start, const vec2f &end, NavPath *path)
    {
        uint16_t startFace = m_mesh.GetFaceIndex(start);
        uint16_t endFace = m_mesh.GetFaceIndex(end);

        if (FindNodePath(startFace, endFace))
        {
            FindStraightPath(start, end, path);
            return true;
        }

        return false;
    }

    void Navigation::RenderMesh(rob::Renderer *renderer) const
    {
        renderer->SetModel(mat4f::Identity);
        renderer->BindColorShader();
//        renderer->SetColor(rob::Color::LightBlue);

//        const size_t vertCount = m_mesh.GetVertexCount();
//        for (size_t i = 0; i < vertCount; i+=2)
//        {
//            const NavMesh::Vert &v = m_mesh.GetVertex(i);
////            if (v.x >= -20.0f && v.x < 20.0f)
//            if (v.flags & NavMesh::VertActive)
//                renderer->SetColor(rob::Color(0.0f, 0.5f, 1.0f));
//            else
//                renderer->SetColor(rob::Color::Red);
//            renderer->DrawFilledCircle(v.x, v.y, 0.10f);
//        }

//        renderer->SetColor(rob::Color::LightGreen);
        const size_t faceCount = m_mesh.GetFaceCount();
        for (size_t i = 0; i < faceCount; i++)
        {
            const NavMesh::Face &f = m_mesh.GetFace(i);
            const NavMesh::Vert &v0 = m_mesh.GetVertex(f.vertices[0]);
            const NavMesh::Vert &v1 = m_mesh.GetVertex(f.vertices[1]);
            const NavMesh::Vert &v2 = m_mesh.GetVertex(f.vertices[2]);

            if (f.flags & NavMesh::FaceActive)
                renderer->SetColor(rob::Color::LightGreen);
            else
                renderer->SetColor(rob::Color::Red);

            renderer->DrawLine(v0.x, v0.y, v1.x, v1.y);
            renderer->DrawLine(v1.x, v1.y, v2.x, v2.y);
            renderer->DrawLine(v2.x, v2.y, v0.x, v0.y);
        }

        renderer->SetColor(rob::Color::Magenta);
        const size_t pathLen = m_path.len;
        for (size_t i = 0; i < pathLen; i++)
        {
            const NavMesh::Face &f = m_mesh.GetFace(m_path.path[i]);
            const NavMesh::Vert &v0 = m_mesh.GetVertex(f.vertices[0]);
            const NavMesh::Vert &v1 = m_mesh.GetVertex(f.vertices[1]);
            const NavMesh::Vert &v2 = m_mesh.GetVertex(f.vertices[2]);

            renderer->DrawLine(v0.x, v0.y, v1.x, v1.y);
            renderer->DrawLine(v1.x, v1.y, v2.x, v2.y);
            renderer->DrawLine(v2.x, v2.y, v0.x, v0.y);
        }
    }

    void Navigation::RenderPath(rob::Renderer *renderer, const NavPath *path) const
    {
        renderer->SetModel(mat4f::Identity);
        renderer->BindColorShader();
        renderer->SetColor(rob::Color::White);

        const size_t pathLen = path->GetLength();
        for (size_t i = 1; i < pathLen; i++)
        {
            const vec2f &v0 = path->GetVertex(i - 1);
            const vec2f &v1 = path->GetVertex(i);
            renderer->DrawLine(v0.x, v0.y, v1.x, v1.y);
        }
    }

} // sneaky
