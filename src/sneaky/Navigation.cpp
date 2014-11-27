
#include "Navigation.h"
#include "Sensor.h"

#include "rob/memory/LinearAllocator.h"
#include "rob/renderer/Renderer.h"
#include "rob/Assert.h"
#include "rob/Log.h"

namespace sneaky
{

    NavPath::NavPath()
        : m_len(0)
    { }

    void NavPath::Clear()
    { m_len = 0; }

    void NavPath::AppendVertex(float x, float y)
    {
        ROB_ASSERT(m_len < MAX_PATH_LEN);
        m_path[m_len++] = vec2f(x, y);
    }

    void NavPath::ReplaceVertex(size_t index, float x, float y)
    {
        ROB_ASSERT(index < m_len);
        m_path[index] = vec2f(x, y);
    }

    void NavPath::InsertVertex(size_t index, float x, float y)
    {
        ROB_ASSERT(m_len < MAX_PATH_LEN);
        ROB_ASSERT(index < m_len);
        for (size_t i = m_len; i > index; i--)
            m_path[i] = m_path[i - 1];
        m_path[index] = vec2f(x, y);
        m_len++;
    }

    bool NavPath::TryInsertVertex(size_t index, const vec2f &v, float maxReplaceDist)
    {
        ROB_ASSERT(index < m_len);
        if (rob::Distance(m_path[index], v) < maxReplaceDist)
        {
            ReplaceVertex(index, v.x, v.y);
        }
        else
        {
            if (index < MAX_PATH_LEN)
                InsertVertex(index, v.x, v.y);
            else
                return false;
        }
        return true;
    }

    size_t NavPath::GetLength() const
    { return m_len; }

    const vec2f &NavPath::GetVertex(size_t index) const
    {
        ROB_ASSERT(index < m_len);
        return m_path[index];
    }

    const vec2f &NavPath::GetDestination() const
    {
        ROB_ASSERT(m_len > 0);
        return m_path[m_len - 1];
    }

    bool NavPath::IsEmpty() const
    { return m_len == 0; }



    Navigation::Navigation()
        : m_world(nullptr)
        , m_mesh()
    { }

    Navigation::~Navigation()
    { }

    bool Navigation::CreateNavMesh(rob::LinearAllocator &alloc, const b2World *world, const float worldHalfW, const float worldHalfH, const float agentRadius)
    {
        m_world = world;
        m_mesh.Allocate(alloc);
        m_mesh.SetGrid(world, worldHalfW, worldHalfH, agentRadius);
        m_nodes = alloc.AllocateArray<Node>(m_mesh.GetFaceCount());
        m_np.SetMemory(alloc.AllocateArray<NavPath>(16), rob::GetArraySize<NavPath>(16));
        return false;
    }

    NavPath *Navigation::ObtainNavPath()
    { return m_np.Obtain(); }

    void Navigation::ReturnNavPath(NavPath *path)
    { m_np.Return(path); }

    bool Navigation::FindNodePath(const vec2f &start, const vec2f &end, uint16_t startFace, uint16_t endFace)
    {
//        rob::log::Debug("Nav: Start node: ", startFace, ", end node: ", endFace);

        NavMesh::Face sface = m_mesh.GetFace(startFace);
        if ((sface.flags & NavMesh::FaceActive) == 0)
        {
            bool goodFace = false;
            for (size_t i = 0; i < 3; i++)
            {
                if (sface.neighbours[i] == NavMesh::InvalidIndex) continue;
                const NavMesh::Face &nface = m_mesh.GetFace(sface.neighbours[i]);
                if ((nface.flags & NavMesh::FaceActive) != 0)
                {
                    startFace = sface.neighbours[i];
                    goodFace = true;
                    break;
                }
            }
            if (!goodFace && sface.neighbours[2] != NavMesh::InvalidIndex)
            {
                sface = m_mesh.GetFace(sface.neighbours[2]);
                if (sface.neighbours[0] != NavMesh::InvalidIndex)
                {
                    const NavMesh::Face &nface0 = m_mesh.GetFace(sface.neighbours[0]);
                    if ((nface0.flags & NavMesh::FaceActive) != 0)
                    {
                        startFace = sface.neighbours[0];
                    }
                }
                if (sface.neighbours[1] != NavMesh::InvalidIndex)
                {
                    const NavMesh::Face &nface1 = m_mesh.GetFace(sface.neighbours[1]);
                    if ((nface1.flags & NavMesh::FaceActive) != 0)
                    {
                        startFace = sface.neighbours[1];
                    }
                }
            }
        }

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

        index_t bestFace = startFace;
        float bestHeuristicCost = rob::Distance(start, end);
        bool found = true;

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
            if(d > inf - 100.0f)
            {
                // return false;
                found = false; // Did not find full path.
                endFace = bestFace;
                break;
            }

            for (int i = 0; i < 3; i++)
            {
                index_t v = m_mesh.GetFace(u).neighbours[i];
                if (v == NavMesh::InvalidIndex)
                    continue;
                if (m_nodes[v].closed) // TODO: This is bad if the navmesh is not a grid (maybe)
                    continue;
                if ((m_mesh.GetFace(v).flags & NavMesh::FaceActive) == 0)
                    continue;

                const float alt = d + m_mesh.GetDist(u, v);
                const float heuristic = rob::Distance(m_mesh.GetFaceCenter(m_mesh.GetFace(v)), end);

                if (alt < m_nodes[v].dist)
                {
                    m_nodes[v].dist = alt;
                    m_nodes[v].prev = u;
                }
                if (heuristic < bestHeuristicCost)
                {
                    bestHeuristicCost = heuristic;
                    bestFace = v;
                }
            }
        }

        index_t v = endFace;
        while (m_nodes[v].prev != NavMesh::InvalidIndex)
        {
            v = m_nodes[v].prev;
            m_path.len++;
        }

        if (m_path.len > MAX_PATH_LEN)
            return false;

        index_t u = endFace;
        for (int i = m_path.len - 1; i >= 0; i--)
        {
            m_path.path[i] = u;
            u = m_nodes[u].prev;
        }

//        rob::log::Debug("Nav: Nodes in node path: ", m_path.len);

        return found;
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
            return hit ? 0 : -1;
        }

        bool ShouldQueryParticleSystem(const b2ParticleSystem* particleSystem) override
        {
            B2_NOT_USED(particleSystem);
            return false;
        }
    };

    void Navigation::FindStraightPath(const vec2f &start, const vec2f &end, NavPath *path, bool fullPath)
    {
        int num = 0;
        vec2f candidates[3];
        vec2f dest = start;
        vec2f current = start;

        path->Clear();

//        path->AppendVertex(start.x, start.y);
        if (m_path.len <= 1 && fullPath)
        {
            path->AppendVertex(end.x, end.y);
            return;
        }

        for (size_t i = 0; i < m_path.len; i++)
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

        if (!fullPath)
        {
            path->AppendVertex(dest.x, dest.y);
            return;
        }

        PathRayCast rayCast;
        m_world->RayCast(&rayCast, ToB2(current), ToB2(end));
        if (!rayCast.hit)
        {
            path->AppendVertex(end.x, end.y);
        }
        else
        {
            path->AppendVertex(dest.x, dest.y);
            path->AppendVertex(end.x, end.y);
        }
    }

    bool Navigation::Navigate(const vec2f &start, const vec2f &end, NavPath *path)
    {
        index_t startFace = m_mesh.GetFaceIndex(start);
        index_t endFace = m_mesh.GetFaceIndex(end);

//        if (FindNodePath(start, end, startFace, endFace))
//        {
//            FindStraightPath(start, end, path);
//            return true;
//        }
//        return false;

        const bool found = FindNodePath(start, end, startFace, endFace);
        FindStraightPath(start, end, path, found);
        return found;
    }

    struct RayCastCb : public b2RayCastCallback
    {
        b2Body *body;
        //float32 fraction;
        uint16_t mask;
        uint16_t ignore;

        RayCastCb(uint16_t mask, uint16_t ignore)
            : body(nullptr), /*fraction(1e6),*/ mask(mask), ignore(ignore|SensorBit) { }

        float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float32 fraction) override
        {
            if ((fixture->GetFilterData().categoryBits & ignore) == 0)
                body = fixture->GetBody();
            else return -1;
            return fraction;

//            if (fixture->GetFilterData().categoryBits & mask)
//            {
//                body = fixture->GetBody();
//                return 0;
//                //this->fraction = fraction;
//                return fraction;
//            }
//            if (fixture->GetFilterData().categoryBits & ignore)
//            {
//                return -1;
//            }
//            //if (body && fraction < this->fraction)
//            //{
//                body = nullptr;
//                return 0;
//            //}
//            //return 1;
        }
    };

    b2Body *Navigation::RayCast(const vec2f &start, const vec2f &end, uint16_t mask/* = 0xffff */, uint16_t ignore/* = 0x0 */)
    {
        RayCastCb rayCast(mask, ignore);
        m_world->RayCast(&rayCast, ToB2(start), ToB2(end));

        return rayCast.body;
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
