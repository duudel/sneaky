
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
    { AppendVertex(vec2f(x, y)); }

    void NavPath::AppendVertex(const vec2f &v)
    {
        ROB_ASSERT(m_len < MAX_PATH_LEN);
        m_path[m_len++] = v;
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
        m_mesh.Create(world, worldHalfW, worldHalfH, agentRadius);
        m_nodes = alloc.AllocateArray<Node>(m_mesh.GetFaceCount());
        m_np.SetMemory(alloc.AllocateArray<NavPath>(16), rob::GetArraySize<NavPath>(16));
        return false;
    }

    NavPath *Navigation::ObtainNavPath()
    { return m_np.Obtain(); }

    void Navigation::ReturnNavPath(NavPath *path)
    { m_np.Return(path); }

    static inline vec2f ClosestPointOnEdge(const vec2f &a, const vec2f &b, const vec2f &p)
    {
        const vec2f e = b - a;
        const vec2f v = p - a;
        const float d = e.Dot(e);
        float t = e.Dot(v);
        if (d > 0) t /= d;
        if (t < 0) t = 0;
        else if (t > 1) t = 1;
        return a + t * e;
    }

    vec2f Navigation::CalculateNodePos(index_t face, int edge, const vec2f &prevPos) const
    {
        static const int nextV[] = { 1, 2, 0 };
        const NavMesh::Face &f = m_mesh.GetFace(face);
        const NavMesh::Vert &vert0 = m_mesh.GetVertex(f.vertices[edge]);
        const NavMesh::Vert &vert1 = m_mesh.GetVertex(f.vertices[nextV[edge]]);
        return ClosestPointOnEdge(vec2f(vert0.x, vert0.y), vec2f(vert1.x, vert1.y), prevPos);
    }

    bool Navigation::FindNodePath(const vec2f &start, const vec2f &end, index_t startFace, index_t endFace)
    {
//        rob::log::Info("Nav: Start node: ", startFace, ", end node: ", endFace, ", faces:", m_mesh.GetFaceCount());

        m_path.len = 0;
        if (startFace == endFace) return true;

        const float inf = 1e6f;
        const int nodeCount = m_mesh.GetFaceCount();

        int n = nodeCount;

        for (int i = 0; i < nodeCount; i++)
        {
            m_nodes[i].dist = inf;
            m_nodes[i].prev = NavMesh::InvalidIndex;
            m_nodes[i].pos.x = 0.0f;
            m_nodes[i].pos.y = 0.0f;
            m_nodes[i].posCalculated = false;
            m_nodes[i].closed = false;
        }

        m_nodes[startFace].dist = 0.0f;
        m_nodes[startFace].pos = start;
        m_nodes[startFace].posCalculated = true;
        m_nodes[endFace].pos = end;
        m_nodes[endFace].posCalculated = true;

        index_t bestFace = startFace;
        float bestHeuristicCost = rob::Distance2(start, end);
        bool found = true;

        while (n > 0)
        {
            index_t u = NavMesh::InvalidIndex;
            float d = inf;
            for (int i = 0; i < nodeCount; i++)
            {
                if (m_nodes[i].closed) continue;
                if (m_nodes[i].dist < d)
                {
                    u = i;
                    d = m_nodes[i].dist;
                }
            }

            if(u == endFace) break;
            if(u == NavMesh::InvalidIndex)
            {
                found = false; // Did not find full path.
                endFace = bestFace;
                break;
            }

            m_nodes[u].closed = true;
            n--;

            for (int i = 0; i < 3; i++)
            {
                index_t v = m_mesh.GetFace(u).neighbours[i];
                if (v == NavMesh::InvalidIndex)
                    continue;
                if (m_nodes[v].closed) // TODO: This is bad if the navmesh is not a grid (maybe)
                    continue;

                if (!m_nodes[v].posCalculated)
                {
                    m_nodes[v].pos = m_mesh.GetEdgeCenter(u, i);
//                    m_nodes[v].pos = CalculateNodePos(u, i, m_nodes[u].pos);
                    m_nodes[v].posCalculated = true;
                }

//                const float alt = d + m_mesh.GetDist(u, v);
//                const float alt = d + m_mesh.GetDist(u, v, i);
                const float alt = d + rob::Distance(m_nodes[u].pos, m_nodes[v].pos);
                const float heuristic = rob::Distance2(m_mesh.GetFaceCenter(m_mesh.GetFace(v)), end);

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
        m_path.len++;
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

//        rob::log::Info("Path length: ", m_nodes[endFace].dist);

//        rob::log::Info("Nav: Nodes in node path: ", m_path.len);

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

    static inline float TriArea(const vec2f &a, const vec2f &b, const vec2f &c)
    {
        const vec2f ab = b - a;
        const vec2f ac = c - a;
        return (ac.x * ab.y - ab.x * ac.y);
    }

    static inline float DistanceFromEdge2(const vec2f &a, const vec2f &b, const vec2f &p)
    {
        const vec2f e = b - a;
        const vec2f v = p - a;
        const float d = e.Dot(e);
        float t = e.Dot(v);
        if (d > 0) t /= d;
        if (t < 0) t = 0;
        else if (t > 1) t = 1;
        const vec2f offt = p + t * e - p;
        return offt.Length2();
    }

    void Navigation::FindStraightPath(const vec2f &start, const vec2f &end, NavPath *path, bool fullPath)
    {
        path->Clear();
        path->AppendVertex(start.x, start.y);
        if (m_path.len > 1)
        {
            vec2f apex, portalLeft, portalRight;
            apex = portalLeft = portalRight = start;
            int apexInd = 0;
            int leftInd = 0;
            int rightInd = 0;

            for (size_t i = 0; i < m_path.len; i++)
            {
                vec2f left, right;
                if (i + 1 < m_path.len)
                {
                    m_mesh.GetPortalPoints(m_path.path[i], m_path.path[i + 1], left, right);

                    if (i == 0 && DistanceFromEdge2(apex, left, right) < 0.001f * 0.001f)
                        continue;
                }
                else
                {
                    left = right = end;
                }

                // Right vertex:
                if (TriArea(apex, portalRight, right) <= 0.0f)
                {
                    if (vec2f::Equals(apex, portalRight) || TriArea(apex, portalLeft, right) > 0.0f)
                    {
                        portalRight = right;
                        rightInd = i;
                    }
                    else
                    {
                        apex = portalLeft;
                        apexInd = leftInd;

                        path->AppendVertex(apex);

                        portalLeft = portalRight = apex;
                        leftInd = rightInd = apexInd;
                        // Restart
                        i = apexInd;
                        continue;
                    }
                }

                // Left vertex:
                if (TriArea(apex, portalLeft, left) >= 0.0f)
                {
                    if (vec2f::Equals(apex, portalLeft) || TriArea(apex, portalRight, left) < 0.0f)
                    {
                        portalLeft = left;
                        leftInd = i;
                    }
                    else
                    {
                        apex = portalRight;
                        apexInd = rightInd;

                        path->AppendVertex(apex);

                        portalLeft = portalRight = apex;
                        leftInd = rightInd = apexInd;
                        // Restart
                        i = apexInd;
                        continue;
                    }
                }
            }
        }
        path->AppendVertex(end.x, end.y);
    }

    bool Navigation::Navigate(const vec2f &start, const vec2f &end, NavPath *path)
    {
        vec2f s = start, e = end;
        const index_t startFace = m_mesh.GetClampedFaceIndex(&s);
        const index_t endFace = m_mesh.GetClampedFaceIndex(&e);

        const bool found = FindNodePath(s, e, startFace, endFace);
        if (!found) // If no full path was found, fix the end point
        {
            const NavMesh::Face &lastFace = m_mesh.GetFace(m_path.path[m_path.len - 1]);
            e = m_mesh.GetClosestPointOnFace(lastFace, e);
        }
        FindStraightPath(s, e, path, found);
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


    void RenderClippedPath(rob::Renderer *renderer, const std::vector<vec2f> &path)
    {
        for (size_t i = 1; i < path.size(); i++)
        {
            const vec2f v0(path[i - 1]);
            const vec2f v1(path[i]);
            renderer->DrawLine(v0.x, v0.y, v1.x, v1.y);
        }
        const vec2f v0(path[path.size() - 1]);
        const vec2f v1(path[0]);
        renderer->DrawLine(v0.x, v0.y, v1.x, v1.y);
    }

    void Navigation::RenderMesh(rob::Renderer *renderer) const
    {
        renderer->SetModel(mat4f::Identity);
        renderer->BindColorShader();

        static const rob::Color colors[] = {
            rob::Color::LightGreen,
            rob::Color::Yellow,
            rob::Color::DarkGreen,
            rob::Color::Orange,
            rob::Color::LightBlue,
            rob::Color::DarkBlue,
            rob::Color::LightRed,
            rob::Color::DarkRed
        };

        renderer->SetColor(rob::Color::LightGreen);
        const size_t faceCount = m_mesh.GetFaceCount();
        for (size_t i = 0; i < faceCount; i++)
        {
            const NavMesh::Face &f = m_mesh.GetFace(i);
            const NavMesh::Vert &v0 = m_mesh.GetVertex(f.vertices[0]);
            const NavMesh::Vert &v1 = m_mesh.GetVertex(f.vertices[1]);
            const NavMesh::Vert &v2 = m_mesh.GetVertex(f.vertices[2]);

//        if (f.flags == 0)
//            renderer->SetColor(rob::Color::LightGreen);
//        else if (f.flags == 1)
//            renderer->SetColor(rob::Color::Yellow);
//        else
//            renderer->SetColor(rob::Color::DarkGreen);

            renderer->SetColor(colors[f.flags & 0x7]);

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

            const vec2f &np = m_nodes[m_path.path[i]].pos;
            renderer->DrawCircle(np.x, np.y, 0.2f);

            renderer->DrawLine(v0.x, v0.y, v1.x, v1.y);
            renderer->DrawLine(v1.x, v1.y, v2.x, v2.y);
            renderer->DrawLine(v2.x, v2.y, v0.x, v0.y);
        }

        if (m_path.len > 0)
        {
            renderer->SetColor(rob::Color::Blue);
            const NavMesh::Face &f = m_mesh.GetFace(m_path.path[0]);
            for (int n = 0; n < 3; n++)
            {
                if (f.neighbours[n] == NavMesh::InvalidIndex) continue;

                const NavMesh::Face &nf = m_mesh.GetFace(f.neighbours[n]);
                const NavMesh::Vert &v0 = m_mesh.GetVertex(nf.vertices[0]);
                const NavMesh::Vert &v1 = m_mesh.GetVertex(nf.vertices[1]);
                const NavMesh::Vert &v2 = m_mesh.GetVertex(nf.vertices[2]);

                renderer->DrawLine(v0.x, v0.y, v1.x, v1.y);
                renderer->DrawLine(v1.x, v1.y, v2.x, v2.y);
                renderer->DrawLine(v2.x, v2.y, v0.x, v0.y);
            }
        }

//        renderer->SetColor(rob::Color::Orange);
//        for (size_t i = 0; i < m_mesh.m_solids.size(); i++)
//        {
//            RenderClippedPath(renderer, m_mesh.m_solids[i]);
//        }
//
//        renderer->SetColor(rob::Color::Red);
//        for (size_t i = 0; i < m_mesh.m_holes.size(); i++)
//        {
//            RenderClippedPath(renderer, m_mesh.m_holes[i]);
//        }
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
