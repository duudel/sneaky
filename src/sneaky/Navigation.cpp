
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
        const size_t sideVerts = size_t(halfSize / agentRadius) * 2 + 1;
        const float gridSz = agentRadius;

        const size_t vertCount = sideVerts * sideVerts;
        rob::log::Debug("NavMesh: ", vertCount, " vertices");
        ROB_ASSERT(vertCount <= MAX_VERTICES);

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
        Vert *v = &m_vertices[m_vertexCount++];
        v->x = x;
        v->y = y;
        v->flags = 0;
        v->flags |= active ? VertActive : 0x0;
        for (size_t i = 0; i < 8; i++) v->faces[i] = InvalidIndex;
    }



    Navigation::Navigation()
        : m_mesh()
    { }

    Navigation::~Navigation()
    { }

    bool Navigation::CreateNavMesh(rob::LinearAllocator &alloc, const b2World *world, const float worldHalfSize, const float agentRadius)
    {
        m_mesh.Allocate(alloc);
        m_mesh.SetGrid(world, worldHalfSize, agentRadius);
        return false;
    }

    void Navigation::RenderMesh(rob::Renderer *renderer)
    {
        renderer->SetModel(mat4f::Identity);
        renderer->BindColorShader();
        renderer->SetColor(rob::Color::LightBlue);

        const size_t vertCount = m_mesh.GetVertexCount();
        for (size_t i = 0; i < vertCount; i+=2)
        {
            const NavMesh::Vert &v = m_mesh.GetVertex(i);
//            if (v.x >= -20.0f && v.x < 20.0f)
            if (v.flags & NavMesh::VertActive)
                renderer->SetColor(rob::Color(0.0f, 0.5f, 1.0f));
            else
                renderer->SetColor(rob::Color::Red);
            renderer->DrawFilledCircle(v.x, v.y, 0.10f);
        }
//        renderer->DrawFilledCircle(0.0f, 0.0f, 0.25f);
    }

} // sneaky
