
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

        for (size_t y = 0; y < sideVerts - 1; y++)
        {
            for (size_t x = 0; x < sideVerts - 1; x++)
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
                if (v0.flags & v1.flags & v2.flags & v3.flags & VertActive)
                {
                    const uint16_t fi0 = AddFace(i0, i1, i2);
                    const uint16_t fi1 = AddFace(i0, i2, i3);
                    Face &f0 = m_faces[fi0];
                    Face &f1 = m_faces[fi1];
                    f0.neighbours[0] = fi1;
                    f1.neighbours[0] = fi0;
                }
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
        Vert &v = m_vertices[m_vertexCount++];
        v.x = x;
        v.y = y;
        v.flags = 0;
        v.flags |= active ? VertActive : 0x0;
        for (size_t i = 0; i < 8; i++) v.faces[i] = InvalidIndex;
    }

    uint16_t NavMesh::AddFace(uint16_t i0, uint16_t i1, uint16_t i2)
    {
        ROB_ASSERT(m_faceCount < MAX_FACES);
        const uint16_t faceI = m_faceCount++;
        Face &f = m_faces[faceI];
        f.vertices[0] = i0;
        f.vertices[1] = i1;
        f.vertices[2] = i2;
        for (size_t i = 0; i < 3; i++) f.neighbours[i] = InvalidIndex;
        return faceI;
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

        renderer->SetColor(rob::Color::LightGreen);
        const size_t faceCount = m_mesh.GetFaceCount();
        for (size_t i = 0; i < faceCount; i++)
        {
            const NavMesh::Face &f = m_mesh.GetFace(i);
            const NavMesh::Vert &v0 = m_mesh.GetVertex(f.vertices[0]);
            const NavMesh::Vert &v1 = m_mesh.GetVertex(f.vertices[1]);
            const NavMesh::Vert &v2 = m_mesh.GetVertex(f.vertices[2]);

            renderer->DrawLine(v0.x, v0.y, v1.x, v1.y);
            renderer->DrawLine(v1.x, v1.y, v2.x, v2.y);
            renderer->DrawLine(v2.x, v2.y, v0.x, v0.y);
        }
    }

} // sneaky
