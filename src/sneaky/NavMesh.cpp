
#include "NavMesh.h"

#include "rob/memory/LinearAllocator.h"
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

    index_t NavMesh::GetFaceIndex(const vec2f &v) const
    {
        const int gridW = (m_halfW * 2.0f) / m_gridSz;
        const int gridH = (m_halfH * 2.0f) / m_gridSz;
        const vec2f p = v + vec2f(m_halfW, m_halfH);
        int x = (p.x + 0.5f) / m_gridSz;
        int y = (p.y + 0.5f) / m_gridSz;
        x = rob::Clamp(x, 0, gridW - 1);
        y = rob::Clamp(y, 0, gridH - 1);
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

    vec2f NavMesh::GetEdgeCenter(index_t f, int edge) const
    {
        const Face &face = GetFace(f);
        const NavMesh::Vert &v0 = GetVertex(face.vertices[edge]);
        const NavMesh::Vert &v1 = GetVertex(face.vertices[((edge + 2) & 3) - 1]);
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

    void NavMesh::AddVertex(float x, float y, bool active)
    {
        ROB_ASSERT(m_vertexCount < MAX_VERTICES);
        Vert &v = m_vertices[m_vertexCount++];
        v.x = x;
        v.y = y;
        v.flags = 0;
        v.flags |= active ? VertActive : 0x0;
//        for (size_t i = 0; i < 8; i++) v.faces[i] = InvalidIndex;
    }

    index_t NavMesh::AddFace(index_t i0, index_t i1, index_t i2, bool active)
    {
        ROB_ASSERT(m_faceCount < MAX_FACES);
        const index_t faceI = m_faceCount++;
        Face &f = m_faces[faceI];
        f.vertices[0] = i0;
        f.vertices[1] = i1;
        f.vertices[2] = i2;
        for (size_t i = 0; i < 3; i++) f.neighbours[i] = InvalidIndex;
        f.flags = active ? FaceActive : 0;
        return faceI;
    }

} // sneaky
