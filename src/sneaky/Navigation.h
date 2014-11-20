
#ifndef H_SNEAKY_NAVIGATION_H
#define H_SNEAKY_NAVIGATION_H

#include "Physics.h"

namespace rob
{
    class LinearAllocator;
    class Renderer;
} // rob

namespace sneaky
{

    class NavMesh
    {
    public:
        static const uint16_t InvalidIndex = -1;
        struct Face
        {
            uint16_t vertices[3];
            uint16_t neighbours[3];
        };

        enum VertBits
        {
            VertActive = 0x1,
        };

        struct Vert
        {
            float x, y;
            uint32_t flags;
            uint16_t faces[8];
        };

        static const uint16_t MAX_FACES = 1024;
        static const uint16_t MAX_VERTICES = 1024*32;
    public:
        NavMesh();
        ~NavMesh();

        void Allocate(rob::LinearAllocator &alloc);
        void SetGrid(const b2World *world, const float halfSize, const float agentRadius);

        size_t GetVertexCount() const { return m_vertexCount; }
        const Vert& GetVertex(size_t index) const { return m_vertices[index]; }

    private:
        static bool TestPoint(const b2World *world, float x, float y);
        void AddVertex(float x, float y, bool active);

    private:
        size_t m_faceCount;
        Face *m_faces;

        size_t m_vertexCount;
        Vert *m_vertices;
    };

    class Navigation
    {
    public:
        Navigation();
        ~Navigation();

        bool CreateNavMesh(rob::LinearAllocator &alloc, const b2World *world, const float worldHalfSize, const float agentRadius);

        void RenderMesh(rob::Renderer *renderer);

    private:
        NavMesh m_mesh;
    };

} // sneaky

#endif // H_SNEAKY_NAVIGATION_H

