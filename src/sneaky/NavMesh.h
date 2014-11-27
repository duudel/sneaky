
#ifndef H_SNEAKY_NAV_MESH_H
#define H_SNEAKY_NAV_MESH_H

#include "Physics.h"

namespace rob
{
    class LinearAllocator;
} // rob

namespace sneaky
{


    typedef uint32_t index_t;

    class NavMesh
    {
    public:
        static const index_t InvalidIndex = -1;

        enum FaceBits
        {
            FaceActive = 0x1,
        };

        struct Face
        {
            index_t vertices[3];
            index_t neighbours[3];
            uint32_t flags;
        };

        enum VertBits
        {
            VertActive = 0x1,
        };

        struct Vert
        {
            float x, y;
            uint32_t flags;
            //index_t faces[8];
        };

        static const index_t MAX_FACES = 1024*64;
        static const index_t MAX_VERTICES = 1024*64;
    public:
        NavMesh();
        ~NavMesh();

        void Allocate(rob::LinearAllocator &alloc);
        void SetGrid(const b2World *world, const float halfW, const float halfH, const float agentRadius);

        vec2f GetHalfSize() const
        { return vec2f(m_halfW, m_halfH); }

        size_t GetFaceCount() const;
        const Face& GetFace(size_t index) const;

        size_t GetVertexCount() const;
        const Vert& GetVertex(size_t index) const;

        index_t GetFaceIndex(const vec2f &v) const;

        vec2f GetFaceCenter(const Face &f) const;
        vec2f GetEdgeCenter(index_t f, int edge) const;
        float GetDist(index_t f0, index_t f1) const;

    private:
        static bool TestPoint(const b2World *world, float x, float y);
        void AddVertex(float x, float y, bool active);
        index_t AddFace(index_t i0, index_t i1, index_t i2, bool active);

    private:
        size_t m_faceCount;
        Face *m_faces;

        size_t m_vertexCount;
        Vert *m_vertices;

        float m_gridSz;
        float m_halfW;
        float m_halfH;
    };

} // sneaky

#endif // H_SNEAKY_NAV_MESH_H

