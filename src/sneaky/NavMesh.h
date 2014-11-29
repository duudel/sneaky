
#ifndef H_SNEAKY_NAV_MESH_H
#define H_SNEAKY_NAV_MESH_H

#include "Physics.h"
#include <clipper.hpp>

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
        void Create(const b2World *world, const float halfW, const float halfH, const float agentRadius);
        void SetGrid(const b2World *world, const float halfW, const float halfH, const float agentRadius);

        vec2f GetHalfSize() const
        { return vec2f(m_halfW, m_halfH); }

        size_t GetFaceCount() const;
        const Face& GetFace(size_t index) const;

        size_t GetVertexCount() const;
        const Vert& GetVertex(size_t index) const;

        index_t GetFaceIndex(const vec2f &v) const;

        vec2f GetClosestPointOnFace(const Face &face, const vec2f &p) const;
        index_t GetClampedFaceIndex(vec2f *v) const;

        vec2f GetFaceCenter(const Face &f) const;
        vec2f GetEdgeCenter(index_t f, int edge) const;
        float GetDist(index_t f0, index_t f1) const;
        float GetDist(index_t f0, index_t f1, int edge) const;

        void GetPortalPoints(const index_t from, const index_t to, vec2f &left, vec2f &right) const;

        std::vector<std::vector<vec2f> > m_solids;
        std::vector<std::vector<vec2f> > m_holes;

    private:
        void TriangulatePath2(const ClipperLib::Path &path, const ClipperLib::Paths &holes, const float clipperScale);
        void TriangulatePath(const ClipperLib::Path &path, const ClipperLib::Paths &holes, const float clipperScale);

        int Refine();

        static bool TestPoint(const b2World *world, float x, float y);
        Vert* AddVertex(float x, float y, bool active);
        Vert* GetVertex(float x, float y, index_t *index);
        index_t AddFace(index_t i0, index_t i1, index_t i2);
        bool FaceContainsPoint(const Face &face, const vec2f &v) const;
        bool FaceHasEdge(const Face &face, index_t v0, index_t v1);
        void SetNeighbour(index_t fi, int ni, index_t fj, index_t v0, index_t v1);

    private:
        size_t m_faceCount;
        Face *m_faces;

        size_t m_vertexCount;
        Vert *m_vertices;

        struct VertexCache
        {
            Vert *m_v[MAX_VERTICES];
        };
        VertexCache m_vertCache;

        float m_gridSz;
        float m_halfW;
        float m_halfH;
    };

} // sneaky

#endif // H_SNEAKY_NAV_MESH_H

