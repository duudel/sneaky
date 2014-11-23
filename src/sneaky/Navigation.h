
#ifndef H_SNEAKY_NAVIGATION_H
#define H_SNEAKY_NAVIGATION_H

#include "Physics.h"
#include "rob/memory/Pool.h"
#include "rob/math/Random.h"

namespace rob
{
    class LinearAllocator;
    class Renderer;
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

    void ToggleDistF();

    constexpr size_t MAX_PATH_LEN = 256;

    class NavPath
    {
    public:
        NavPath();

        void Clear();

        void AppendVertex(float x, float y);
        void ReplaceVertex(size_t index, float x, float y);
        void InsertVertex(size_t index, float x, float y);

        bool TryInsertVertex(size_t index, const vec2f &v, float maxReplaceDist);

        size_t GetLength() const;
        const vec2f &GetVertex(size_t index) const;

        const vec2f &GetDestination() const;

        bool IsEmpty() const;

    private:
        size_t m_len;
        vec2f m_path[MAX_PATH_LEN];
    };

    class Navigation
    {
        struct NodePath
        {
            size_t len;
            index_t path[MAX_PATH_LEN];

            NodePath() : len(0) { }
        };

        struct Node
        {
            float dist;
            index_t prev;
            bool closed;
        };

    public:
        Navigation();
        ~Navigation();

        bool CreateNavMesh(rob::LinearAllocator &alloc, const b2World *world, const float worldHalfW, const float worldHalfH, const float agentRadius);

        bool IsWalkable(const vec2f &point) const
        {
            const index_t i = m_mesh.GetFaceIndex(point);
            const NavMesh::Face &f = m_mesh.GetFace(i);
            return (f.flags & NavMesh::FaceActive) != 0;
        }

        vec2f GetRandomNavigableWorldPoint(rob::Random &rand) const
        {
            const vec2f halfSize = m_mesh.GetHalfSize();

            vec2f point;
            do
            {
                point.x = rand.GetReal(-halfSize.x, halfSize.x);
                point.y = rand.GetReal(-halfSize.y, halfSize.y);
            } while (!IsWalkable(point));

            return point;
        }

        NavPath *ObtainNavPath();
        void ReturnNavPath(NavPath *path);

        bool Navigate(const vec2f &start, const vec2f &end, NavPath *path);

        b2Body *RayCast(const vec2f &start, const vec2f &end, uint16_t mask = 0xffff, uint16_t ignore = 0x0);

        void RenderMesh(rob::Renderer *renderer) const;
        void RenderPath(rob::Renderer *renderer, const NavPath *path) const;

    private:
        bool FindNodePath(const vec2f &start, const vec2f &end, uint16_t startFace, uint16_t endFace);
        void FindStraightPath(const vec2f &start, const vec2f &end, NavPath *path, bool fullPath);

    private:
        const b2World *m_world;
        NavMesh m_mesh;
        NodePath m_path;
        Node *m_nodes;
        rob::Pool<NavPath> m_np;
    };

} // sneaky

#endif // H_SNEAKY_NAVIGATION_H

