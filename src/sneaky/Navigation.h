
#ifndef H_SNEAKY_NAVIGATION_H
#define H_SNEAKY_NAVIGATION_H

#include "Physics.h"
#include "rob/memory/Pool.h"

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

        enum FaceBits
        {
            FaceActive = 0x1,
        };

        struct Face
        {
            uint16_t vertices[3];
            uint16_t neighbours[3];
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
            uint16_t faces[8];
        };

        static const uint16_t MAX_FACES = 1024*24;
        static const uint16_t MAX_VERTICES = 1024*32;
    public:
        NavMesh();
        ~NavMesh();

        void Allocate(rob::LinearAllocator &alloc);
        void SetGrid(const b2World *world, const float halfSize, const float agentRadius);

        size_t GetFaceCount() const;
        const Face& GetFace(size_t index) const;

        size_t GetVertexCount() const;
        const Vert& GetVertex(size_t index) const;

        uint16_t GetFaceIndex(const vec2f &v) const;

        vec2f GetFaceCenter(const Face &f) const;
        vec2f GetEdgeCenter(uint16_t f, int edge) const;
        float GetDist(uint16_t f0, uint16_t f1, int neighborIndex, const vec2f &prevPoint) const;

    private:
        static bool TestPoint(const b2World *world, float x, float y);
        void AddVertex(float x, float y, bool active);
        uint16_t AddFace(uint16_t i0, uint16_t i1, uint16_t i2, bool active);

    private:
        size_t m_faceCount;
        Face *m_faces;

        size_t m_vertexCount;
        Vert *m_vertices;

        float m_gridSz;
        float m_halfSize;
    };

    void ToggleDistF();

    constexpr uint16_t MAX_PATH_LEN = 256;

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
            uint16_t len;
            uint16_t path[MAX_PATH_LEN];

            NodePath() : len(0) { }
        };

        struct Node
        {
            float dist;
            int prev;
            bool closed;
        };

    public:
        Navigation();
        ~Navigation();

        bool CreateNavMesh(rob::LinearAllocator &alloc, const b2World *world, const float worldHalfSize, const float agentRadius);

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

