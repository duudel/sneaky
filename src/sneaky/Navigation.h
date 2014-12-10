
#ifndef H_SNEAKY_NAVIGATION_H
#define H_SNEAKY_NAVIGATION_H

#include "Physics.h"
#include "NavMesh.h"

#include "rob/memory/Pool.h"
#include "rob/math/Random.h"

namespace rob
{
    class LinearAllocator;
    class Renderer;
} // rob

namespace sneaky
{

    constexpr size_t MAX_PATH_LEN = 128;

    class NavPath
    {
    public:
        NavPath();

        void Clear();

        void AppendVertex(float x, float y);
        void AppendVertex(const vec2f &v);
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
            vec2f pos;
            bool posCalculated;
            bool closed;
        };

    public:
        Navigation();
        ~Navigation();

        bool CreateNavMesh(rob::LinearAllocator &alloc, const b2World *world, const float worldHalfW, const float worldHalfH, const float agentRadius);

        const NavMesh& GetMesh() const { return m_mesh; }
        NavMesh& GetMesh() { return m_mesh; }

        bool IsWalkable(const vec2f &point) const
        {
            const index_t i = m_mesh.GetFaceIndex(point);
            return (i != NavMesh::InvalidIndex);

            const NavMesh::Face &f = m_mesh.GetFace(i);
            return (f.flags & NavMesh::FaceActive) != 0;
        }

        vec2f GetRandomNavigableWorldPoint(rob::Random &rand) const
        {
            const vec2f halfSize = m_mesh.GetHalfSize();

            vec2f point;
            point.x = rand.GetReal(-halfSize.x, halfSize.x);
            point.y = rand.GetReal(-halfSize.y, halfSize.y);
            m_mesh.GetClampedFaceIndex(&point);
            return point;
        }

        NavPath *ObtainNavPath();
        void ReturnNavPath(NavPath *path);

        bool Navigate(const vec2f &start, const vec2f &end, NavPath *path);

        b2Body *RayCast(const vec2f &start, const vec2f &end, uint16_t mask = 0xffff, uint16_t ignore = 0x0);

        void RenderMesh(rob::Renderer *renderer) const;
        void RenderPath(rob::Renderer *renderer, const NavPath *path) const;

    private:
        vec2f CalculateNodePos(index_t face, int edge, const vec2f &prevPos) const;
        bool FindNodePath(const vec2f &start, const vec2f &end, index_t startFace, index_t endFace);
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

