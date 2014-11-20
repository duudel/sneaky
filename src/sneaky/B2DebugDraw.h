
#ifndef H_SNEAKY_B2_DEBUG_DRAW_H
#define H_SNEAKY_B2_DEBUG_DRAW_H

#include "Box2D/Box2D.h"

namespace rob
{
    class Renderer;
} // rob

namespace sneaky
{

    class DebugDraw : public b2Draw
    {
    public:
        explicit DebugDraw(rob::Renderer *renderer);
        virtual ~DebugDraw();

        void DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color);
        void DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color);
        void DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color);
        void DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color);
        void DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color);
        void DrawTransform(const b2Transform& xf);
        void DrawParticles(const b2Vec2 *centers, float32 radius, const b2ParticleColor *colors, int32 count);

    private:
        rob::Renderer *m_renderer;
    };

} // sneaky

#endif // H_SNEAKY_B2_DEBUG_DRAW_H


