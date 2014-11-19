
#include "B2DebugDraw.h"
#include "Physics.h"
#include "rob/renderer/Renderer.h"

namespace sneaky
{

    using namespace rob;

    DebugDraw::DebugDraw(Renderer *renderer)
        : m_renderer(renderer)
    { }

    DebugDraw::~DebugDraw()
    { }

    void DebugDraw::DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color)
    {
        m_renderer->SetColor(Color(color.r, color.g, color.b));

        const b2Vec2 &p0 = vertices[vertexCount-1];
        const b2Vec2 &p1 = vertices[0];
        m_renderer->DrawLine(p0.x, p0.y, p1.x, p1.y);

        for (int32 i = 1; i < vertexCount; i++)
        {
            const b2Vec2 &p0 = vertices[i-1];
            const b2Vec2 &p1 = vertices[i];
            m_renderer->DrawLine(p0.x, p0.y, p1.x, p1.y);
        }
    }

    void DebugDraw::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color)
    {
        m_renderer->SetColor(Color(color.r, color.g, color.b));

        const b2Vec2 &p0 = vertices[vertexCount-1];
        const b2Vec2 &p1 = vertices[0];
        m_renderer->DrawLine(p0.x, p0.y, p1.x, p1.y);

        for (int32 i = 1; i < vertexCount; i++)
        {
            const b2Vec2 &p0 = vertices[i-1];
            const b2Vec2 &p1 = vertices[i];
            m_renderer->DrawLine(p0.x, p0.y, p1.x, p1.y);
        }
    }

    void DebugDraw::DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color)
    {
        m_renderer->SetColor(Color(color.r, color.g, color.b));
        m_renderer->DrawCircle(center.x, center.y, radius);
    }

    void DebugDraw::DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color)
    {
        m_renderer->SetColor(Color(color.r, color.g, color.b));
        m_renderer->DrawFilledCircle(center.x, center.y, radius);
    }

    void DebugDraw::DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color)
    {
        m_renderer->SetColor(Color(color.r, color.g, color.b));
        m_renderer->DrawLine(p1.x, p1.y, p2.x, p2.y);
    }

    void DebugDraw::DrawTransform(const b2Transform& xf)
    {
        const float scale = 0.4f;

        const vec2f origin = FromB2(xf.p);
        const vec2f xAxis = FromB2(xf.q.GetXAxis()) * scale;
        const vec2f yAxis = FromB2(xf.q.GetYAxis()) * scale;

        const vec2f x0 = origin - xAxis;
        const vec2f x1 = origin + xAxis;
        const vec2f y0 = origin - yAxis;
        const vec2f y1 = origin + yAxis;

        m_renderer->SetColor(Color(1.0f, 0.0f, 0.0f));
        m_renderer->DrawLine(x0.x, x0.y, x1.x, x1.y);
        m_renderer->SetColor(Color(0.0f, 1.0f, 0.0f));
        m_renderer->DrawLine(y0.x, y0.y, y1.x, y1.y);
    }

    void DebugDraw::DrawParticles(const b2Vec2 *centers, float32 radius, const b2ParticleColor *colors, int32 count)
    {
        for (int32 i = 0; i < count; i++)
        {
            const b2Vec2 &center = centers[i];
            const b2Color color = colors[i].GetColor();
            m_renderer->SetColor(Color(color.r, color.g, color.b));
            m_renderer->DrawFilledCircle(center.x, center.y, radius);
        }
    }

} // sneaky
