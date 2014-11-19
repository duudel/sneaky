
#include "GameObject.h"
#include "rob/renderer/Renderer.h"
#include "rob/graphics/Graphics.h"

namespace sneaky
{

    using namespace rob;

    GameObject::GameObject()
        : m_body(nullptr)
        , m_color(Color::White)
        , m_texture(InvalidHandle)
        , m_textureScale(1.0f)
        , m_renderLayer(0)
        , m_destroyed(false)
        , m_next(nullptr)
    { }

    GameObject::~GameObject()
    { }

    void GameObject::SetPosition(const vec2f &pos)
    { m_body->SetTransform(ToB2(pos), m_body->GetAngle()); }

    vec2f GameObject::GetPosition() const
    { return FromB2(m_body->GetPosition()); }

    vec2f GameObject::GetDimensions() const
    {
        const b2Fixture *fixture = m_body->GetFixtureList();
        const b2Shape *shape = fixture->GetShape();

        b2AABB aabb;
        b2Transform tr;
        tr.SetIdentity();

        shape->ComputeAABB(&aabb, tr, 0);
        while (fixture)
        {
            const b2Shape *shape = fixture->GetShape();
            for (int i = 0; i < shape->GetChildCount(); i++)
            {
                const b2Vec2 r(shape->m_radius, shape->m_radius);
                b2AABB shapeAabb;
                shape->ComputeAABB(&shapeAabb, tr, i);
                shapeAabb.lowerBound = shapeAabb.lowerBound + r;
                shapeAabb.upperBound = shapeAabb.upperBound - r;
                aabb.Combine(shapeAabb);
            }
            fixture = fixture->GetNext();
        }

        return FromB2(aabb.GetExtents());
    }

    void GameObject::SetColor(const Color &color)
    { m_color = color; }

    Color GameObject::GetColor() const
    { return m_color; }

    void GameObject::SetTexture(TextureHandle texture)
    { m_texture = texture; }

    TextureHandle GameObject::GetTexture() const
    { return m_texture; }

    void GameObject::SetLayer(int layer)
    { m_renderLayer = layer; }

    int GameObject::GetLayer() const
    { return m_renderLayer; }

    void GameObject::Update(const GameTime &gameTime)
    {
//        float dt = gameTime.GetDeltaSeconds();
    }

    void GameObject::Render(Renderer *renderer)
    {
        vec2f dim = GetDimensions();

        renderer->SetColor(m_color);

        const b2Fixture *fixture = m_body->GetFixtureList();
        const b2Shape *shape = fixture->GetShape();

        const mat4f model = FromB2Transform(m_body->GetTransform());
        renderer->SetModel(model);

        if (m_texture == InvalidHandle)
        {
            renderer->BindColorShader();
            switch (shape->GetType())
            {
            case b2Shape::e_polygon:
                renderer->DrawFilledRectangle(-dim.x, -dim.y, dim.x, dim.y);
                break;

            case b2Shape::e_circle:
                renderer->DrawFilledCircle(0.0f, 0.0f, dim.x);
                break;

            default:
                break;
            }
        }
        else
        {
            dim *= m_textureScale;
            renderer->GetGraphics()->SetUniform(renderer->GetGlobals().texture0, 1);
            renderer->GetGraphics()->BindTexture(1, m_texture);
            renderer->BindTextureShader();
            renderer->DrawTexturedRectangle(-dim.x, -dim.y, dim.x, dim.y);
        }
    }

    void GameObject::SetNext(GameObject *object)
    { m_next = object; }

    GameObject* GameObject::GetNext()
    { return m_next; }

} // sneaky
