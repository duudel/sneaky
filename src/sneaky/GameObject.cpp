
#include "GameObject.h"
#include "Brain.h"

#include "rob/renderer/Renderer.h"
#include "rob/graphics/Graphics.h"

namespace sneaky
{

    using namespace rob;

    GameObject::GameObject()
        : m_body(nullptr)
        , m_brain(nullptr)
        , m_color(Color::White)
        , m_texture(InvalidHandle)
        , m_textureScale(1.0f)
        , m_renderLayer(0)
        , m_destroyed(false)
        , m_next(nullptr)
    { }

    GameObject::~GameObject()
    {
        if (m_brain) m_brain->~Brain();
    }

    void GameObject::SetPosition(const vec2f &pos)
    { m_body->SetTransform(ToB2(pos), m_body->GetAngle()); }

    vec2f GameObject::GetPosition() const
    { return FromB2(m_body->GetPosition()); }

    void GameObject::MoveLocal(const vec2f &delta)
    {
        const vec2f globalDelta = FromB2(m_body->GetWorldVector(ToB2(delta)));
        MoveGlobal(globalDelta);
    }

    void GameObject::MoveGlobal(const vec2f &delta)
    { SetPosition(GetPosition() + delta); }

    void GameObject::SetRotation(const vec2f &dir)
    {
        m_body->SetTransform(ToB2(GetPosition()), b2Atan2(-dir.x, dir.y));
//        m_body->SetTransform(ToB2(GetPosition()), b2Atan2(dir.y, dir.x));
    }

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

    void GameObject::SetBrain(Brain *brain)
    { m_brain = brain; m_brain->SetOwner(this); }

    Brain* GameObject::GetBrain()
    { return m_brain; }

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
        if (m_brain) m_brain->Update(gameTime);
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

        if (m_brain) m_brain->DebugRender(renderer);
    }

    void GameObject::SetNext(GameObject *object)
    { m_next = object; }

    GameObject* GameObject::GetNext()
    { return m_next; }

} // sneaky
