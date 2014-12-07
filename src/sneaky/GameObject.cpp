
#include "GameObject.h"
#include "Brain.h"

#include "rob/renderer/Renderer.h"
#include "rob/graphics/Graphics.h"

namespace sneaky
{

    using namespace rob;



    Drawable::Drawable()
        : m_object(nullptr)
        , m_texture(InvalidHandle)
        , m_scale(1.0f)
        , m_additive(false)
        , m_layer(0)
    { }

    void Drawable::SetObject(GameObject *object)
    { m_object = object; }

    void Drawable::Draw(Renderer *renderer) const
    {
        if (m_texture == InvalidHandle) return;

//        if (m_brain && m_debugDraw)
//            renderer->SetColor(m_debugColor);
//        else
//            renderer->SetColor(m_color);
        renderer->SetColor(Color::White);

        const mat4f model = FromB2Transform(m_object->GetBody()->GetTransform());
        renderer->SetModel(model);

        const vec2f dim = m_object->GetDimensions() * m_scale;
        renderer->GetGraphics()->SetUniform(renderer->GetGlobals().texture0, 1);
        renderer->GetGraphics()->BindTexture(1, m_texture);
        renderer->BindTextureShader();
        renderer->DrawTexturedRectangle(-dim.x, -dim.y, dim.x, dim.y);
    }

    void Drawable::SetTexture(TextureHandle texture)
    { m_texture = texture; }

    void Drawable::SetTextureScale(float scale)
    { m_scale = scale; }

    void Drawable::SetLayer(int layer)
    { m_layer = layer; }

    int Drawable::GetLayer() const
    { return m_layer; }

    void Drawable::SetAdditive(bool additive)
    { m_additive = additive; }

    bool Drawable::IsAdditive() const
    { return m_additive; }



    GameObject::GameObject()
        : m_body(nullptr)
        , m_brain(nullptr)
        , m_color(Color::White)
        , m_debugColor(Color::White)
        , m_texture(InvalidHandle)
        , m_textureScale(1.0f)
        , m_renderLayer(0)
        , m_destroyed(false)
        , m_debugDraw(false)
        , m_drawableCount(0)
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
        while (fixture && fixture->IsSensor())
            fixture = fixture->GetNext();
        if (!fixture) return vec2f::Zero;

        const b2Shape *shape = fixture->GetShape();

        b2AABB aabb;
        b2Transform tr;
        tr.SetIdentity();

        shape->ComputeAABB(&aabb, tr, 0);
        while (fixture)
        {
            if (!fixture->IsSensor())
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
            }
            fixture = fixture->GetNext();
        }

        return FromB2(aabb.GetExtents());
    }

    void GameObject::SetBrain(Brain *brain)
    {
        m_brain = brain;
        m_brain->SetOwner(this);
        m_brain->OnInitialize();
    }

    Brain* GameObject::GetBrain()
    { return m_brain; }

    Drawable* GameObject::AddDrawable(TextureHandle texture)
    {
        ROB_ASSERT(m_drawableCount < MAX_DRAWABLES);
        Drawable *drawable = &m_drawables[m_drawableCount++];
        drawable->SetObject(this);
        drawable->SetTexture(texture);
        return drawable;
    }

    Drawable* GameObject::AddDrawable(TextureHandle texture, float scale, bool additive, int layer)
    {
        Drawable *drawable = AddDrawable(texture);
        drawable->SetTextureScale(scale);
        drawable->SetAdditive(additive);
        drawable->SetLayer(layer);
        return drawable;
    }

    const Drawable* GameObject::GetDrawables() const
    { return m_drawables; }

    size_t GameObject::GetDrawableCount() const
    { return m_drawableCount; }

    void GameObject::SetColor(const Color &color)
    { m_color = color; }

    Color GameObject::GetColor() const
    { return m_color; }

    void GameObject::SetDebugColor(const Color &color)
    { m_debugColor = color; }

    Color GameObject::GetDebugColor() const
    { return m_debugColor; }

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

        if (m_brain && m_debugDraw)
            renderer->SetColor(m_debugColor);
        else
            renderer->SetColor(m_color);

        const mat4f model = FromB2Transform(m_body->GetTransform());
        renderer->SetModel(model);

        if (m_texture == InvalidHandle)
        {
//            const b2Fixture *fixture = m_body->GetFixtureList();
//            const b2Shape *shape = fixture->GetShape();
//            renderer->BindColorShader();
//            switch (shape->GetType())
//            {
//            case b2Shape::e_polygon:
//                renderer->DrawFilledRectangle(-dim.x, -dim.y, dim.x, dim.y);
//                break;
//
//            case b2Shape::e_circle:
//                renderer->DrawFilledCircle(0.0f, 0.0f, dim.x);
//                break;
//
//            default:
//                break;
//            }
        }
        else
        {
            dim *= m_textureScale;
            renderer->GetGraphics()->SetUniform(renderer->GetGlobals().texture0, 1);
            renderer->GetGraphics()->BindTexture(1, m_texture);
            renderer->BindTextureShader();
            renderer->DrawTexturedRectangle(-dim.x, -dim.y, dim.x, dim.y);
        }

        if (m_brain) m_brain->Render(renderer);
        if (m_brain && m_debugDraw) m_brain->DebugRender(renderer);
    }

    void GameObject::SetNext(GameObject *object)
    { m_next = object; }

    GameObject* GameObject::GetNext()
    { return m_next; }

} // sneaky
