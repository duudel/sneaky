
#ifndef H_DUCK_GAME_OBJECT_H
#define H_DUCK_GAME_OBJECT_H

#include "Physics.h"

#include "rob/application/GameTime.h"
#include "rob/graphics/GraphicsTypes.h"
#include "rob/renderer/Color.h"
#include "rob/Assert.h"

namespace rob
{
    class Renderer;
} // rob

namespace sneaky
{

    using rob::vec2f;
    using rob::GameTime;
    using rob::Color;

    class Brain;

    class Drawable
    {
    public:
        Drawable();

        void SetTexture(rob::TextureHandle texture);
        void SetTextureScale(float scale);

        void SetLayer(int layer);
        int GetLayer() const;

        void SetAdditive(bool additive);
        bool IsAdditive() const;

    private:
//        Color m_color;
        rob::TextureHandle m_texture;
        float m_scale;
        bool m_additive;
        int m_layer;
    };

    class GameObject
    {
    public:
        GameObject();
        ~GameObject();

        void SetPosition(const vec2f &pos);
        vec2f GetPosition() const;

        vec2f GetForward() const
        { return FromB2(m_body->GetWorldVector(b2Vec2(0.0f, 1.0f))); }
        vec2f GetRight() const
        { return FromB2(m_body->GetWorldVector(b2Vec2(1.0f, 0.0f))); }

        void MoveLocal(const vec2f &delta);
        void MoveGlobal(const vec2f &delta);

        void SetRotation(const vec2f &dir);

        vec2f GetDimensions() const;

        void SetBody(b2Body *body) { m_body = body; }
        b2Body* GetBody() { return m_body; }

        void SetBrain(Brain *brain);
        Brain* GetBrain();

        Drawable* AddDrawable(rob::TextureHandle texture);
        Drawable* AddDrawable(rob::TextureHandle texture, float scale, bool additive, int layer);
        const Drawable* GetDrawables() const;
        size_t GetDrawableCount() const;

        void SetColor(const Color &color);
        Color GetColor() const;
        void SetDebugColor(const Color &color);
        Color GetDebugColor() const;

        void SetTexture(rob::TextureHandle texture);
        rob::TextureHandle GetTexture() const;

        void SetTextureScale(float scale)
        { m_textureScale = scale; }

        void SetLayer(int layer);
        int GetLayer() const;

        void SetDestroyed(bool destroyed)
        { m_destroyed = destroyed; }
        bool IsDestroyed() const
        { return m_destroyed; }

        void SetDebugDraw(bool debugDraw)
        { m_debugDraw = debugDraw; }
        bool IsDebugDraw() const
        { return m_debugDraw; }

        void Update(const GameTime &gameTime);
        void Render(rob::Renderer *renderer);

        void SetNext(GameObject *object);
        GameObject *GetNext();

    private:
        b2Body *m_body;
        Brain *m_brain;

        Color m_color;
        Color m_debugColor;
        rob::TextureHandle m_texture;
        float m_textureScale;
        int m_renderLayer;

        bool m_destroyed;
        bool m_debugDraw;

        static const size_t MAX_DRAWABLES = 4;

        Drawable m_drawables[MAX_DRAWABLES];
        size_t m_drawableCount;

        GameObject *m_next;
    };

} // sneaky

#endif // H_DUCK_GAME_OBJECT_H

