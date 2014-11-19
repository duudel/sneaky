
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

    class GameObject
    {
    public:
        GameObject();
        ~GameObject();

        void SetPosition(const vec2f &pos);
        vec2f GetPosition() const;

        vec2f GetDimensions() const;

        void SetBody(b2Body *body) { m_body = body; }
        b2Body* GetBody() { return m_body; }

        void SetColor(const Color &color);
        Color GetColor() const;

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

        void Update(const GameTime &gameTime);
        void Render(rob::Renderer *renderer);

        void SetNext(GameObject *object);
        GameObject *GetNext();

    private:
        b2Body *m_body;

        Color m_color;
        rob::TextureHandle m_texture;
        float m_textureScale;
        int m_renderLayer;

        bool m_destroyed;

        GameObject *m_next;
    };

} // sneaky

#endif // H_DUCK_GAME_OBJECT_H

