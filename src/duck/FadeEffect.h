
#ifndef H_DUCK_FADE_EFFECT_H
#define H_DUCK_FADE_EFFECT_H

#include "rob/renderer/Color.h"

namespace rob
{
    class Renderer;
} // rob

namespace sneaky
{

    class FadeEffect
    {
    public:
        explicit FadeEffect(const rob::Color &color);

        void SetFadeAcceleration(float acc);
        void Activate(float delta);
        void Reset();

        void Update(const float deltaTime);
        void Render(rob::Renderer *renderer);

    private:
        rob::Color m_color;
        float m_fade;
        float m_deltaFade;
        float m_fadeAcceleration;
        float m_maxFade;
    };

} // sneaky

#endif // H_DUCK_FADE_EFFECT_H

