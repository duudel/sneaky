
#ifndef H_SNEAKY_BRAIN_H
#define H_SNEAKY_BRAIN_H

#include "Physics.h"

namespace rob
{
    class GameTime;
    class Renderer;
} // rob

namespace sneaky
{

    class GameObject;

    class Brain
    {
    public:
        Brain()
            : m_owner(nullptr)
        { }

        virtual ~Brain() { }

        virtual void Update(const rob::GameTime &gameTime) = 0;
        virtual void DebugRender(rob::Renderer *renderer) const { }

        void SetOwner(GameObject *owner) { m_owner = owner; }
        virtual void OnInitialize() { }

    protected:
        GameObject *m_owner;
    };


    class Input;

    class PlayerBrain : public Brain
    {
    public:
        explicit PlayerBrain(Input *input)
            : Brain()
            , m_input(input)
            , m_target(0.0f, 0.0f)
        { }
        void Update(const rob::GameTime &gameTime) override;
        void DebugRender(rob::Renderer *renderer) const override;
    private:
        Input *m_input;
        vec2f m_target;
    };

} // sneaky

#endif // H_SNEAKY_BRAIN_H

