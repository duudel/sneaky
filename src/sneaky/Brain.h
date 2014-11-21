
#ifndef H_SNEAKY_BRAIN_H
#define H_SNEAKY_BRAIN_H

#include "Physics.h"
#include "rob/math/Random.h"

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
        { }
        void Update(const rob::GameTime &gameTime) override;
    private:
        Input *m_input;
    };


    class Navigation;
    class NavPath;

    class GuardBrain : public Brain
    {
    public:
        explicit GuardBrain(Navigation *nav);
        ~GuardBrain();

        void Update(const rob::GameTime &gameTime) override;
        void DebugRender(rob::Renderer *renderer) const override;

        void Navigate(const vec2f &pos);

    private:
        Navigation *m_nav;
        NavPath *m_path;
        size_t m_pathPos;
        rob::Random m_rand;
    };

} // sneaky

#endif // H_SNEAKY_BRAIN_H

