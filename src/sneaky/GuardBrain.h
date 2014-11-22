
#ifndef H_SNEAKY_GUARD_BRAIN_H
#define H_SNEAKY_GUARD_BRAIN_H

#include "Brain.h"
#include "GuardLocalSensor.h"

#include "rob/math/Random.h"

namespace sneaky
{

    class Navigation;
    class NavPath;

    class GuardBrain : public Brain
    {
        enum class State
        {
            Watch,
            Patrol,
            Chase
        };

    public:
        explicit GuardBrain(Navigation *nav, rob::Random &rand);
        ~GuardBrain();

        void OnInitialize() override;

    private:
        State UpdateWatch(const rob::GameTime &gameTime);
        State UpdatePatrol(const rob::GameTime &gameTime);
        State UpdateChase(const rob::GameTime &gameTime);

    public:
        void Update(const rob::GameTime &gameTime) override;
        void DebugRender(rob::Renderer *renderer) const override;

        void Navigate(const vec2f &pos);
        void NavigateRandom();

    private:
        Navigation *m_nav;
        NavPath *m_path;
        size_t m_pathPos;
        rob::Random &m_rand;

        GuardLocalSensor m_rightSensor;
        GuardLocalSensor m_leftSensor;

        float m_stuckMeter;
        vec2f m_prevPosition;
        State m_state;
        float m_stateTimer;
        float m_watchTimer;
    };

} // sneaky

#endif // H_SNEAKY_GUARD_BRAIN_H
