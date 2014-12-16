
#ifndef H_SNEAKY_GUARD_BRAIN_H
#define H_SNEAKY_GUARD_BRAIN_H

#include "Brain.h"
#include "GuardSensors.h"

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
            Suspect,
            Inspect,
            Chase
        };

    public:
        explicit GuardBrain(SneakyState *game, Navigation *nav, rob::Random &rand);
        ~GuardBrain();

        void OnInitialize() override;
        void ReportSound(const vec2f &position, const float volume) override;

    private:
        void HearSound(const vec2f &position, float volume);

        bool LookForPlayer();
        void StartChasingIfPlayerSighted();
        void StartSuspectingIfHeard();

        void ChangeToSuspectState();
        void ChangeToInspectState();
        void ChangeToWatchState();
        void ChangeToPatrolState();
        void ChangeToChaseState();

        void AvoidObstacles();
        void Move(float speed, float dt);
        bool IsStuck() const { return m_stuckMeter > 5.0f; }
        bool IsEndOfPath() const;

        void Inspect(const vec2f &location);

        void UpdateSuspect(const rob::GameTime &gameTime);
        void UpdateInspect(const rob::GameTime &gameTime);
        void UpdateWatch(const rob::GameTime &gameTime);
        void UpdatePatrol(const rob::GameTime &gameTime);
        void UpdateChase(const rob::GameTime &gameTime);

    public:
        void Update(const rob::GameTime &gameTime) override;
        void DebugRender(rob::Renderer *renderer) const override;

        void Navigate(const vec2f &pos);
        void NavigateRandom();

    private:
        SneakyState *m_game;
        Navigation *m_nav;
        NavPath *m_path;
        size_t m_pathPos;
        rob::Random &m_rand;

        GuardLocalSensor m_rightSensor;
        GuardLocalSensor m_leftSensor;
        GuardVisionSensor m_visionSensor;

        float m_stuckMeter;
        vec2f m_prevPosition;
        State m_state;
        float m_stateTimer;
        float m_watchTimer;
        vec2f m_lastKnownPlayerPos;

        bool m_soundHeard;
        vec2f m_soundSource;
        float m_soundIntrest;
    };

} // sneaky

#endif // H_SNEAKY_GUARD_BRAIN_H
