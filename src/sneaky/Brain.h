
#ifndef H_SNEAKY_BRAIN_H
#define H_SNEAKY_BRAIN_H

#include "Physics.h"
#include "Sensor.h"

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


    class CakeSensor : public Sensor
    {
    public:
        CakeSensor()
            : Sensor(CakeBit, false)
            , m_hit(0)
        { }

        void BeginContact(b2Fixture *fixture, void *userData) override
        { m_hit++; }

        void EndContact(b2Fixture *fixture, void *userData) override
        { m_hit--; }

        bool HitsCake() const
        { return (m_hit > 0); }

    private:
        size_t m_hit;
    };


    class Input;

    class PlayerBrain : public Brain
    {
    public:
        explicit PlayerBrain(SneakyState *game, Input *input)
            : Brain()
            , m_game(game)
            , m_input(input)
            , m_target(0.0f, 0.0f)
        { }

        void OnInitialize() override;

        void Update(const rob::GameTime &gameTime) override;
        void DebugRender(rob::Renderer *renderer) const override;
    private:
        SneakyState *m_game;
        Input *m_input;
        vec2f m_target;
        CakeSensor m_cakeSensor;
    };

} // sneaky

#endif // H_SNEAKY_BRAIN_H

