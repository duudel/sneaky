
#ifndef H_SNEAKY_GUARD_SENSORS_H
#define H_SNEAKY_GUARD_SENSORS_H

#include "Sensor.h"

namespace sneaky
{

    class GuardLocalSensor : public Sensor
    {
    public:
        GuardLocalSensor()
            : Sensor(GuardBit | StaticBit | CakeBit, false)
            , m_hit(0)
            , m_guards(0)
        { }

        void BeginContact(b2Fixture *fixture, void *userData) override
        {
            if (m_body != fixture->GetBody()) m_hit++;
            if (fixture->GetBody()->GetType() != b2_staticBody)
                m_guards++;
        }

        void EndContact(b2Fixture *fixture, void *userData) override
        {
            if (m_body != fixture->GetBody()) m_hit--;
            if (fixture->GetBody()->GetType() != b2_staticBody)
                m_guards--;
        }

        bool HitsObstacle() const
        { return m_hit > 0; }

        bool HitsGuard() const
        { return m_guards > 0; }

        bool HitsWall() const
        { return (m_hit - m_guards) > 0; }

    private:
        size_t m_hit;
        size_t m_guards;
    };

    class GuardVisionSensor : public Sensor
    {
    public:
        GuardVisionSensor()
            : Sensor(PlayerBit, false)
            , m_body(nullptr)
        { }

        void BeginContact(b2Fixture *fixture, void *userData) override
        { m_body = fixture->GetBody(); }

        void EndContact(b2Fixture *fixture, void *userData) override
        { m_body = nullptr; }

        bool PlayerSighted() const
        { return m_body; }

        b2Body *GetBody() const
        { return m_body; }

    private:
        b2Body *m_body;
    };

} // sneaky

#endif // H_SNEAKY_GUARD_SENSORS_H

