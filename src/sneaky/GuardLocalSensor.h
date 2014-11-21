
#ifndef H_SNEAKY_GUARD_LOCAL_SENSOR_H
#define H_SNEAKY_GUARD_LOCAL_SENSOR_H

#include "Sensor.h"

namespace sneaky
{

    class GuardLocalSensor : public Sensor
    {
    public:
        GuardLocalSensor()
            : Sensor(GuardBit | StaticBit, false)
            , m_hit(0)
        { }

        void BeginContact(b2Fixture *fixture, void *userData) override
        { if (m_body != fixture->GetBody()) m_hit++; }

        void EndContact(b2Fixture *fixture, void *userData) override
        { if (m_body != fixture->GetBody()) m_hit--; }

        bool HitsObstacle() const
        { return m_hit > 0; }

    private:
        size_t m_hit;
    };

} // sneaky

#endif // H_SNEAKY_GUARD_LOCAL_SENSOR_H

