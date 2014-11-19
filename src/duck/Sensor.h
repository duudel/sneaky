
#ifndef H_SNEAKY_SENSOR_H
#define H_SNEAKY_SENSOR_H

#include "Physics.h"

#define BIT(x) (1 << x)

namespace sneaky
{

    class SneakyState;

    enum
    {
        BirdBits = BIT(5),
        SensorBits = BIT(6),
        StaticBits = BIT(7),
        WheelBits = BIT(8)
    };

    class Sensor
    {
    public:
        Sensor(uint16 maskBits)
            : m_maskBits(maskBits)
            , m_particles(false)
        { }
        Sensor(uint16 maskBits, bool particles)
            : m_maskBits(maskBits)
            , m_particles(particles)
        { }

        bool WithParticles() const { return m_particles; }

        virtual ~Sensor()
        { }

        void SetDuckState(SneakyState *duckState)
        { m_duckState = duckState; }

        void SetBody(b2Body *body)
        { m_body = body; }
        b2Body* GetBody() const
        { return m_body; }

        void SetShape(const b2Shape *shape)
        {
            b2FixtureDef fixDef;
            fixDef.shape = shape;
            fixDef.userData = this;
            fixDef.isSensor = true;
            fixDef.filter.categoryBits = SensorBits;
            fixDef.filter.maskBits = m_maskBits;
            m_body->CreateFixture(&fixDef);
        }

        virtual void BeginContact(void *userData) { }
        virtual void EndContact(void *userData) { }
        virtual void BeginParticleContact(b2ParticleSystem *ps, const b2ParticleBodyContact *particleBodyContact) { }
        virtual void EndParticleContact(b2ParticleSystem *ps, int index) { }

    protected:
        SneakyState *m_duckState;

    private:
        uint16 m_maskBits;
        bool m_particles;
        b2Body *m_body;
    };

    class SensorListener : public b2ContactListener
    {
    public:
        void BeginContact(b2Contact* contact) override
        {
            Sensor *sensor = nullptr;
            void *userData = GetUserDataAndSensor(&sensor, contact);
            if (sensor)
                sensor->BeginContact(userData);
        }

        void EndContact(b2Contact *contact) override
        {
            Sensor *sensor = nullptr;
            void *userData = GetUserDataAndSensor(&sensor, contact);
            if (sensor)
                sensor->EndContact(userData);
        }

        void BeginContact(b2ParticleSystem *ps, b2ParticleBodyContact *particleBodyContact) override
        {
            Sensor *sensor = GetSensor(particleBodyContact->fixture);
            if (sensor && sensor->WithParticles()) sensor->BeginParticleContact(ps, particleBodyContact);
        }

        void EndContact(b2Fixture *fixture, b2ParticleSystem *ps, int32 index) override
        {
            Sensor *sensor = GetSensor(fixture);
            if (sensor && sensor->WithParticles()) sensor->EndParticleContact(ps, index);
        }

    private:
        void* GetUserDataAndSensor(Sensor **sensor, b2Contact *contact)
        {
            if (( *sensor = GetSensor(contact->GetFixtureA()) ))
                return contact->GetFixtureB()->GetUserData();
            if (( *sensor = GetSensor(contact->GetFixtureB()) ))
                return contact->GetFixtureA()->GetUserData();
            return nullptr;
        }

        Sensor *GetSensor(const b2Fixture *fixture)
        {
            if (fixture->GetFilterData().categoryBits == SensorBits)
                return (Sensor*)fixture->GetUserData();
            return nullptr;
        }
    };

} // sneaky

#endif // H_SNEAKY_SENSOR_H
