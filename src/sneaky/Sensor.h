
#ifndef H_SNEAKY_SENSOR_H
#define H_SNEAKY_SENSOR_H

#include "Physics.h"

#define BIT(x) (1 << x)

namespace sneaky
{

    class SneakyState;

    enum
    {
        PlayerBit = BIT(4),
        GuardBit = BIT(5),
        SensorBit = BIT(6),
        StaticBit = BIT(7),
        AllBits = 0xffff
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

        void SetState(SneakyState *state)
        { m_state = state; }

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
            fixDef.filter.categoryBits = SensorBit;
            fixDef.filter.maskBits = m_maskBits;
            m_body->CreateFixture(&fixDef);
        }

        virtual void BeginContact(b2Fixture *fixture, void *userData) { }
        virtual void EndContact(b2Fixture *fixture, void *userData) { }
        virtual void BeginParticleContact(b2ParticleSystem *ps, const b2ParticleBodyContact *particleBodyContact) { }
        virtual void EndParticleContact(b2ParticleSystem *ps, int index) { }

    protected:
        SneakyState *m_state;
        b2Body *m_body;

    private:
        uint16 m_maskBits;
        bool m_particles;
    };

    class SensorListener : public b2ContactListener
    {
    public:
        void BeginContact(b2Contact* contact) override
        {
            Sensor *sensor = nullptr;
            b2Fixture *fixture = GetFixtureAndSensor(&sensor, contact);
            if (sensor)
                sensor->BeginContact(fixture, fixture->GetUserData());
        }

        void EndContact(b2Contact *contact) override
        {
            Sensor *sensor = nullptr;
            b2Fixture *fixture = GetFixtureAndSensor(&sensor, contact);
            if (sensor)
                sensor->EndContact(fixture, fixture->GetUserData());
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
        b2Fixture* GetFixtureAndSensor(Sensor **sensor, b2Contact *contact)
        {
            if (( *sensor = GetSensor(contact->GetFixtureA()) ))
                return contact->GetFixtureB();
            if (( *sensor = GetSensor(contact->GetFixtureB()) ))
                return contact->GetFixtureA();
            return nullptr;
        }

        Sensor *GetSensor(const b2Fixture *fixture)
        {
            if (fixture->GetFilterData().categoryBits == SensorBit)
                return (Sensor*)fixture->GetUserData();
            return nullptr;
        }
    };

} // sneaky

#endif // H_SNEAKY_SENSOR_H
