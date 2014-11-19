
#include "GameTime.h"

namespace rob
{

    GameTime::GameTime()
        : m_deltaTime(16666) // = 16.666ms
        , m_time(0)
        , m_accumulator(0)
    { }

    void GameTime::Update(const Time_t frameTime)
    {
        m_accumulator += frameTime;
    }

    bool GameTime::Step()
    {
        if (m_accumulator > m_deltaTime)
        {
            m_accumulator -= m_deltaTime;
            m_time += m_deltaTime;
            return true;
        }
        return false;
    }

    Time_t GameTime::GetDeltaMicroseconds() const
    { return m_deltaTime; }

    double GameTime::GetDeltaSeconds() const
    { return double(m_deltaTime) / 1000000.0; }

    Time_t GameTime::GetTotalMicroseconds() const
    { return m_time; }

    double GameTime::GetTotalSeconds() const
    { return double(m_time) / 1000000.0; }

} // rob
