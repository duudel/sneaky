
#include "VirtualTime.h"

namespace rob
{

    VirtualTime::VirtualTime(MicroTicker &ticker)
        : m_ticker(ticker)
        , m_time(0)
        , m_last(0)
        , m_paused(true)
    { }

    void VirtualTime::Restart()
    {
        m_time = 0;
        Resume();
    }

    void VirtualTime::Pause()
    {
        m_paused = true;
    }

    void VirtualTime::Resume()
    {
        m_last = m_ticker.GetTicks();
        m_paused = false;
    }

    void VirtualTime::Update()
    {
        if (!m_paused)
        {
            Time_t t = m_ticker.GetTicks();
            m_time += t - m_last;
            m_last = t;
        }
    }

    double VirtualTime::GetTime() const
    {
        return m_time / 1000000.0;
    }

    Time_t VirtualTime::GetTimeMicros() const
    {
        return m_time;
    }

    bool VirtualTime::IsPaused() const
    {
        return m_paused;
    }

} // rob
