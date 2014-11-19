
#ifndef H_ROB_VIRTUAL_TIME_H
#define H_ROB_VIRTUAL_TIME_H

#include "MicroTicker.h"

namespace rob
{

    class VirtualTime
    {
    public:
        VirtualTime(MicroTicker &ticker);

        void Restart();
        void Pause();
        void Resume();
        void Update();

        double GetTime() const;
        Time_t GetTimeMicros() const;

        bool IsPaused() const;

    private:
        MicroTicker &m_ticker;
        Time_t m_time;
        Time_t m_last;
        bool m_paused;
    };

} // rob

#endif // H_ROB_VIRTUAL_TIME_H
