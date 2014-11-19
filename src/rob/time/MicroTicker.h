
#ifndef H_ROB_MICRO_TICKER_H
#define H_ROB_MICRO_TICKER_H

#include "../Types.h"

namespace rob
{

    class MicroTicker
    {
    public:
        void Init();

        Time_t GetTicks();

    private:
        Time_t m_frequency;
        Time_t m_startTicks;
        Time_t m_errorTicks;
    };

} // rob

#endif // H_ROB_MICRO_TICKER_H
