
#ifndef H_ROB_GAME_TIME_H
#define H_ROB_GAME_TIME_H

#include "../Types.h"

namespace rob
{

    class GameTime
    {
    public:
        GameTime();
        void Update(const Time_t frameTime);
        bool Step();
        Time_t GetDeltaMicroseconds() const;
        double GetDeltaSeconds() const;
        Time_t GetTotalMicroseconds() const;
        double GetTotalSeconds() const;
    private:
        Time_t m_deltaTime;
        Time_t m_time;
        Time_t m_accumulator;
    };

} // rob

#endif // H_ROB_GAME_TIME_H
