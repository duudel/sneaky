
#include "MicroTicker.h"

#include <SDL2/SDL.h>

namespace rob
{

    void MicroTicker::Init()
    {
        m_frequency = SDL_GetPerformanceFrequency();
        m_startTicks = SDL_GetPerformanceCounter();
        m_errorTicks = 0;
    }

    Time_t MicroTicker::GetTicks()
    {
        Time_t deltaTicks = SDL_GetPerformanceCounter() - m_startTicks;

        Time_t seconds = deltaTicks / m_frequency;
        deltaTicks -= seconds * m_frequency;

        m_errorTicks += deltaTicks*1000000ULL;
        Time_t micros = m_errorTicks / m_frequency;
        m_errorTicks -= micros * m_frequency;

        return seconds*1000000ULL + micros;
    }

} // rob
