
#include "Time.h"

#include <SDL2/SDL.h>
#include <ctime>

namespace rob
{

    uint32_t GetTicks()
    { return SDL_GetTicks(); }

    uint32_t GetTime()
    { return std::time(0); }

    void Delay(uint32_t milliseconds)
    { ::SDL_Delay(milliseconds); }

} // rob
