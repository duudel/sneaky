
#include "Time.h"

#include <SDL2/SDL.h>

namespace rob
{

    uint32_t GetTicks()
    { return SDL_GetTicks(); }

    void Delay(uint32_t milliseconds)
    { ::SDL_Delay(milliseconds); }

} // rob
