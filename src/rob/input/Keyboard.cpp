
#include "Keyboard.h"

#include <SDL2/SDL.h>

namespace rob
{

    const char* GetKeyName(Keyboard::Key key)
    { return ::SDL_GetKeyName(static_cast<SDL_Keycode>(key)); }

    const char* GetScancodeName(Keyboard::Scancode key)
    { return ::SDL_GetScancodeName(static_cast<SDL_Scancode>(key)); }

} // rob
