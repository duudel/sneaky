
#ifndef H_ROB_MOUSE_H
#define H_ROB_MOUSE_H

#include "../math/Vector2.h"
#include "../math/Types.h"

#include <SDL2/SDL_mouse.h>

namespace rob
{

    enum class MouseButton
    {
        Left = SDL_BUTTON_LEFT,
        Middle = SDL_BUTTON_MIDDLE,
        Right = SDL_BUTTON_RIGHT,
        X1 = SDL_BUTTON_X1,
        X2 = SDL_BUTTON_X2,

        NUM_BUTTONS
    };

    class Mouse;

    void GetMouseState(Mouse &mouse);

    class Mouse
    {
    public:
        bool ButtonDown(MouseButton button) const;
        vec2f GetPosition() const;
        vec2f GetDeltaPosition() const;

    private:
        friend void GetMouseState(Mouse &mouse);

        void SetButtons(const uint32_t buttons);
        void SetPosition(int x, int y, int dx, int dy);

    private:
        bool m_buttons[static_cast<size_t>(MouseButton::NUM_BUTTONS)];
        vec2f m_position;
        vec2f m_delta;
    };


} // rob

#endif // H_ROB_MOUSE_H

