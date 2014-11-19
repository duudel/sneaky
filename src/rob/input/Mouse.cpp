
#include "Mouse.h"

namespace rob
{

    bool Mouse::ButtonDown(MouseButton button) const
    { return m_buttons[static_cast<int>(button)]; }

    vec2f Mouse::GetPosition() const
    { return m_position; }

    vec2f Mouse::GetDeltaPosition() const
    { return m_delta; }

    void Mouse::SetButtons(const uint32_t buttons)
    {
        m_buttons[static_cast<int>(MouseButton::Left)] = buttons & SDL_BUTTON_LMASK;
        m_buttons[static_cast<int>(MouseButton::Middle)] = buttons & SDL_BUTTON_MMASK;
        m_buttons[static_cast<int>(MouseButton::Right)] = buttons & SDL_BUTTON_RMASK;
        m_buttons[static_cast<int>(MouseButton::X1)] = buttons & SDL_BUTTON_X1MASK;
        m_buttons[static_cast<int>(MouseButton::X2)] = buttons & SDL_BUTTON_X2MASK;
    }

    void Mouse::SetPosition(int x, int y, int dx, int dy)
    {
        m_position.x = x;
        m_position.y = y;
        m_delta.x = dx;
        m_delta.y = dy;
    }

    void GetMouseState(Mouse &mouse)
    {
        int mx, my, dx, dy;
        const uint32_t buttons = ::SDL_GetMouseState(&mx, &my);
        ::SDL_GetRelativeMouseState(&dx, &dy);

        mouse.SetPosition(mx, my, dx, dy);
        mouse.SetButtons(buttons);
    }

} // rob
