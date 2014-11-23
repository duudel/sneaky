
#ifndef H_SNEAKY_INPUT_H
#define H_SNEAKY_INPUT_H

#include "rob/input/Keyboard.h"
#include "rob/input/Mouse.h"

namespace rob
{
    class View;
} // rob

namespace sneaky
{

    using namespace rob;

    class Input
    {
    public:
        Input()
            : m_enabled(true)
        {
            memset(m_keys, 0, sizeof(m_keys));
        }

        void SetView(const View *view)
        { m_view = view; }

        void SetEnabled(bool enabled)
        { m_enabled = enabled; if (!enabled) ResetMouseState(m_mouse); }

        void SetKey(Keyboard::Scancode key, bool down)
        { m_keys[static_cast<size_t>(key)] = down; }

        bool KeyDown(Keyboard::Scancode key) const
        { return m_keys[static_cast<size_t>(key)] && m_enabled; }

        void UpdateMouse()
        { if (m_enabled) GetMouseState(m_mouse); }

        bool ButtonDown(MouseButton button) const
        { return m_mouse.ButtonDown(button); }

        vec2f GetMousePosition() const
        { return m_mouse.GetPosition(); }

        vec2f GetMouseWorldPosition() const;

        vec2f GetMouseDelta() const
        { return m_mouse.GetDeltaPosition(); }

    private:
        bool m_keys[static_cast<size_t>(Keyboard::Scancode::NUM_KEYS)];
        Mouse m_mouse;
        bool m_enabled;

        const View *m_view;
    };

} // sneaky

#endif // H_SNEAKY_INPUT_H

