
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
        void SetView(const View *view)
        { m_view = view; }

        void SetKey(Keyboard::Scancode key, bool down)
        { m_keys[static_cast<size_t>(key)] = down; }

        bool KeyDown(Keyboard::Scancode key) const
        { return m_keys[static_cast<size_t>(key)]; }

        void UpdateMouse()
        { GetMouseState(m_mouse); }

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

        const View *m_view;
    };

} // sneaky

#endif // H_SNEAKY_INPUT_H

