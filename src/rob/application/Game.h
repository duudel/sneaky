
#ifndef H_ROB_GAME_H
#define H_ROB_GAME_H

#include "../memory/LinearAllocator.h"
#include "../input/Keyboard.h"
#include "../input/Mouse.h"

namespace rob
{

    class Window;
    class Graphics;
    class AudioSystem;
    class MasterCache;
    class Renderer;
    class GameState;

    class Game
    {
    public:
        Game();
        Game(const Game&) = delete;
        Game& operator = (const Game&) = delete;
        ~Game();

        virtual bool Initialize() { return true; }

        void Run();

        virtual void OnTextInput(const char *str);
        virtual void OnResize(int w, int h);
        virtual void OnKeyPress(Keyboard::Key key, Keyboard::Scancode scancode, uint32_t mods);
        virtual void OnKeyDown(Keyboard::Key key, Keyboard::Scancode scancode, uint32_t mods);
        virtual void OnKeyUp(Keyboard::Key key, Keyboard::Scancode scancode, uint32_t mods);
        virtual void OnMouseDown(MouseButton button, int x, int y);
        virtual void OnMouseUp(MouseButton button, int x, int y);
        virtual void OnMouseMove(int x, int y);

    protected:
        virtual void HandleStateChange(int state) { }

        template <class State, class... Args>
        void ChangeState(Args&& ...args)
        {
            m_stateAlloc.del_object(m_state);
            m_stateAlloc.Reset();
            m_state = m_stateAlloc.new_object<State>(std::forward<Args&&>(args)...);
            InitState();
        }

    private:
        bool Setup();
        void InitState();

    protected:
        LinearAllocator m_staticAlloc;
        Window *m_window;
        Graphics *m_graphics;
        AudioSystem *m_audio;
        MasterCache *m_cache;
        Renderer *m_renderer;

        GameState *m_state;
        LinearAllocator m_stateAlloc;
    };

} // rob

#endif // H_ROB_GAME_H

