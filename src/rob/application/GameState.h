
#ifndef H_ROB_GAME_STATE_H
#define H_ROB_GAME_STATE_H

#include "GameTime.h"
#include "../time/MicroTicker.h"
#include "../time/VirtualTime.h"

#include "../renderer/Renderer.h"

#include "../input/Keyboard.h"
#include "../input/Mouse.h"
#include "../Log.h"
#include "../String.h"

namespace rob
{

    class GameTime;
    class LinearAllocator;
    class AudioSystem;
    class MasterCache;
//    class Renderer;
    class Window;

    class GameState
    {
    public:
        GameState();
        GameState(const GameState&) = delete;
        GameState& operator = (const GameState&) = delete;
        virtual ~GameState() { }

        void SetAllocator(LinearAllocator &alloc) { m_alloc = &alloc; }
        LinearAllocator& GetAllocator() { return *m_alloc; }

        void SetAudio(AudioSystem *audio) { m_audio = audio; }
        AudioSystem& GetAudio() { return *m_audio; }

        void SetCache(MasterCache *cache) { m_cache = cache; }
        MasterCache& GetCache() { return *m_cache; }

        void SetRenderer(Renderer *renderer) { m_renderer = renderer; }
        Renderer& GetRenderer() { return *m_renderer; }

        void SetWindow(Window *window) { m_window = window; }
        Window& GetWindow() { return *m_window; }

        const View& GetDefaultView() const { return m_defaultView; }

        /// Gets called from Game. Handles fixed step update and calls virtual method Update.
        void DoUpdate();
        /// Gets called from Game. Calls virtual method Render.
        void DoRender();

        void Resize(int w, int h);


        virtual bool Initialize() { return true; }

        virtual void RealtimeUpdate(const Time_t deltaMicroseconds) { }
        virtual void Update(const GameTime &gameTime) { }
        virtual void Render() { }

        virtual void OnResize(int w, int h) { }


        virtual void OnTextInput(const char *str) { }

        virtual void OnKeyPress(Keyboard::Key key, Keyboard::Scancode scancode, uint32_t mods) { }
        virtual void OnKeyDown(Keyboard::Key key, Keyboard::Scancode scancode, uint32_t mods) { }
        virtual void OnKeyUp(Keyboard::Key key, Keyboard::Scancode scancode, uint32_t mods) { }

        virtual void OnMouseDown(MouseButton button, int x, int y) { }
        virtual void OnMouseUp(MouseButton button, int x, int y) { }
        virtual void OnMouseMove(int x, int y) { }

        void QuitState() { m_quit = true; }
        bool IsQuiting() const { return m_quit; }

        void ChangeState(int state) { m_nextState = state; }
        int NextState() const { return m_nextState; }

    private:
        MicroTicker m_ticker;
    protected:
        VirtualTime m_time;
        GameTime m_gameTime;

    private:
        LinearAllocator *   m_alloc;
        AudioSystem *       m_audio;
        MasterCache *       m_cache;
        Renderer *          m_renderer;
        Window *            m_window;
        bool m_quit;
        int m_nextState;

        View m_defaultView;

        int m_fps;
        int m_frames;
        Time_t m_lastTime;
        Time_t m_accumulator;
    };

} // rob

#endif // H_ROB_GAME_STATE_H

