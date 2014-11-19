
#include "Game.h"
#include "GameState.h"
#include "Window.h"
#include "../graphics/Graphics.h"
#include "../audio/AudioSystem.h"
#include "../resource/MasterCache.h"
#include "../renderer/Renderer.h"

#include "../Log.h"

#include <SDL2/SDL.h>

namespace rob
{

    static const size_t STATIC_MEMORY_SIZE = 4 * 1024 * 1024;

    Game::Game()
        : m_staticAlloc(STATIC_MEMORY_SIZE)
        , m_window(nullptr)
        , m_graphics(nullptr)
        , m_audio(nullptr)
        , m_cache(nullptr)
        , m_renderer(nullptr)
        , m_state(nullptr)
        , m_stateAlloc()
    {
        ::SDL_Init(SDL_INIT_EVERYTHING);

        m_window = m_staticAlloc.new_object<Window>();
        m_graphics = m_staticAlloc.new_object<Graphics>(m_staticAlloc);
        m_audio = m_staticAlloc.new_object<AudioSystem>(m_staticAlloc);
        m_cache = m_staticAlloc.new_object<MasterCache>(m_graphics, m_audio, m_staticAlloc);
        m_renderer = m_staticAlloc.new_object<Renderer>(m_graphics, m_cache, m_staticAlloc);

        log::Info("GL debug output: ", (m_graphics->HasDebugOutput()?"yes":"no"));

        const size_t freeMemory = STATIC_MEMORY_SIZE - m_staticAlloc.GetAllocatedSize();
        log::Info("Static memory used: ", m_staticAlloc.GetAllocatedSize(), " B from ",
                  STATIC_MEMORY_SIZE, " B total", " (", freeMemory ," B free)");
        m_stateAlloc.SetMemory(m_staticAlloc.Allocate(freeMemory), freeMemory);
    }

    Game::~Game()
    {
        m_stateAlloc.del_object(m_state);
        m_staticAlloc.del_object(m_renderer);
        m_staticAlloc.del_object(m_cache);
        m_staticAlloc.del_object(m_audio);
        m_staticAlloc.del_object(m_graphics);
        m_staticAlloc.del_object(m_window);
        ::SDL_Quit();
    }

    bool Game::Setup()
    {
        if (!m_graphics->IsInitialized())
        {
            log::Error("Graphics was not initialized, aborting...");
            return false;
        }

        if (!Initialize())
        {
            log::Error("Could not initialize game, aborting...");
            return false;
        }

        if (m_state == nullptr)
        {
            log::Error("Game state not initialized by Game::Initialize(), aborting...");
            return false;
        }

        return true;
    }

    void Game::InitState()
    {
        m_state->SetAllocator(m_stateAlloc);
        m_state->SetAudio(m_audio);
        m_state->SetCache(m_cache);
        m_state->SetRenderer(m_renderer);
        m_state->SetWindow(m_window);
        m_state->Initialize();

        int w, h;
        m_window->GetSize(&w, &h);
        OnResize(w, h);
    }

    void Game::Run()
    {
        if (!Setup())
            return;

        while (m_window->HandleEvents(this))
        {
            m_graphics->Clear();

            m_audio->Update();

            m_state->DoUpdate();
            m_state->DoRender();

            m_window->SwapBuffers();

            if (m_state->IsQuiting())
                break;

            HandleStateChange(m_state->NextState());
        }
    }

    void Game::OnTextInput(const char *str)
    { m_state->OnTextInput(str); }

    void Game::OnResize(int w, int h)
    { m_state->Resize(w, h); }

    void Memory(size_t totalBytes, size_t &MB, size_t &kB, size_t &B)
    {
        B = totalBytes;
        kB = B / 1024;
        B -= kB * 1024;
        MB = kB / 1024;
        kB -= MB * 1024;
    }

    void ReportMemoryUsage(const size_t used, const size_t total)
    {
        const size_t free = total - used;
        log::Info("State memory usage:");
        size_t MB, kB, B;
        Memory(used, MB, kB, B);
        log::Info("  used:  ", MB, " MB ", kB, " kB ", B, " B");
        Memory(free, MB, kB, B);
        log::Info("  free:  ", MB, " MB ", kB, " kB ", B, " B");
        Memory(total, MB, kB, B);
        log::Info("  total: ", MB, " MB ", kB, " kB ", B, " B");
    }

    void Game::OnKeyPress(Keyboard::Key key, Keyboard::Scancode scancode, uint32_t mods)
    {
        if (key == Keyboard::Key::F12)
            ReportMemoryUsage(m_stateAlloc.GetAllocatedSize(), m_stateAlloc.GetTotalSize());
        m_state->OnKeyPress(key, scancode, mods);
    }

    void Game::OnKeyDown(Keyboard::Key key, Keyboard::Scancode scancode, uint32_t mods)
    { m_state->OnKeyDown(key, scancode, mods); }

    void Game::OnKeyUp(Keyboard::Key key, Keyboard::Scancode scancode, uint32_t mods)
    { m_state->OnKeyUp(key, scancode, mods); }

    void Game::OnMouseDown(MouseButton button, int x, int y)
    { m_state->OnMouseDown(button, x, y); }

    void Game::OnMouseUp(MouseButton button, int x, int y)
    { m_state->OnMouseUp(button, x, y); }

    void Game::OnMouseMove(int x, int y)
    { m_state->OnMouseMove(x, y); }

} // rob
