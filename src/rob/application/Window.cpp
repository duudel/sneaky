
#include "Window.h"
#include "Game.h"

#include "../Log.h"
#include "../Assert.h"

#include <SDL2/SDL.h>
#include <GL/glew.h>

namespace rob
{

    struct WindowData
    {
        SDL_Window *    m_window;
        SDL_GLContext   m_context;
    };

    Window::Window()
        : m_windowDataBuffer()
        , m_data(nullptr)
    {
        m_data = reinterpret_cast<WindowData*>(m_windowDataBuffer.m_value);
        m_data->m_window = nullptr;
        m_data->m_context = nullptr;

        SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
        SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
        SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
        SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 2);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
    #ifdef ROB_DEBUG
//        SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS,
            SDL_GL_CONTEXT_DEBUG_FLAG | SDL_GL_CONTEXT_PROFILE_CORE);
    #endif // ROB_DEBUG


        m_data->m_window = SDL_CreateWindow("Rob engine",
                                            SDL_WINDOWPOS_UNDEFINED,
                                            SDL_WINDOWPOS_UNDEFINED,
                                            1280, 720,
                                            SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
//                                            0, 0,
//                                            SDL_WINDOW_OPENGL | SDL_WINDOW_FULLSCREEN_DESKTOP);
        if (!m_data->m_window)
        {
            log::Error("Could not create a window");
            return;
        }

        m_data->m_context = ::SDL_GL_CreateContext(m_data->m_window);
        if (!m_data->m_context)
        {
            log::Error("Could not create GL context");
            return;
        }

        if (::glewInit() != GLEW_OK)
        {
            log::Error("Could not initialize GLEW");
            return;
        }

        ROB_ASSERT(GLEW_VERSION_2_1);

        SDL_GL_SetSwapInterval(0);
    }

    Window::~Window()
    {
        if (m_data->m_context)
        {
            ::SDL_GL_DeleteContext(m_data->m_context);
        }
        if (m_data->m_window)
        {
            ::SDL_DestroyWindow(m_data->m_window);
        }
    }

    bool Window::HandleEvents(Game *game)
    {
        ::SDL_PumpEvents();
        SDL_Event event;
        while (::SDL_PollEvent(&event) == 1)
        {
            switch (event.type)
            {
            case SDL_QUIT:
                return false;
            case SDL_TEXTINPUT:
                game->OnTextInput(event.text.text);
                break;
            case SDL_KEYDOWN:
                {
                    const Keyboard::Key key = static_cast<Keyboard::Key>(event.key.keysym.sym);
                    const Keyboard::Scancode scancode = static_cast<Keyboard::Scancode>(event.key.keysym.scancode);
                    if (event.key.repeat == 0)
                        game->OnKeyPress(key, scancode, ::SDL_GetModState());
                    game->OnKeyDown(key, scancode, ::SDL_GetModState());
                }
                break;
            case SDL_KEYUP:
                {
                    const Keyboard::Key key = static_cast<Keyboard::Key>(event.key.keysym.sym);
                    const Keyboard::Scancode scancode = static_cast<Keyboard::Scancode>(event.key.keysym.scancode);
                    game->OnKeyUp(key, scancode, ::SDL_GetModState());
                }
                break;
            case SDL_MOUSEBUTTONDOWN:
                game->OnMouseDown(static_cast<MouseButton>(event.button.button), event.button.x, event.button.y);
                break;
            case SDL_MOUSEBUTTONUP:
                game->OnMouseUp(static_cast<MouseButton>(event.button.button), event.button.x, event.button.y);
                break;
            case SDL_MOUSEMOTION:
                game->OnMouseMove(event.motion.x, event.motion.y);
                break;
            case SDL_WINDOWEVENT:
                switch (event.window.event)
                {
                case SDL_WINDOWEVENT_RESIZED:
                    game->OnResize(event.window.data1, event.window.data2);
                    break;
                }
            }
        }
        return true;
    }

    void Window::SwapBuffers()
    { ::SDL_GL_SwapWindow(m_data->m_window); }

    void Window::SetTitle(const char * const title)
    { ::SDL_SetWindowTitle(m_data->m_window, title); }

    void Window::GetSize(int *w, int *h) const
    { ::SDL_GetWindowSize(m_data->m_window, w, h); }

    void Window::GrabMouse()
    {
        ::SDL_SetWindowGrab(m_data->m_window, SDL_TRUE);
        ::SDL_SetRelativeMouseMode(SDL_TRUE);
    }

    void Window::UnGrabMouse()
    {
        ::SDL_SetWindowGrab(m_data->m_window, SDL_FALSE);
        ::SDL_SetRelativeMouseMode(SDL_FALSE);
    }

    void Window::ToggleGrabMouse()
    {
        if (IsMouseGrabbed())
            UnGrabMouse();
        else
            GrabMouse();
    }

    bool Window::IsMouseGrabbed() const
    { return ::SDL_GetWindowGrab(m_data->m_window) == SDL_TRUE; }

} // rob
