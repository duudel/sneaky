
#ifndef H_ROB_WINDOW_H
#define H_ROB_WINDOW_H

#include "../memory/AlignedStorage.h"

namespace rob
{

    class Game;
    struct WindowData;

    class Window
    {
    public:
        Window();
        Window(const Window&) = delete;
        Window& operator = (const Window&) = delete;
        ~Window();

        bool HandleEvents(Game *game);

        void SwapBuffers();

        void SetTitle(const char * const title);
        void GetSize(int *w, int *h) const;

        void GrabMouse();
        void UnGrabMouse();
        void ToggleGrabMouse();
        bool IsMouseGrabbed() const;

    private:
        // Reserve enough space for SDL window data
        static const unsigned SIZE_OF_WIN_DATA_BUFFER = 6 * sizeof(void*);
        AlignedStorage<SIZE_OF_WIN_DATA_BUFFER, sizeof(void*)> m_windowDataBuffer;
//        char m_windowDataBuffer[SIZE_OF_WIN_DATA_BUFFER];

        WindowData *m_data;
    };

} // rob

#endif // H_ROB_WINDOW_H

