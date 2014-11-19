
#ifndef H_ROB_TEXT_INPUT_H
#define H_ROB_TEXT_INPUT_H

#include "../Types.h"
#include "Keyboard.h"

namespace rob
{

    const char* GetClipboardText();
    void FreeClipboardText(const char *text);
    void SetClipboardText(const char *text);

    class TextInput
    {
        // TODO: Enable dynamic text length, with optional length limit.
        // This would need an allocator given as a constructor parameter or
        // the memory would be paseed to the TextInput as constructor parameters.
        static const size_t MAX_LENGTH = 64;
        char m_text[MAX_LENGTH + 1];
        size_t m_length;
        size_t m_cursor;
        size_t m_lengthLimit;
    public:
        TextInput();

        const char* GetText() const;
        size_t GetLength() const;
        size_t GetCursor() const;

        void SetLengthLimit(size_t limit);
        size_t LengthLimit() const;

        bool MoveLeft();
        bool MoveRight();

        void MoveWordLeft();
        void MoveWordRight();

        void MoveHome();
        void MoveEnd();

        void Insert(const char *str);

        void Delete();
        void DeleteLeft();

        void DeleteWord();
        void DeleteWordLeft();
    };

    class TextInputHandler
    {
        TextInput &m_input;
    public:
        TextInputHandler(TextInput &input)
            : m_input(input)
        { }

        void OnTextInput(const char *str)
        { m_input.Insert(str); }

        void OnKeyDown(Keyboard::Key key, uint32_t mods)
        {
            bool numlock = (mods & KeyMod::Num);
            if (mods & KeyMod::Ctrl)
            {
                switch (key)
                {
                case Keyboard::Key::C:
                    SetClipboardText(m_input.GetText()); break;
                case Keyboard::Key::V:
                {
                    const char *t = GetClipboardText();
                    m_input.Insert(t);
                    FreeClipboardText(t);
                    break;
                }
                case Keyboard::Key::Delete:     m_input.DeleteWord(); break;
                case Keyboard::Key::Backspace:  m_input.DeleteWordLeft(); break;
                case Keyboard::Key::Left:       m_input.MoveWordLeft(); break;
                case Keyboard::Key::Right:      m_input.MoveWordRight(); break;

                case Keyboard::Key::Kp_7: if (numlock) break;
                case Keyboard::Key::Home:
                    m_input.MoveHome(); break;

                case Keyboard::Key::Kp_1: if (numlock) break;
                case Keyboard::Key::End:
                    m_input.MoveEnd(); break;

                default: break;
                }
            }
            else
            {
                switch (key)
                {
                case Keyboard::Key::Delete:      m_input.Delete(); break;
                case Keyboard::Key::Backspace:   m_input.DeleteLeft(); break;
                case Keyboard::Key::Left:        m_input.MoveLeft(); break;
                case Keyboard::Key::Right:       m_input.MoveRight(); break;

                case Keyboard::Key::Kp_7: if (numlock) break;
                case Keyboard::Key::Home:
                    m_input.MoveHome(); break;

                case Keyboard::Key::Kp_1: if (numlock) break;
                case Keyboard::Key::End:
                    m_input.MoveEnd(); break;

                default: break;
                }
            }
        }
    };

} // rob

#endif // H_ROB_TEXT_INPUT_H

