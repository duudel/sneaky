
#include "TextInput.h"
#include "../String.h"
#include "../Assert.h"

#include "../math/Functions.h"

#include <cctype>

#include <SDL2/SDL_clipboard.h>

namespace rob
{

    const char* GetClipboardText()
    { return ::SDL_GetClipboardText(); }

    void FreeClipboardText(const char *text)
    { ::SDL_free(const_cast<char*>(text)); }

    void SetClipboardText(const char *text)
    { ::SDL_SetClipboardText(text); }


    TextInput::TextInput()
        : m_length(0)
        , m_cursor(0)
        , m_lengthLimit(MAX_LENGTH)
    {
        for (size_t i = 0; i <= MAX_LENGTH; i++)
            m_text[i] = 0;
    }

    const char* TextInput::GetText() const
    { return m_text; }

    size_t TextInput::GetLength() const
    { return m_length; }

    size_t TextInput::GetCursor() const
    { return m_cursor; }

    void TextInput::SetLengthLimit(size_t limit)
    { m_lengthLimit = limit > MAX_LENGTH ? MAX_LENGTH : limit; }

    size_t TextInput::LengthLimit() const
    { return m_lengthLimit; }

    bool TextInput::MoveLeft()
    {
        if (m_cursor > 0)
        {
            const char *s = m_text + m_cursor;
            SkipUtf8Left(s, m_text);
            m_cursor = s - m_text;
            ROB_ASSERT(m_cursor <= m_length);
            return true;
        }
        return false;
    }

    bool TextInput::MoveRight()
    {
        if (m_cursor < m_length)
        {
            const char *s = m_text + m_cursor;
            SkipUtf8Right(s, m_text + MAX_LENGTH);
            m_cursor = s - m_text;
            ROB_ASSERT(m_cursor <= m_length);
            return true;
        }
        return false;
    }

    void TextInput::MoveWordLeft()
    {
        MoveLeft();
        while (m_cursor > 0)
        {
            const char *s = m_text + m_cursor;
            const uint32_t prev = SkipUtf8Left(s, m_text);
            if (::isspace(prev) || ::ispunct(prev))
                break;
            m_cursor = s - m_text;
            ROB_ASSERT(m_cursor <= m_length);
        }
    }

    void TextInput::MoveWordRight()
    {
        MoveRight();
        while (m_cursor < m_length)
        {
            const char *s = m_text + m_cursor;
            const uint32_t next = SkipUtf8Right(s, m_text + MAX_LENGTH);
            m_cursor = s - m_text;
            ROB_ASSERT(m_cursor <= m_length);
            if (::isspace(next) || ::ispunct(next))
                break;
        }
    }

    void TextInput::MoveHome()
    { m_cursor = 0; }

    void TextInput::MoveEnd()
    { m_cursor = m_length; }

    void TextInput::Insert(const char *str)
    {
        const size_t MAX_L = m_lengthLimit;
        const size_t slen = StringLength(str);
        size_t l = Min(m_length - 1, MAX_L - slen - 1);
        while (l >= m_cursor)
        {
            size_t sz = 0, sz2 = 0;
            const char *s = m_text + l;
            SkipUtf8Left(s, m_text, &sz);
            const char *s2 = s;
            SkipUtf8Right(s2, m_text + MAX_L, &sz2);

            if (sz != sz2)
            {
                char *b = m_text + (s - m_text);
                while (b != s2)
                    *b++ = 0;
            }
            else
            {
                for (size_t i = 0; i < sz; i++)
                    m_text[l + slen - i] = m_text[l - i];
            }

            const size_t diff = (m_text + l) - s;
            if (l == 0) break;
            l -= diff;
        }
//        const size_t maxCopy = Min(slen, MAX_LENGTH - m_cursor);
//        m_cursor += CopyUtf8_N(m_text + m_cursor, str, MAX_LENGTH - m_cursor);
        m_cursor += CopyUtf8(m_text + m_cursor, str, MAX_L - m_cursor);
        m_length = Min(m_length + slen, MAX_L);

        ROB_ASSERT(m_cursor <= m_length);
        ROB_ASSERT(m_length <= MAX_LENGTH);
    }

    void TextInput::Delete()
    {
        if (m_cursor < m_length)
        {
            size_t oldCursor = m_cursor;
            MoveRight();
//            CopyString(m_text + oldCursor, m_text + m_cursor, MAX_LENGTH + 1 - oldCursor);
            CopyUtf8_N(m_text + oldCursor, m_text + m_cursor, MAX_LENGTH - oldCursor);
            m_length -= (m_cursor - oldCursor);
            m_cursor = oldCursor;
        }
    }

    void TextInput::DeleteLeft()
    {
        if (m_cursor > 0)
        {
            size_t oldCursor = m_cursor;
            MoveLeft();
//            CopyString(m_text + m_cursor, m_text + oldCursor, MAX_LENGTH + 1 - oldCursor);
            CopyUtf8_N(m_text + m_cursor, m_text + oldCursor, MAX_LENGTH - m_cursor);
            m_length -= (oldCursor - m_cursor);
        }
    }

    void TextInput::DeleteWord()
    {
        if (m_cursor < m_length)
        {
            size_t oldCursor = m_cursor;
            MoveWordRight();
//            CopyString(m_text + oldCursor, m_text + m_cursor, MAX_LENGTH + 1 - oldCursor);
            CopyUtf8_N(m_text + oldCursor, m_text + m_cursor, MAX_LENGTH - oldCursor);
            m_length -= (m_cursor - oldCursor);
            m_cursor = oldCursor;
        }
    }

    void TextInput::DeleteWordLeft()
    {
        if (m_cursor > 0)
        {
            size_t oldCursor = m_cursor;
            MoveWordLeft();
//            CopyString(m_text + m_cursor, m_text + oldCursor, MAX_LENGTH + 1 - oldCursor);
            CopyUtf8_N(m_text + m_cursor, m_text + oldCursor, MAX_LENGTH - m_cursor);
            m_length -= (oldCursor - m_cursor);
        }
    }

} // rob
