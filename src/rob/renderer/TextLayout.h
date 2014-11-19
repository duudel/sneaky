
#ifndef H_ROB_TEXT_LAYOUT_H
#define H_ROB_TEXT_LAYOUT_H

#include "Renderer.h"
#include "../input/TextInput.h"

namespace rob
{

    struct TextLayout
    {
        Renderer &m_renderer;
        vec2f m_cursor;
        vec2f m_start;

        TextLayout(Renderer &renderer, float x, float y)
            : m_renderer(renderer)
            , m_cursor(x, y)
            , m_start(x, y)
        { }

        void AddLine()
        {
            AddLines(1);
        }

        void AddLines(int n)
        {
            m_cursor.x = m_start.x;
            m_cursor.y += m_renderer.GetFontHeight() * n;
        }

        void AddText(const char *str, const float width)
        {
            m_renderer.DrawText(m_cursor.x + width, m_cursor.y, str);
            m_cursor.x += width + m_renderer.GetTextWidth(str);
        }

        void AddTextX(const char *str, const float width)
        {
            m_renderer.DrawTextX(m_cursor.x + width, m_cursor.y, str);
            m_cursor.x += width + m_renderer.GetTextWidth(str);
        }

        void DrawCursor(const float x, const float y)
        {
            m_renderer.BindColorShader();
            m_renderer.DrawLine(x, y, x, y + m_renderer.GetFontHeight());
            m_renderer.BindFontShader();
        }

        void AddTextInput(const TextInput &input, const float width)
        {
            const float cx = m_cursor.x + width;
            const float cy = m_cursor.y;
            m_renderer.DrawText(cx, cy, input.GetText());

            const float cursorX = m_renderer.GetTextWidth(input.GetText(), input.GetCursor());
            DrawCursor(cx + cursorX, cy);

            m_cursor.x = cx + m_renderer.GetTextWidth(input.GetText());
        }

        void AddTextAlignL(const char *str, const float width)
        { m_renderer.DrawText(m_cursor.x + width, m_cursor.y, str); }

        void AddTextXAlignL(const char *str, const float width)
        { m_renderer.DrawTextX(m_cursor.x + width, m_cursor.y, str); }

        void AddTextInputAlignL(const TextInput &input, const float width)
        {
            const float cx = m_cursor.x + width;
            const float cy = m_cursor.y;
            m_renderer.DrawText(cx, cy, input.GetText());

            const float cursorX = m_renderer.GetTextWidth(input.GetText(), input.GetCursor());
            DrawCursor(cx + cursorX, cy);
        }

        void AddTextAlignC(const char *str, const float width)
        {
            const float tw = m_renderer.GetTextWidth(str);
            m_renderer.DrawText(m_cursor.x + (width - tw / 2.0f), m_cursor.y, str);
            m_cursor.x += width;
        }

        void AddTextXAlignC(const char *str, const float width)
        {
            const float tw = m_renderer.GetTextWidth(str);
            m_renderer.DrawTextX(m_cursor.x + (width - tw / 2.0f), m_cursor.y, str);
            m_cursor.x += width;
        }

        void AddTextInputAlignC(const TextInput &input, const float width)
        {
            const float tw = m_renderer.GetTextWidth(input.GetText());
            const float cx = m_cursor.x + (width - tw / 2.0f);
            const float cy = m_cursor.y;
            m_renderer.DrawText(cx, cy, input.GetText());

            const float cursorX = m_renderer.GetTextWidth(input.GetText(), input.GetCursor());
            DrawCursor(cx + cursorX, cy);

            m_cursor.x += width;
        }

        void AddTextAlignR(const char *str, const float width)
        {
            const float tw = m_renderer.GetTextWidth(str);
            m_renderer.DrawText(m_cursor.x + width - tw, m_cursor.y, str);
            m_cursor.x += width;
        }

        void AddTextXAlignR(const char *str, const float width)
        {
            const float tw = m_renderer.GetTextWidth(str);
            m_renderer.DrawTextX(m_cursor.x + width - tw, m_cursor.y, str);
            m_cursor.x += width;
        }

        void AddTextInputAlignR(const TextInput &input, const float width)
        {
            const float tw = m_renderer.GetTextWidth(input.GetText());
            const float cx = m_cursor.x + width - tw;
            const float cy = m_cursor.y;
            m_renderer.DrawText(cx, cy, input.GetText());

            const float cursorX = m_renderer.GetTextWidth(input.GetText(), input.GetCursor());
            DrawCursor(cx + cursorX, cy);

            m_cursor.x += width;
        }
    };

} // rob

#endif // H_ROB_TEXT_LAYOUT_H


