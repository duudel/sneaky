
#ifndef H_ROB_RENDERER_H
#define H_ROB_RENDERER_H

#include "../graphics/GraphicsTypes.h"
#include "../resource/ResourceID.h"
#include "Color.h"
#include "Font.h"

#include "../math/Types.h"
#include "../math/Matrix4.h"

#include "../memory/LinearAllocator.h"

namespace rob
{

    class Graphics;
    class MasterCache;
    class Font;

    struct GlobalUniforms
    {
        UniformHandle projection;
        UniformHandle model;
        UniformHandle position;
        UniformHandle time_ms;
        UniformHandle texture0;
    };

    struct Viewport
    {
        int x, y, w, h;
    };

    struct View
    {
        void SetViewport(int x, int y, int w, int h)
        {
            m_viewport.x = x; m_viewport.y = y;
            m_viewport.w = w; m_viewport.h = h;
        }

        Viewport m_viewport;
        mat4f m_projection;
    };

    struct FontVertex;

    class Renderer
    {
    public:
        Renderer(Graphics *graphics, MasterCache *cache, LinearAllocator &alloc);
        Renderer(const Renderer&) = delete;
        Renderer& operator = (const Renderer&) = delete;
        ~Renderer();

        ShaderProgramHandle CompileShaderProgram(const char * const vert, const char * const frag);

        Graphics* GetGraphics();
        const GlobalUniforms& GetGlobals() const;

        void SetView(const View &view);
        View GetView() const;

        void SetProjection(const mat4f &projection);
        void SetModel(const mat4f &model);
        void SetTime(uint64_t timeMicroseconds);

        void BindShader(ShaderProgramHandle shader);
        void BindColorShader();
        void BindTextureShader();
        void BindFontShader();

        void SetColor(const Color &color);

        void DrawLine(float x0, float y0, float x1, float y1);
        void DrawRectangle(float x0, float y0, float x1, float y1);
        void DrawFilledRectangle(float x0, float y0, float x1, float y1);
        void DrawTexturedRectangle(float x0, float y0, float x1, float y1);
        void DrawColorQuad(const vec2f &p0, const Color &color0, const vec2f &p1, const Color &color1,
                           const vec2f &p2, const Color &color2, const vec2f &p3, const Color &color3);
        void DrawCircle(float x, float y, float radius);
        void DrawFilledCircle(float x, float y, float radius);
        void DrawFilledCircle(float x, float y, float radius, const Color &center);

        void DrawText(float x, float y, const char *text);
        void DrawTextX(float x, float y, const char *text);
        float GetTextWidth(const char *text) const;
        float GetTextWidth(const char *text, size_t charCount) const;

        void DrawTextAscii(float x, float y, const char *text);
        float GetTextWidthAscii(const char *text) const;
        float GetTextWidthAscii(const char *text, size_t charCount) const;

        void SetFontScale(float scale);
        float GetFontScale() const;

        float GetFontHeight() const;
        float GetFontLineSpacing() const;

    private:
        void AddFontVertex(FontVertex *&vertex, const float x, const float y, const float u, const float v);
        void AddFontQuad(FontVertex *&vertex, const uint32_t c, const Glyph &glyph,
                           float &cursorX, float &cursorY,
                           const size_t textureW, const size_t textureH);
        void AddFontQuadX(FontVertex *&vertex, const uint32_t c, const Glyph &glyph,
                           float &cursorX, float &cursorY,
                           const size_t textureW, const size_t textureH, const bool super);

    private:
        LinearAllocator m_alloc;
        LinearAllocator m_vb_alloc;
        Graphics *m_graphics;

        GlobalUniforms m_globals;

        View m_view;

        VertexBufferHandle      m_vertexBuffer;
        ShaderProgramHandle     m_colorProgram;
        ShaderProgramHandle     m_textureProgram;
        ShaderProgramHandle     m_fontProgram;

        Color m_color;
        Font m_font;
        float m_fontScale;
    };

} // rob

#endif // H_ROB_RENDERER_H

