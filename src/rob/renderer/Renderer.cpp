
#include "Renderer.h"
#include "../graphics/Graphics.h"
#include "../graphics/Shader.h"
#include "../graphics/ShaderProgram.h"
#include "../graphics/VertexBuffer.h"
#include "../graphics/Texture.h"

#include "../resource/MasterCache.h"

#include "../math/Math.h"

#include "../Log.h"
#include "../String.h"

#include <GL/glew.h>

namespace rob
{

    extern const char * const g_colorVertexShader;
    extern const char * const g_colorFragmentShader;
    extern const char * const g_textureVertexShader;
    extern const char * const g_textureFragmentShader;
    extern const char * const g_fontVertexShader;
    extern const char * const g_fontFragmentShader;

    struct ColorVertex
    {
        float x, y;
        float r, g, b, a;
    };

    struct FontVertex
    {
        float x, y, u, v;
        float r, g, b, a;
    };

    struct TextureVertex
    {
        float x, y, u, v;
        float r, g, b, a;
    };


    ShaderProgramHandle Renderer::CompileShaderProgram(const char * const vert, const char * const frag)
    {
        VertexShaderHandle vs = m_graphics->CreateVertexShader();
        VertexShader *vertShader = m_graphics->GetVertexShader(vs);
        vertShader->SetSource(vert);
        if (!vertShader->Compile())
        {
            char buffer[512];
            vertShader->GetCompileInfo(buffer, 512);
            log::Error(&buffer[0]);
            m_graphics->DestroyVertexShader(vs);
            return InvalidHandle;
        }

        FragmentShaderHandle fs = m_graphics->CreateFragmentShader();
        FragmentShader *fragShader = m_graphics->GetFragmentShader(fs);
        fragShader->SetSource(frag);
        if (!fragShader->Compile())
        {
            char buffer[512];
            fragShader->GetCompileInfo(buffer, 512);
            log::Error(&buffer[0]);
            m_graphics->DestroyVertexShader(vs);
            m_graphics->DestroyFragmentShader(fs);
            return InvalidHandle;
        }

        ShaderProgramHandle p = m_graphics->CreateShaderProgram();
        ShaderProgram *program = m_graphics->GetShaderProgram(p);
        program->SetShaders(vertShader, fragShader);
        if (!program->Link())
        {
            char buffer[512];
            program->GetLinkInfo(buffer, 512);
            log::Error(&buffer[0]);
            m_graphics->DestroyVertexShader(vs);
            m_graphics->DestroyFragmentShader(fs);
            m_graphics->DestroyFragmentShader(p);
            return InvalidHandle;
        }

        m_graphics->DestroyVertexShader(vs);
        m_graphics->DestroyFragmentShader(fs);

        m_graphics->AddProgramUniform(p, m_globals.projection);
        m_graphics->AddProgramUniform(p, m_globals.model);
        m_graphics->AddProgramUniform(p, m_globals.position);
        m_graphics->AddProgramUniform(p, m_globals.time_ms);
        m_graphics->AddProgramUniform(p, m_globals.texture0);
        return p;
    }

    static const size_t RENDERER_MEMORY = 4 * 1024;
    static const size_t MAX_VERTEX_BUFFER_SIZE = 1 * 1024 * 1024;

    Renderer::Renderer(Graphics *graphics, MasterCache *cache, LinearAllocator &alloc)
        : m_alloc(alloc.Allocate(RENDERER_MEMORY), RENDERER_MEMORY)
        , m_vb_alloc(alloc.Allocate(MAX_VERTEX_BUFFER_SIZE), MAX_VERTEX_BUFFER_SIZE)
        , m_graphics(graphics)
        , m_globals()
        , m_vertexBuffer(InvalidHandle)
        , m_colorProgram(InvalidHandle)
        , m_textureProgram(InvalidHandle)
        , m_fontProgram(InvalidHandle)
        , m_color(Color::White)
        , m_font()
        , m_fontScale(1.0f)
    {
        m_globals.projection    = m_graphics->CreateGlobalUniform("u_projection", UniformType::Mat4);
        m_globals.model         = m_graphics->CreateGlobalUniform("u_model", UniformType::Mat4);
        m_globals.position      = m_graphics->CreateGlobalUniform("u_position", UniformType::Vec4);
        m_globals.time_ms       = m_graphics->CreateGlobalUniform("u_time_ms", UniformType::Int);
        m_globals.texture0      = m_graphics->CreateGlobalUniform("u_texture0", UniformType::Int);
        m_graphics->SetUniform(m_globals.projection, mat4f::Identity);
        m_graphics->SetUniform(m_globals.model, mat4f::Identity);
        m_graphics->SetUniform(m_globals.time_ms, 0);
        m_graphics->SetUniform(m_globals.texture0, 0);

        m_colorProgram = CompileShaderProgram(g_colorVertexShader, g_colorFragmentShader);
        m_textureProgram = CompileShaderProgram(g_textureVertexShader, g_textureFragmentShader);
        m_fontProgram = CompileShaderProgram(g_fontVertexShader, g_fontFragmentShader);

//        m_font = cache->GetFont("lucida_24.fnt");
//        m_font = cache->GetFont("dejavu_24.fnt");
//        m_font = cache->GetFont("dejavu_96.fnt");
        m_font = cache->GetFont("dejavu_192.fnt");

        m_vertexBuffer = m_graphics->CreateVertexBuffer();
        m_graphics->BindVertexBuffer(m_vertexBuffer);
        VertexBuffer *vb = m_graphics->GetVertexBuffer(m_vertexBuffer);
        vb->Resize(MAX_VERTEX_BUFFER_SIZE, false);
    }

    Renderer::~Renderer()
    {
        m_graphics->DestroyVertexBuffer(m_vertexBuffer);
        if (m_colorProgram != InvalidHandle)
            m_graphics->DestroyShaderProgram(m_colorProgram);
        if (m_textureProgram != InvalidHandle)
            m_graphics->DestroyShaderProgram(m_textureProgram);
        if (m_fontProgram != InvalidHandle)
            m_graphics->DestroyShaderProgram(m_fontProgram);
        m_graphics->DecRefUniform(m_globals.projection);
        m_graphics->DecRefUniform(m_globals.model);
        m_graphics->DecRefUniform(m_globals.position);
        m_graphics->DecRefUniform(m_globals.time_ms);
        m_graphics->DecRefUniform(m_globals.texture0);
    }

    Graphics* Renderer::GetGraphics()
    { return m_graphics; }

    const GlobalUniforms& Renderer::GetGlobals() const
    { return m_globals; }

    void Renderer::SetProjection(const mat4f &projection)
    { m_graphics->SetUniform(m_globals.projection, projection); }

    void Renderer::SetModel(const mat4f &model)
    { m_graphics->SetUniform(m_globals.model, model); }

    void Renderer::SetView(const View &view)
    {
        m_view = view;
        m_graphics->SetViewport(m_view.m_viewport.x,
                                m_view.m_viewport.y,
                                m_view.m_viewport.w,
                                m_view.m_viewport.h);
        SetProjection(m_view.m_projection);
    }

    View Renderer::GetView() const
    { return m_view; }

    /// Set shader time in micro seconds. The time passed to shaders will be
    /// a fixed point number scaled to units of 1ms(=0.001s). The
    /// time will wrap every 100 000s ~= 55h. This is unavoidable as the shader
    ///  can only handle 32bit signed integers, and we don't want negative time
    /// due to int wrapping. So we wrap it explicitly before that happens. This
    /// means the time in shaders should never be multiplied by a factor greater
    /// than 20 as it would cause the time to wrap near the explicit wrap limit
    /// causing negative numbers. (NOTE: if the time is converted to float,
    /// there is no problems with wrapping, but the precision is lost).
    void Renderer::SetTime(uint64_t timeMicroseconds)
    {
        uint32_t wrapMax = 100000000u;
        uint32_t utime_ms = timeMicroseconds / 1000ull;
        int32_t time_ms = utime_ms % wrapMax;
        m_graphics->SetUniform(m_globals.time_ms, time_ms);
    }


    void Renderer::BindShader(ShaderProgramHandle shader)
    { m_graphics->BindShaderProgram(shader); }

    void Renderer::BindColorShader()
    { BindShader(m_colorProgram); }

    void Renderer::BindTextureShader()
    { BindShader(m_textureProgram); }

    void Renderer::BindFontShader()
    { BindShader(m_fontProgram); }

    void Renderer::SetColor(const Color &color)
    { m_color = color; }

    void Renderer::DrawLine(float x0, float y0, float x1, float y1)
    {
        const size_t vertexCount = 2;
        ColorVertex* vertices = m_vb_alloc.AllocateArray<ColorVertex>(vertexCount);
        vertices[0] = { x0, y0, m_color.r, m_color.g, m_color.b, m_color.a };
        vertices[1] = { x1, y1, m_color.r, m_color.g, m_color.b, m_color.a };

        m_graphics->SetUniform(m_globals.position, vec4f(x0, y0, 0.0f, 1.0f));

        m_graphics->BindVertexBuffer(m_vertexBuffer);
        VertexBuffer *buffer = m_graphics->GetVertexBuffer(m_vertexBuffer);
        buffer->Write(0, vertexCount * sizeof(ColorVertex), vertices);
        m_graphics->SetAttrib(0, 2, sizeof(ColorVertex), 0);
        m_graphics->SetAttrib(1, 4, sizeof(ColorVertex), sizeof(float) * 2);
        m_graphics->DrawLineLoopArrays(0, vertexCount);

        m_vb_alloc.Reset();
    }

    void Renderer::DrawRectangle(float x0, float y0, float x1, float y1)
    {
        const size_t vertexCount = 4;
        ColorVertex* vertices = m_vb_alloc.AllocateArray<ColorVertex>(vertexCount);
        vertices[0] = { x0, y0, m_color.r, m_color.g, m_color.b, m_color.a };
        vertices[1] = { x1, y0, m_color.r, m_color.g, m_color.b, m_color.a };
        vertices[2] = { x1, y1, m_color.r, m_color.g, m_color.b, m_color.a };
        vertices[3] = { x0, y1, m_color.r, m_color.g, m_color.b, m_color.a };

        m_graphics->SetUniform(m_globals.position, vec4f(x0, y0, 0.0f, 1.0f));

        m_graphics->BindVertexBuffer(m_vertexBuffer);
        VertexBuffer *buffer = m_graphics->GetVertexBuffer(m_vertexBuffer);
        buffer->Write(0, vertexCount * sizeof(ColorVertex), vertices);
        m_graphics->SetAttrib(0, 2, sizeof(ColorVertex), 0);
        m_graphics->SetAttrib(1, 4, sizeof(ColorVertex), sizeof(float) * 2);
        m_graphics->DrawLineLoopArrays(0, vertexCount);

        m_vb_alloc.Reset();
    }

    void Renderer::DrawFilledRectangle(float x0, float y0, float x1, float y1)
    {
        const size_t vertexCount = 4;
        ColorVertex* vertices = m_vb_alloc.AllocateArray<ColorVertex>(vertexCount);
        vertices[0] = { x0, y0, m_color.r, m_color.g, m_color.b, m_color.a };
        vertices[1] = { x1, y0, m_color.r, m_color.g, m_color.b, m_color.a };
        vertices[2] = { x0, y1, m_color.r, m_color.g, m_color.b, m_color.a };
        vertices[3] = { x1, y1, m_color.r, m_color.g, m_color.b, m_color.a };

        m_graphics->SetUniform(m_globals.position, vec4f(x0, y0, 0.0f, 1.0f));

        m_graphics->BindVertexBuffer(m_vertexBuffer);
        VertexBuffer *buffer = m_graphics->GetVertexBuffer(m_vertexBuffer);
        buffer->Write(0, vertexCount * sizeof(ColorVertex), vertices);
        m_graphics->SetAttrib(0, 2, sizeof(ColorVertex), 0);
        m_graphics->SetAttrib(1, 4, sizeof(ColorVertex), sizeof(float) * 2);
        m_graphics->DrawTriangleStripArrays(0, vertexCount);

        m_vb_alloc.Reset();
    }

    void Renderer::DrawTexturedRectangle(float x0, float y0, float x1, float y1)
    {
        const size_t vertexCount = 4;
        TextureVertex* vertices = m_vb_alloc.AllocateArray<TextureVertex>(vertexCount);
        vertices[0] = { x0, y0, 0.0f, 0.0f, m_color.r, m_color.g, m_color.b, m_color.a };
        vertices[1] = { x1, y0, 1.0f, 0.0f, m_color.r, m_color.g, m_color.b, m_color.a };
        vertices[2] = { x0, y1, 0.0f, 1.0f, m_color.r, m_color.g, m_color.b, m_color.a };
        vertices[3] = { x1, y1, 1.0f, 1.0f, m_color.r, m_color.g, m_color.b, m_color.a };

        m_graphics->SetUniform(m_globals.position, vec4f(x0, y0, 0.0f, 1.0f));

        m_graphics->BindVertexBuffer(m_vertexBuffer);
        VertexBuffer *buffer = m_graphics->GetVertexBuffer(m_vertexBuffer);
        buffer->Write(0, vertexCount * sizeof(TextureVertex), vertices);
        m_graphics->SetAttrib(0, 4, sizeof(TextureVertex), 0);
        m_graphics->SetAttrib(1, 4, sizeof(TextureVertex), sizeof(float) * 4);
        m_graphics->DrawTriangleStripArrays(0, vertexCount);

        m_vb_alloc.Reset();
    }

    void Renderer::DrawColorQuad(const vec2f &p0, const Color &color0, const vec2f &p1, const Color &color1,
                                 const vec2f &p2, const Color &color2, const vec2f &p3, const Color &color3)
    {
        const size_t vertexCount = 4;
        ColorVertex* vertices = m_vb_alloc.AllocateArray<ColorVertex>(vertexCount);
        vertices[0] = { p0.x, p0.y, color0.r, color0.g, color0.b, color0.a };
        vertices[1] = { p1.x, p1.y, color1.r, color1.g, color1.b, color1.a };
        vertices[2] = { p3.x, p3.y, color3.r, color3.g, color3.b, color3.a };
        vertices[3] = { p2.x, p2.y, color2.r, color2.g, color2.b, color2.a };

        m_graphics->SetUniform(m_globals.position, vec4f(p0.x, p0.y, 0.0f, 1.0f));

        m_graphics->BindVertexBuffer(m_vertexBuffer);
        VertexBuffer *buffer = m_graphics->GetVertexBuffer(m_vertexBuffer);
        buffer->Write(0, vertexCount * sizeof(ColorVertex), vertices);
        m_graphics->SetAttrib(0, 2, sizeof(ColorVertex), 0);
        m_graphics->SetAttrib(1, 4, sizeof(ColorVertex), sizeof(float) * 2);
        m_graphics->DrawTriangleStripArrays(0, vertexCount);

        m_vb_alloc.Reset();
    }

    static const size_t CIRCLE_SEGMENTS = 48;
    static const float SEG_RADIUS_SCALE = 1.0f;

    void Renderer::DrawCircle(float x, float y, float radius)
    {
        const size_t segs = CIRCLE_SEGMENTS * (radius / SEG_RADIUS_SCALE);
        const size_t segments = Min((segs + 3) & ~0x3, CIRCLE_SEGMENTS);
        const size_t quarter = segments / 4;
        const size_t vertexCount = segments;
        ColorVertex* vertices = m_vb_alloc.AllocateArray<ColorVertex>(vertexCount);

        float angle = 0.0f;
        const float deltaAngle = 2.0f * PI_f / segments;
        for (size_t i = 0; i < quarter; i++, angle += deltaAngle)
        {
            float sn, cs;
            rob::FastSinCos(angle, sn, cs);
            sn *= radius;
            cs *= radius;

            const size_t i0 = i;
            const size_t i1 = i0 + quarter;
            const size_t i2 = i1 + quarter;
            const size_t i3 = i2 + quarter;
            vertices[i0] = { x-cs, y-sn, m_color.r, m_color.g, m_color.b, m_color.a };
            vertices[i1] = { x+sn, y-cs, m_color.r, m_color.g, m_color.b, m_color.a };
            vertices[i2] = { x+cs, y+sn, m_color.r, m_color.g, m_color.b, m_color.a };
            vertices[i3] = { x-sn, y+cs, m_color.r, m_color.g, m_color.b, m_color.a };
        };

        m_graphics->SetUniform(m_globals.position, vec4f(x, y, 0.0f, 1.0f));

        m_graphics->BindVertexBuffer(m_vertexBuffer);
        VertexBuffer *buffer = m_graphics->GetVertexBuffer(m_vertexBuffer);
        buffer->Write(0, vertexCount * sizeof(ColorVertex), vertices);
        m_graphics->SetAttrib(0, 2, sizeof(ColorVertex), 0);
        m_graphics->SetAttrib(1, 4, sizeof(ColorVertex), sizeof(float) * 2);
        m_graphics->DrawLineLoopArrays(0, vertexCount);

        m_vb_alloc.Reset();
    }

    void Renderer::DrawFilledCircle(float x, float y, float radius)
    {
        const size_t segs = CIRCLE_SEGMENTS * (radius / SEG_RADIUS_SCALE);
        const size_t segments = Min((segs + 3) & ~0x3, CIRCLE_SEGMENTS);
        const size_t quarter = segments / 4;
        const size_t vertexCount = segments + 2;
        ColorVertex* vertices = m_vb_alloc.AllocateArray<ColorVertex>(vertexCount);

        float angle = 0.0f;
        const float deltaAngle = 2.0f * PI_f / segments;
        vertices[0] = { x+0.0f, y+0.0f, m_color.r, m_color.g, m_color.b, m_color.a };
        for (size_t i = 0; i < quarter; i++, angle += deltaAngle)
        {
            float sn, cs;
            rob::FastSinCos(angle, sn, cs);
            sn *= radius;
            cs *= radius;

            const size_t i0 = 1 + i;
            const size_t i1 = i0 + quarter;
            const size_t i2 = i1 + quarter;
            const size_t i3 = i2 + quarter;
            vertices[i0] = { x-cs, y-sn, m_color.r, m_color.g, m_color.b, m_color.a };
            vertices[i1] = { x+sn, y-cs, m_color.r, m_color.g, m_color.b, m_color.a };
            vertices[i2] = { x+cs, y+sn, m_color.r, m_color.g, m_color.b, m_color.a };
            vertices[i3] = { x-sn, y+cs, m_color.r, m_color.g, m_color.b, m_color.a };
        };
        vertices[2 + segments - 1] = vertices[1];

        m_graphics->SetUniform(m_globals.position, vec4f(x, y, 0.0f, 1.0f));

        m_graphics->BindVertexBuffer(m_vertexBuffer);
        VertexBuffer *buffer = m_graphics->GetVertexBuffer(m_vertexBuffer);
        buffer->Write(0, vertexCount * sizeof(ColorVertex), vertices);
        m_graphics->SetAttrib(0, 2, sizeof(ColorVertex), 0);
        m_graphics->SetAttrib(1, 4, sizeof(ColorVertex), sizeof(float) * 2);
        m_graphics->DrawTriangleFanArrays(0, vertexCount);

        m_vb_alloc.Reset();
    }

    void Renderer::DrawFilledCircle(float x, float y, float radius, const Color &center)
    {
        const size_t segs = CIRCLE_SEGMENTS * (radius / SEG_RADIUS_SCALE);
        const size_t segments = Min((segs + 3) & ~0x3, CIRCLE_SEGMENTS);
        const size_t quarter = segments / 4;
        const size_t vertexCount = segments + 2;
        ColorVertex* vertices = m_vb_alloc.AllocateArray<ColorVertex>(vertexCount);

        float angle = 0.0f;
        const float deltaAngle = 2.0f * PI_f / segments;
        vertices[0] = { x+0.0f, y+0.0f, center.r, center.g, center.b, center.a };
        for (size_t i = 0; i < quarter; i++, angle += deltaAngle)
        {
            float sn, cs;
            rob::FastSinCos(angle, sn, cs);
            sn *= radius;
            cs *= radius;

            const size_t i0 = 1 + i;
            const size_t i1 = i0 + quarter;
            const size_t i2 = i1 + quarter;
            const size_t i3 = i2 + quarter;
            vertices[i0] = { x-cs, y-sn, m_color.r, m_color.g, m_color.b, m_color.a };
            vertices[i1] = { x+sn, y-cs, m_color.r, m_color.g, m_color.b, m_color.a };
            vertices[i2] = { x+cs, y+sn, m_color.r, m_color.g, m_color.b, m_color.a };
            vertices[i3] = { x-sn, y+cs, m_color.r, m_color.g, m_color.b, m_color.a };
        };
        vertices[2 + segments - 1] = vertices[1];

        m_graphics->SetUniform(m_globals.position, vec4f(x, y, 0.0f, 1.0f));

        m_graphics->BindVertexBuffer(m_vertexBuffer);
        VertexBuffer *buffer = m_graphics->GetVertexBuffer(m_vertexBuffer);
        buffer->Write(0, vertexCount * sizeof(ColorVertex), vertices);
        m_graphics->SetAttrib(0, 2, sizeof(ColorVertex), 0);
        m_graphics->SetAttrib(1, 4, sizeof(ColorVertex), sizeof(float) * 2);
        m_graphics->DrawTriangleFanArrays(0, vertexCount);

        m_vb_alloc.Reset();
    }


    void Renderer::AddFontVertex(FontVertex *&vertex, const float x, const float y, const float u, const float v)
    {
        FontVertex &vert = *vertex++;
        vert.x = x; vert.y = y; vert.u = u; vert.v = v;
        vert.r = m_color.r; vert.g = m_color.g; vert.b = m_color.b; vert.a = m_color.a;
    }

    void Renderer::AddFontQuad(FontVertex *&vertex, const uint32_t c, const Glyph &glyph,
                               float &cursorX, float &cursorY,
                               const size_t textureW, const size_t textureH)
    {
        if (c > ' ')
        {
            const float gW = float(glyph.m_width) * m_fontScale;
            const float gH = float(glyph.m_height) * m_fontScale;
            const float uvW = float(glyph.m_width) / textureW;
            const float uvH = -float(glyph.m_height) / textureH;

            const float uvX = float(glyph.m_x) / textureW;
            const float uvY = -float(glyph.m_y) / textureH;

            const float cX = cursorX + glyph.m_offsetX * m_fontScale;
            const float cY = cursorY + glyph.m_offsetY * m_fontScale;

            AddFontVertex(vertex, cX,       cY,         uvX,        uvY);
            AddFontVertex(vertex, cX + gW,  cY,         uvX + uvW,  uvY);
            AddFontVertex(vertex, cX,       cY + gH,    uvX,        uvY + uvH);
            AddFontVertex(vertex, cX,       cY + gH,    uvX,        uvY + uvH);
            AddFontVertex(vertex, cX + gW,  cY,         uvX + uvW,  uvY);
            AddFontVertex(vertex, cX + gW,  cY + gH,    uvX + uvW,  uvY + uvH);
        }

        cursorX += float(glyph.m_advance) * m_fontScale;
    }

    void Renderer::AddFontQuadX(FontVertex *&vertex, const uint32_t c, const Glyph &glyph,
                               float &cursorX, float &cursorY,
                               const size_t textureW, const size_t textureH, const bool super)
    {
        if (c > ' ')
        {
            const float superF = super ? 0.6f : 1.0f;
            const float gW = float(glyph.m_width) * m_fontScale * superF;
            const float gH = float(glyph.m_height) * m_fontScale * superF;
            const float uvW = float(glyph.m_width) / textureW;
            const float uvH = -float(glyph.m_height) / textureH;

            const float uvX = float(glyph.m_x) / textureW;
            const float uvY = -float(glyph.m_y) / textureH;

            const float cX = cursorX + glyph.m_offsetX * m_fontScale;
            const float cY = cursorY + glyph.m_offsetY * m_fontScale * superF;

            AddFontVertex(vertex, cX,       cY,         uvX,        uvY);
            AddFontVertex(vertex, cX + gW,  cY,         uvX + uvW,  uvY);
            AddFontVertex(vertex, cX,       cY + gH,    uvX,        uvY + uvH);
            AddFontVertex(vertex, cX,       cY + gH,    uvX,        uvY + uvH);
            AddFontVertex(vertex, cX + gW,  cY,         uvX + uvW,  uvY);
            AddFontVertex(vertex, cX + gW,  cY + gH,    uvX + uvW,  uvY + uvH);
        }

        cursorX += float(glyph.m_advance) * m_fontScale;
    }

    void Renderer::DrawText(float x, float y, const char *text)
    {
        if (!m_font.IsReady()) return;

        m_graphics->SetUniform(m_globals.position, vec4f(x, y, 0.0f, 1.0f));
        m_graphics->SetUniform(m_globals.texture0, 0);
        m_graphics->BindVertexBuffer(m_vertexBuffer);
        m_graphics->SetAttrib(0, 4, sizeof(FontVertex), 0);
        m_graphics->SetAttrib(1, 4, sizeof(FontVertex), sizeof(float) * 4);
        VertexBuffer *buffer = m_graphics->GetVertexBuffer(m_vertexBuffer);

        const size_t textLen = StringLength(text);
        const size_t maxVertexCount = textLen * 6;
        FontVertex * const verticesStart = m_vb_alloc.AllocateArray<FontVertex>(maxVertexCount);
        float cursorX = x;
        float cursorY = y;

//        const char * const start = text;
        const char * const end = text + textLen;
        if (text != end)
        {
            bool oneMore = false;
            uint32_t c = DecodeUtf8(text, end);
            do
            {
                FontVertex *vertex = verticesStart;

                const Glyph &glyph = m_font.GetGlyph(c);
                uint16_t texturePage = glyph.m_textureIdx;
                const TextureHandle textureHandle = m_font.GetTexture(texturePage);
                const Texture *texture = m_graphics->GetTexture(textureHandle);

                const size_t textureW = texture->GetWidth();
                const size_t textureH = texture->GetHeight();

                AddFontQuad(vertex, c, glyph, cursorX, cursorY, textureW, textureH);

                oneMore = false;
                while (text != end)
                {
                    c = DecodeUtf8(text, end);
                    const Glyph &glyph = m_font.GetGlyph(c);
                    if (glyph.m_textureIdx != texturePage)
                    {
                        oneMore = true;
                        break;
                    }
                    AddFontQuad(vertex, c, glyph, cursorX, cursorY, textureW, textureH);
                }

                const size_t vertexCount = vertex - verticesStart;
                buffer->Write(0, vertexCount * sizeof(FontVertex), verticesStart);

                m_graphics->BindTexture(0, textureHandle);
                m_graphics->DrawTriangleArrays(0, vertexCount);
            } while (oneMore || text != end);
        }
        m_vb_alloc.Reset();
    }

    void Renderer::DrawTextX(float x, float y, const char *text)
    {
        if (!m_font.IsReady()) return;

        m_graphics->SetUniform(m_globals.position, vec4f(x, y, 0.0f, 1.0f));
        m_graphics->SetUniform(m_globals.texture0, 0);
        m_graphics->BindVertexBuffer(m_vertexBuffer);
        m_graphics->SetAttrib(0, 4, sizeof(FontVertex), 0);
        m_graphics->SetAttrib(1, 4, sizeof(FontVertex), sizeof(float) * 4);
        VertexBuffer *buffer = m_graphics->GetVertexBuffer(m_vertexBuffer);

        const size_t textLen = StringLength(text);
        const size_t maxVertexCount = textLen * 6;
        FontVertex * const verticesStart = m_vb_alloc.AllocateArray<FontVertex>(maxVertexCount);
        float cursorX = x;
        float cursorY = y;

//        const char * const start = text;
        const char * const end = text + textLen;
        if (text != end)
        {
            bool oneMore = false;
            uint32_t c = DecodeUtf8(text, end);
            bool super = (c == '^');
            if (super && text != end) c = DecodeUtf8(text, end);
            do
            {
                FontVertex *vertex = verticesStart;

                const Glyph &glyph = m_font.GetGlyph(c);
                uint16_t texturePage = glyph.m_textureIdx;
                const TextureHandle textureHandle = m_font.GetTexture(texturePage);
                const Texture *texture = m_graphics->GetTexture(textureHandle);

                const size_t textureW = texture->GetWidth();
                const size_t textureH = texture->GetHeight();

                AddFontQuadX(vertex, c, glyph, cursorX, cursorY, textureW, textureH, super);

                oneMore = false;
                while (text != end)
                {
                    c = DecodeUtf8(text, end);
                    super = (c == '^');
                    if (super && text != end) c = DecodeUtf8(text, end);
                    const Glyph &glyph = m_font.GetGlyph(c);
                    if (glyph.m_textureIdx != texturePage)
                    {
                        oneMore = true;
                        break;
                    }
                    AddFontQuadX(vertex, c, glyph, cursorX, cursorY, textureW, textureH, super);
                }

                const size_t vertexCount = vertex - verticesStart;
                buffer->Write(0, vertexCount * sizeof(FontVertex), verticesStart);

                m_graphics->BindTexture(0, textureHandle);
                m_graphics->DrawTriangleArrays(0, vertexCount);
            } while (oneMore || text != end);
        }
        m_vb_alloc.Reset();
    }

    float Renderer::GetTextWidth(const char *text) const
    {
        float width = 0.0f;
        while (*text)
        {
            const uint32_t c = DecodeUtf8(text, 0);
            const Glyph &glyph = m_font.GetGlyph(c);
            width += float(glyph.m_advance) * m_fontScale;
        }
        return width;
    }

    float Renderer::GetTextWidth(const char *text, size_t charCount) const
    {
        float width = 0.0f;
        int charC = charCount;
        for (; *text && charC > 0; )
        {
            const char * const s = text;
            const uint32_t c = DecodeUtf8(text, 0);
            charC -= (text - s);
            const Glyph &glyph = m_font.GetGlyph(c);
            width += float(glyph.m_advance) * m_fontScale;
        }
        return width;
    }

    void Renderer::DrawTextAscii(float x, float y, const char *text)
    {
        if (!m_font.IsReady()) return;

        m_graphics->SetUniform(m_globals.position, vec4f(x, y, 0.0f, 1.0f));
        m_graphics->SetUniform(m_globals.texture0, 0);
        m_graphics->BindVertexBuffer(m_vertexBuffer);
        m_graphics->SetAttrib(0, 4, sizeof(FontVertex), 0);
        m_graphics->SetAttrib(1, 4, sizeof(FontVertex), sizeof(float) * 4);
        VertexBuffer *buffer = m_graphics->GetVertexBuffer(m_vertexBuffer);

        const size_t textLen = StringLength(text);
        const size_t maxVertexCount = textLen * 6;
        FontVertex * const verticesStart = m_vb_alloc.AllocateArray<FontVertex>(maxVertexCount);
        float cursorX = x;
        float cursorY = y;

        const char * const end = text + textLen;
        while (*text)
        {
            FontVertex *vertex = verticesStart;

            const uint32_t c = uint8_t(*text++);
            const Glyph &glyph = m_font.GetGlyph(c);

            uint16_t texturePage = glyph.m_textureIdx;
            const TextureHandle textureHandle = m_font.GetTexture(texturePage);
            const Texture *texture = m_graphics->GetTexture(textureHandle);

            const size_t textureW = texture->GetWidth();
            const size_t textureH = texture->GetHeight();

            AddFontQuad(vertex, c, glyph, cursorX, cursorY, textureW, textureH);
            while (text != end)
            {
                const uint32_t c = uint8_t(*text++);
                const Glyph &glyph = m_font.GetGlyph(c);
                if (glyph.m_textureIdx != texturePage)
                    break;
                AddFontQuad(vertex, c, glyph, cursorX, cursorY, textureW, textureH);
            }

            const size_t vertexCount = vertex - verticesStart;
            buffer->Write(0, vertexCount * sizeof(FontVertex), verticesStart);

            m_graphics->BindTexture(0, textureHandle);
            m_graphics->DrawTriangleArrays(0, vertexCount);
        }
        m_vb_alloc.Reset();
    }

    float Renderer::GetTextWidthAscii(const char *text) const
    {
        float width = 0.0f;
        while (*text)
        {
            const uint32_t c = uint8_t(*text++);
            const Glyph &glyph = m_font.GetGlyph(c);
            width += float(glyph.m_advance);
        }
        return width;
    }

    float Renderer::GetTextWidthAscii(const char *text, size_t charCount) const
    {
        float width = 0.0f;
        for (; *text && charCount; charCount--)
        {
            const uint32_t c = uint8_t(*text++);
            const Glyph &glyph = m_font.GetGlyph(c);
            width += float(glyph.m_advance);
        }
        return width;
    }

    void Renderer::SetFontScale(float scale)
    { m_fontScale = scale; }

    float Renderer::GetFontScale() const
    { return m_fontScale; }

    float Renderer::GetFontHeight() const
    { return m_font.GetHeight() * m_fontScale; }

    float Renderer::GetFontLineSpacing() const
    { return m_font.GetLineSpacing() * m_fontScale; }

} // rob
