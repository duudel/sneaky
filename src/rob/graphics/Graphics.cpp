
#include "Graphics.h"

#include "Texture.h"
#include "VertexBuffer.h"
#include "IndexBuffer.h"
#include "Shader.h"
#include "ShaderProgram.h"
#include "Uniform.h"

#include "../memory/LinearAllocator.h"

#include "../Log.h"
#include "../String.h"

#include "GLCheck.h"
#include <GL/glew.h>

namespace rob
{

    void __stdcall gl_debug_callback(GLenum source, GLenum type, GLuint id, GLenum severity,
                                  GLsizei length, const GLchar *message, GLvoid *userParam)
    {
        const char *src = "undefined";
        switch (source)
        {
        case GL_DEBUG_SOURCE_API:             src = "API"; break;
        case GL_DEBUG_SOURCE_WINDOW_SYSTEM:   src = "Window system"; break;
        case GL_DEBUG_SOURCE_SHADER_COMPILER: src = "Shader compiler"; break;
        case GL_DEBUG_SOURCE_THIRD_PARTY:     src = "Third party"; break;
        case GL_DEBUG_SOURCE_APPLICATION:     src = "Application"; break;
        case GL_DEBUG_SOURCE_OTHER:           src = "Other"; break;
        }

        const char *tp = "";
        switch (type)
        {
        case GL_DEBUG_TYPE_ERROR_ARB:               tp = "error"; break;
        case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR_ARB: tp = "deprecated"; break;
        case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR_ARB:  tp = "undefined"; break;
        case GL_DEBUG_TYPE_PORTABILITY_ARB:         tp = "portability"; break;
        case GL_DEBUG_TYPE_PERFORMANCE_ARB:         tp = "performance"; break;
        case GL_DEBUG_TYPE_OTHER_ARB:               tp = "other"; break;
        }

        const char *sev = "undefined";
        switch (severity)
        {
        case GL_DEBUG_SEVERITY_HIGH_ARB:   sev = "high";   break;
        case GL_DEBUG_SEVERITY_MEDIUM_ARB: sev = "medium"; break;
        case GL_DEBUG_SEVERITY_LOW_ARB:    sev = "low"; break;
        }

        log::Debug(src, " ", tp, "(", sev, "): ", message, ", ", id);
    }

    Graphics::Graphics(LinearAllocator &alloc)
        : m_bind()
        , m_state()
        , m_textures()
        , m_vertexBuffers()
        , m_indexBuffers()
        , m_vertexShaders()
        , m_fragmentShaders()
        , m_shaderPrograms()
        , m_uniforms()
        , m_initialized(false)
        , m_hasDebugOutput(false)
    {
        SetViewport(0, 0, 0, 0);

        InitState();

    #ifdef ROB_DEBUG
        if (::glewIsSupported("GL_ARB_debug_output"))
        {
            ::glDebugMessageCallback(&gl_debug_callback, nullptr);
            ::glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
            m_hasDebugOutput = true;
        }
    #endif // ROB_DEBUG

        SetBlendAlpha();

        const size_t blockSize = 1024;
        m_textures.SetMemory(alloc.Allocate(blockSize), blockSize);
        m_vertexBuffers.SetMemory(alloc.Allocate(blockSize), blockSize);
        m_indexBuffers.SetMemory(alloc.Allocate(blockSize), blockSize);
        m_vertexShaders.SetMemory(alloc.Allocate(blockSize), blockSize);
        m_fragmentShaders.SetMemory(alloc.Allocate(blockSize), blockSize);
        m_shaderPrograms.SetMemory(alloc.Allocate(blockSize), blockSize);
        m_uniforms.SetMemory(alloc.Allocate(blockSize), blockSize);

        m_initialized = true;
    }

    void Graphics::InitState()
    {
        for (size_t i = 0; i < MAX_TEXTURE_UNITS; i++)
            m_bind.texture[i] = InvalidHandle;
        m_bind.vertexBuffer = InvalidHandle;
        m_bind.indexBuffer = InvalidHandle;
        m_bind.shaderProgram = InvalidHandle;
        m_state = m_bind;
    }

    Graphics::~Graphics()
    {
        ROB_WARN(m_textures.GetAllocationCount() > 0);
        ROB_WARN(m_vertexBuffers.GetAllocationCount() > 0);
        ROB_WARN(m_indexBuffers.GetAllocationCount() > 0);
        ROB_WARN(m_vertexShaders.GetAllocationCount() > 0);
        ROB_WARN(m_fragmentShaders.GetAllocationCount() > 0);
        ROB_WARN(m_shaderPrograms.GetAllocationCount() > 0);
        ROB_WARN(m_uniforms.GetAllocationCount() > 0);
    }

    bool Graphics::IsInitialized() const
    { return m_initialized; }

    bool Graphics::HasDebugOutput() const
    { return m_hasDebugOutput; }

    void Graphics::SetViewport(int x, int y, int w, int h)
    {
        ::glViewport(x, y, w, h);
        m_viewport.x = x;
        m_viewport.y = y;
        m_viewport.w = w;
        m_viewport.h = h;
    }

    void Graphics::GetViewport(int *x, int *y, int *w, int *h) const
    {
        *x = m_viewport.x;
        *y = m_viewport.y;
        *w = m_viewport.w;
        *h = m_viewport.h;
    }

    void Graphics::Clear()
    { ::glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); }

    void Graphics::SetClearColor(float r, float g, float b)
    { ::glClearColor(r, g, b, 1.0f); }

    void Graphics::SetBlendNone()
    {
        ::glDisable(GL_BLEND);
    }

    void Graphics::SetBlendAlpha()
    {
        ::glEnable(GL_BLEND);
        ::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    void Graphics::SetBlendAdditive()
    {
        ::glEnable(GL_BLEND);
        ::glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    }

    void Graphics::SetTexture(size_t unit, TextureHandle texture)
    {
        ROB_ASSERT(unit < MAX_TEXTURE_UNITS);
        m_state.texture[unit] = texture;
    }

    void Graphics::SetVertexBuffer(VertexBufferHandle buffer)
    { m_state.vertexBuffer = buffer; }

    void Graphics::SetIndexBuffer(IndexBufferHandle buffer)
    { m_state.indexBuffer = buffer; }

    void Graphics::SetShaderProgram(ShaderProgramHandle program)
    { m_state.shaderProgram = program; }

    void Graphics::UpdateState()
    {
        for (size_t i = 0; i < MAX_TEXTURE_UNITS; i++)
        {
            BindTexture(i, m_state.texture[i]);
        }
        BindVertexBuffer(m_state.vertexBuffer);
        BindIndexBuffer(m_state.indexBuffer);
    }

    void Graphics::BindTexture(size_t unit, TextureHandle texture)
    {
        ROB_ASSERT(unit < MAX_TEXTURE_UNITS);

        if (m_bind.texture[unit] == texture)
            return;

        m_bind.texture[unit] = texture;
        ::glActiveTexture(GL_TEXTURE0 + unit);
        GL_CHECK;
        if (texture == InvalidHandle)
        {
            ::glBindTexture(GL_TEXTURE_2D, 0);
            GL_CHECK;
        }
        else
        {
            Texture *tex = m_textures.Get(texture);
            ::glBindTexture(GL_TEXTURE_2D, tex->GetObject());
            GL_CHECK;
        }
    }

    void Graphics::BindVertexBuffer(VertexBufferHandle buffer)
    {
        if (m_bind.vertexBuffer == buffer)
            return;

        m_bind.vertexBuffer = buffer;
        if (buffer == InvalidHandle)
        {
            ::glBindBuffer(GL_ARRAY_BUFFER, 0);
            GL_CHECK;
        }
        else
        {
            VertexBuffer *vb = m_vertexBuffers.Get(buffer);
            ::glBindBuffer(GL_ARRAY_BUFFER, vb->GetObject());
            GL_CHECK;
        }
    }

    void Graphics::BindIndexBuffer(IndexBufferHandle buffer)
    {
        if (m_bind.indexBuffer == buffer)
            return;

        m_bind.indexBuffer = buffer;
        if (buffer == InvalidHandle)
        {
            ::glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
            GL_CHECK;
        }
        else
        {
            IndexBuffer *ib = m_indexBuffers.Get(buffer);
            ::glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ib->GetObject());
            GL_CHECK;
        }
    }

    void Graphics::BindShaderProgram(ShaderProgramHandle program)
    {
        ShaderProgram *p = (program != InvalidHandle) ?
            m_shaderPrograms.Get(program) : nullptr;

        if (m_bind.shaderProgram != program)
        {
            m_bind.shaderProgram = program;
            if (program == InvalidHandle)
            {
                ::glUseProgram(0);
                GL_CHECK;
            }
            else
            {
                ::glUseProgram(p->GetObject());
                GL_CHECK;
            }
        }
        if (p) p->UpdateUniforms(this);
    }

    void Graphics::SetUniform(UniformHandle u, int value)
    {
        Uniform *uniform = GetUniform(u);
        ROB_ASSERT(uniform->m_type == UniformType::Int);
        uniform->SetValue(value);
        if (m_bind.shaderProgram != InvalidHandle)
        {
            ShaderProgram *p = m_shaderPrograms.Get(m_bind.shaderProgram);
            p->UpdateUniforms(this);
        }
    }

    void Graphics::SetUniform(UniformHandle u, float value)
    {
        Uniform *uniform = GetUniform(u);
        ROB_ASSERT(uniform->m_type == UniformType::Float);
        uniform->SetValue(value);
        if (m_bind.shaderProgram != InvalidHandle)
        {
            ShaderProgram *p = m_shaderPrograms.Get(m_bind.shaderProgram);
            p->UpdateUniforms(this);
        }
    }

    void Graphics::SetUniform(UniformHandle u, const vec2f &value)
    {
        Uniform *uniform = GetUniform(u);
        ROB_ASSERT(uniform->m_type == UniformType::Vec2);
        uniform->SetValue(value);
        if (m_bind.shaderProgram != InvalidHandle)
        {
            ShaderProgram *p = m_shaderPrograms.Get(m_bind.shaderProgram);
            p->UpdateUniforms(this);
        }
    }

    void Graphics::SetUniform(UniformHandle u, const vec4f &value)
    {
        Uniform *uniform = GetUniform(u);
        ROB_ASSERT(uniform->m_type == UniformType::Vec4);
        uniform->SetValue(value);
        if (m_bind.shaderProgram != InvalidHandle)
        {
            ShaderProgram *p = m_shaderPrograms.Get(m_bind.shaderProgram);
            p->UpdateUniforms(this);
        }
    }

    void Graphics::SetUniform(UniformHandle u, const mat4f &value)
    {
        Uniform *uniform = GetUniform(u);
        ROB_ASSERT(uniform->m_type == UniformType::Mat4);
        uniform->SetValue(value);
        if (m_bind.shaderProgram != InvalidHandle)
        {
            ShaderProgram *p = m_shaderPrograms.Get(m_bind.shaderProgram);
            p->UpdateUniforms(this);
        }
    }


    void Graphics::SetAttrib(size_t attr, size_t size, size_t stride, size_t offset)
    {
        ROB_ASSERT(attr < 8);
        ::glEnableVertexAttribArray(attr);
        GL_CHECK;
        ::glVertexAttribPointer(attr, size, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void*>(offset));
        GL_CHECK;
    }


    void Graphics::DrawTriangleArrays(size_t first, size_t count)
    {
        ::glDrawArrays(GL_TRIANGLES, first, count);
        GL_CHECK;
    }

    void Graphics::DrawTriangleStripArrays(size_t first, size_t count)
    {
        ::glDrawArrays(GL_TRIANGLE_STRIP, first, count);
        GL_CHECK;
    }

    void Graphics::DrawTriangleFanArrays(size_t first, size_t count)
    {
        ::glDrawArrays(GL_TRIANGLE_FAN, first, count);
        GL_CHECK;
    }

    void Graphics::DrawLineArrays(size_t first, size_t count)
    {
        ::glDrawArrays(GL_LINES, first, count);
        GL_CHECK;
    }

    void Graphics::DrawLineLoopArrays(size_t first, size_t count)
    {
        ::glDrawArrays(GL_LINE_LOOP, first, count);
        GL_CHECK;
    }

    // Textures

    TextureHandle Graphics::CreateTexture()
    {
        Texture *texture = m_textures.Obtain();
        return m_textures.IndexOf(texture);
    }

    Texture* Graphics::GetTexture(TextureHandle texture)
    { return m_textures.Get(texture); }

    void Graphics::DestroyTexture(TextureHandle texture)
    { m_textures.Return(GetTexture(texture)); }

    // Vertex and index buffers

    VertexBufferHandle Graphics::CreateVertexBuffer()
    {
        VertexBuffer *buffer = m_vertexBuffers.Obtain();
        return m_vertexBuffers.IndexOf(buffer);
    }

    VertexBuffer* Graphics::GetVertexBuffer(VertexBufferHandle buffer)
    { return m_vertexBuffers.Get(buffer); }

    void Graphics::DestroyVertexBuffer(VertexBufferHandle buffer)
    { m_vertexBuffers.Return(GetVertexBuffer(buffer)); }

    IndexBufferHandle Graphics::CreateIndexBuffer()
    {
        IndexBuffer *buffer = m_indexBuffers.Obtain();
        return m_indexBuffers.IndexOf(buffer);
    }

    IndexBuffer* Graphics::GetIndexBuffer(IndexBufferHandle buffer)
    { return m_indexBuffers.Get(buffer); }

    void Graphics::DestroyIndexBuffer(IndexBufferHandle buffer)
    { m_indexBuffers.Return(GetIndexBuffer(buffer)); }

    // Shaders

    VertexShaderHandle Graphics::CreateVertexShader()
    {
        VertexShader *shader = m_vertexShaders.Obtain();
        return m_vertexShaders.IndexOf(shader);
    }

    VertexShader* Graphics::GetVertexShader(VertexShaderHandle shader)
    { return m_vertexShaders.Get(shader); }

    void Graphics::DestroyVertexShader(VertexShaderHandle shader)
    { m_vertexShaders.Return(GetVertexShader(shader)); }

    FragmentShaderHandle Graphics::CreateFragmentShader()
    {
        FragmentShader *shader = m_fragmentShaders.Obtain();
        return m_fragmentShaders.IndexOf(shader);
    }

    FragmentShader* Graphics::GetFragmentShader(FragmentShaderHandle shader)
    { return m_fragmentShaders.Get(shader); }

    void Graphics::DestroyFragmentShader(FragmentShaderHandle shader)
    { m_fragmentShaders.Return(GetFragmentShader(shader)); }

    ShaderProgramHandle Graphics::CreateShaderProgram()
    {
        ShaderProgram *program = m_shaderPrograms.Obtain();
        return m_shaderPrograms.IndexOf(program);
    }

    ShaderProgram* Graphics::GetShaderProgram(ShaderProgramHandle program)
    { return m_shaderPrograms.Get(program); }

    void Graphics::DestroyShaderProgram(ShaderProgramHandle program)
    {
        ShaderProgram *p = GetShaderProgram(program);
        p->RemoveUniforms(this);
        m_shaderPrograms.Return(p);
    }

    // Uniforms

    UniformHandle Graphics::CreateUniform(const char *name, UniformType type)
    {
        Uniform *uniform = m_uniforms.Obtain();
        uniform->m_type         = type;
        uniform->m_generation   = 0;
        uniform->m_references   = 0;
        uniform->m_upload       = Uniform::GetUploadFuncFromType(type);
        CopyStringN(uniform->m_name, name);
        return m_uniforms.IndexOf(uniform);
    }

    UniformHandle Graphics::CreateGlobalUniform(const char *name, UniformType type)
    {
        UniformHandle uniform = CreateUniform(name, type);
        AddRefUniform(uniform);
        return uniform;
    }

    Uniform* Graphics::GetUniform(UniformHandle uniform)
    { return m_uniforms.Get(uniform); }

    void Graphics::DestroyUniform(UniformHandle uniform)
    {
        Uniform *u = GetUniform(uniform);
        ROB_WARN(u->m_references > 0);
        m_uniforms.Return(u);
    }

    void Graphics::AddRefUniform(UniformHandle uniform)
    {
        Uniform *u = GetUniform(uniform);
        u->m_references++;
    }

    void Graphics::DecRefUniform(UniformHandle uniform)
    {
        Uniform *u = GetUniform(uniform);
        ROB_ASSERT(u->m_references > 0);
        if (--u->m_references == 0)
            DestroyUniform(uniform);
    }

    void Graphics::AddProgramUniform(ShaderProgramHandle program, UniformHandle uniform)
    {
        ShaderProgram *p = GetShaderProgram(program);
        Uniform *u = GetUniform(uniform);
        if (p->AddUniform(uniform, u->m_name))
        {
            u->m_references++;
        }
    }

} // rob
