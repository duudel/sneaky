
#include "ShaderProgram.h"
#include "Shader.h"
#include "Uniform.h"
#include "Graphics.h"

#include "../Assert.h"

#include <GL/glew.h>

namespace rob
{

    ShaderProgram::ShaderProgram()
        : m_object()
        , m_linked(false)
        , m_uniformCount(0)
    {
        m_object = ::glCreateProgram();

        for (size_t i = 0; i < MAX_UNIFORMS; i++)
        {
            UniformInfo &info = m_uniforms[i];
            info.handle = InvalidHandle;
            info.location = -1;
            info.generation = -1;
        }
    }

    ShaderProgram::~ShaderProgram()
    { ::glDeleteProgram(m_object); }

    GLuint ShaderProgram::GetObject() const
    { return m_object; }

    void ShaderProgram::SetShaders(VertexShader *vertexShader, FragmentShader *fragmentShader)
    {
        if (!vertexShader->IsCompiled() || !fragmentShader->IsCompiled())
            return;

        ::glAttachShader(m_object, vertexShader->GetObject());
        ::glAttachShader(m_object, fragmentShader->GetObject());
    }

    bool ShaderProgram::Link()
    {
        ::glLinkProgram(m_object);
        GLint linked = GL_FALSE;
        ::glGetProgramiv(m_object, GL_LINK_STATUS, &linked);
        return m_linked = (linked == GL_TRUE) ? true : false;
    }

    bool ShaderProgram::IsLinked() const
    { return m_linked; }

    size_t ShaderProgram::GetLinkInfoSize() const
    {
        GLint len = 0;
        ::glGetProgramiv(m_object, GL_INFO_LOG_LENGTH, &len);
        return static_cast<size_t>(len);
    }

    void ShaderProgram::GetLinkInfo(char *buffer, size_t bufferSize) const
    { ::glGetProgramInfoLog(m_object, bufferSize, nullptr, buffer); }

    bool ShaderProgram::AddUniform(UniformHandle handle, const char *name)
    {
        for (size_t i = 0; i < m_uniformCount; i++)
        {
            UniformInfo &info = m_uniforms[i];
            // NOTE: Uniform has already been added, or the uniform was
            // deleted and this is a new uniform with the same handle.
            // Either way we just skip adding it again to the uniforms list.
            // If it really was a new uniform, then it should fail in the
            // equality test. This should be implemented as Handle = {index,generation}.
            // Now the potentially new uniform would be added and it's reference
            // count updated.
            ROB_WARN(info.handle == handle);
            if (info.handle == handle)
            {
                return false;
            }
        }
        ROB_ASSERT(m_uniformCount < MAX_UNIFORMS);

        const GLint loc = GetLocation(name);
        if (loc == -1)
            return false;

        UniformInfo &info = m_uniforms[m_uniformCount];
        info.location = loc;
        info.handle = handle;
        info.generation = -1;
        m_uniformCount++;
        return true;
    }

    void ShaderProgram::UpdateUniforms(Graphics *graphics)
    {
        for (size_t i = 0; i < m_uniformCount; i++)
        {
            UniformInfo &info = m_uniforms[i];
            const Uniform* u = graphics->GetUniform(info.handle);
            if (info.generation != u->m_generation)
            {
                u->m_upload(info.location, &u->m_value);
                info.generation = u->m_generation;
            }
        }
    }

    void ShaderProgram::RemoveUniforms(Graphics *graphics)
    {
        for (size_t i = 0; i < m_uniformCount; i++)
        {
            UniformInfo &info = m_uniforms[i];
            graphics->DecRefUniform(info.handle);
        }
        m_uniformCount = 0;
    }

    GLint ShaderProgram::GetLocation(const char *name) const
    { return ::glGetUniformLocation(m_object, name); }

} // rob
