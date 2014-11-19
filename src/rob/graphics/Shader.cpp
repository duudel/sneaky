
#include "Shader.h"

#include <GL/glew.h>

namespace rob
{

    Shader::Shader(GLenum shaderType)
        : m_object()
        , m_compiled(false)
    { m_object = ::glCreateShader(shaderType); }

    Shader::~Shader()
    { ::glDeleteShader(m_object); }

    GLuint Shader::GetObject() const
    { return m_object; }

    void Shader::SetSource(const char * const source)
    {
        const char *src = source;
        ::glShaderSource(m_object, 1, &src, nullptr);
    }

    bool Shader::Compile()
    {
        ::glCompileShader(m_object);
        GLint compiled = GL_FALSE;
        ::glGetShaderiv(m_object, GL_COMPILE_STATUS, &compiled);
        return m_compiled = (compiled == GL_TRUE) ? true : false;
    }

    bool Shader::IsCompiled() const
    { return m_compiled; }

    size_t Shader::GetCompileInfoSize() const
    {
        GLint len = 0;
        ::glGetShaderiv(m_object, GL_INFO_LOG_LENGTH, &len);
        return static_cast<size_t>(len);
    }

    void Shader::GetCompileInfo(char *buffer, size_t bufferSize) const
    { ::glGetShaderInfoLog(m_object, bufferSize, nullptr, buffer); }


    VertexShader::VertexShader()
        : Shader(GL_VERTEX_SHADER)
    { }

    FragmentShader::FragmentShader()
        : Shader(GL_FRAGMENT_SHADER)
    { }

} // rob
