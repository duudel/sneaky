
#ifndef H_ROB_SHADER_H
#define H_ROB_SHADER_H

#include "GLTypes.h"
#include "../Types.h"

namespace rob
{

    class Shader
    {
    public:
        explicit Shader(GLenum shaderType);
        virtual ~Shader();

        GLuint GetObject() const;

        void SetSource(const char * const source);
        bool Compile();

        bool IsCompiled() const;

        size_t GetCompileInfoSize() const;
        void GetCompileInfo(char *buffer, size_t bufferSize) const;

    protected:
        GLuint m_object;
        bool m_compiled;
    };

    class VertexShader : private Shader
    {
    public:
        VertexShader();

        using Shader::GetObject;

        using Shader::SetSource;
        using Shader::Compile;

        using Shader::IsCompiled;

        using Shader::GetCompileInfoSize;
        using Shader::GetCompileInfo;
    };

    class FragmentShader : private Shader
    {
    public:
        FragmentShader();

        using Shader::GetObject;

        using Shader::SetSource;
        using Shader::Compile;

        using Shader::IsCompiled;

        using Shader::GetCompileInfoSize;
        using Shader::GetCompileInfo;
    };

} // rob

#endif // H_ROB_SHADER_H

