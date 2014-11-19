
#ifndef H_ROB_SHADER_PROGRAM_H
#define H_ROB_SHADER_PROGRAM_H

#include "GLTypes.h"
#include "GraphicsTypes.h"
#include "../Types.h"

namespace rob
{

    class Graphics;
    class VertexShader;
    class FragmentShader;

    class ShaderProgram
    {
    public:
        ShaderProgram();
        ~ShaderProgram();

        GLuint GetObject() const;

        void SetShaders(VertexShader *vertexShader, FragmentShader *fragmentShader);
        bool Link();

        bool IsLinked() const;

        size_t GetLinkInfoSize() const;
        void GetLinkInfo(char *buffer, size_t bufferSize) const;

        /// Adds the uniform \c handle to the shader program's uniforms.
        /// Returns true, if the uniform was added, false otherwise.
        bool AddUniform(UniformHandle handle, const char *name);

        /// Updates uniforms that this program references.
        /// The program must be bind before calling.
        void UpdateUniforms(Graphics *graphics);

        /// Removes the uniforms this shader has decreasing the reference
        /// count for those uniforms (and thus possibly destroying them).
        void RemoveUniforms(Graphics *graphics);

    private:
        GLint GetLocation(const char *name) const;

    private:
        GLuint m_object;
        bool m_linked;

        static const size_t MAX_UNIFORMS = 8;

        struct UniformInfo
        {
            UniformHandle   handle;
            GLint           location;
            uint32_t        generation;
        } m_uniforms[MAX_UNIFORMS];
        size_t m_uniformCount;
    };

} // rob

#endif // H_ROB_SHADER_PROGRAM_H

