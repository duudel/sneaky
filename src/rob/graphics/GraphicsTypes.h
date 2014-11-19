
#ifndef H_ROB_GRAPHICS_TYPES_H
#define H_ROB_GRAPHICS_TYPES_H

namespace rob
{

    class Texture;
    class VertexBuffer;
    class IndexBuffer;
    class VertexShader;
    class FragmentShader;
    class ShaderProgram;
    class Uniform;

    typedef unsigned int GraphicsHandle;

    typedef GraphicsHandle TextureHandle;
    typedef GraphicsHandle VertexBufferHandle;
    typedef GraphicsHandle IndexBufferHandle;
    typedef GraphicsHandle VertexShaderHandle;
    typedef GraphicsHandle FragmentShaderHandle;
    typedef GraphicsHandle ShaderProgramHandle;
    typedef GraphicsHandle UniformHandle;

    static const GraphicsHandle InvalidHandle = ~0;

    enum class UniformType
    {
        Int, //UInt,
        Float, Vec2,
        Vec4, Mat4
    };

} // rob

#endif // H_ROB_GRAPHICS_TYPES_H

