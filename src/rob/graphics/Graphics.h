
#ifndef H_ROB_GRAPHICS_H
#define H_ROB_GRAPHICS_H

#include "GraphicsTypes.h"
#include "../math/Types.h"

#include "../memory/Pool.h"

namespace rob
{

    class LinearAllocator;

//    struct Vao
//    {
//        void BindFormat(fmt)
//        {
//            attribute vec4 a_tile;
//            position = a_tile.xy;
//            const float tileSize = 96.0;
//            uniform vec2 tileSetSize;
//            tileUv = (a_tile.zw * tileSize) / tileSetSize;
//
//            attribute vec4 a_sprite;
//            pos = a_sprite.xy;
//            uv = a_sprite.zw;
//
//            switch (fmt)
//            {
//            case TILE_FMT:
//                ::glEnableVertexAttribArray(0);
//                ::glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
//                break;
//            case SPRITE_FMT:
//                ::glEnableVertexAttribArray(0);
//                ::glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
//            case FONT_FMT:
//                ::glEnableVertexAttribArray(0);
//                ::glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
//            }
//        }
//        attrib[8]
//    };

    class Graphics
    {
    public:
        static const size_t MAX_TEXTURE_UNITS = 8;

    public:
        Graphics(LinearAllocator &alloc);
        ~Graphics();

        bool IsInitialized() const;
        bool HasDebugOutput() const;

        void SetViewport(int x, int y, int w, int h);
        void GetViewport(int *x, int *y, int *w, int *h) const;

        void Clear();
        void SetClearColor(float r, float g, float b);
        void SetBlendNone();
        void SetBlendAlpha();
        void SetBlendAdditive();

        void SetTexture(size_t unit, TextureHandle texture);
        void SetVertexBuffer(VertexBufferHandle buffer);
        void SetIndexBuffer(IndexBufferHandle buffer);
        void SetShaderProgram(ShaderProgramHandle program);

        void UpdateState();

        void BindTexture(size_t unit, TextureHandle texture);
        void BindVertexBuffer(VertexBufferHandle buffer);
        void BindIndexBuffer(IndexBufferHandle buffer);
        void BindShaderProgram(ShaderProgramHandle program);

        void SetUniform(UniformHandle u, int value);
        void SetUniform(UniformHandle u, float value);
        void SetUniform(UniformHandle u, const vec2f &value);
        void SetUniform(UniformHandle u, const vec4f &value);
        void SetUniform(UniformHandle u, const mat4f &value);

        void SetAttrib(size_t attr, size_t size, size_t stride, size_t offset);

        void DrawTriangleArrays(size_t first, size_t count);
        void DrawTriangleStripArrays(size_t first, size_t count);
        void DrawTriangleFanArrays(size_t first, size_t count);
        void DrawLineArrays(size_t first, size_t count);
        void DrawLineLoopArrays(size_t first, size_t count);


        TextureHandle CreateTexture();
        Texture* GetTexture(TextureHandle texture);
        void DestroyTexture(TextureHandle texture);

        VertexBufferHandle CreateVertexBuffer();
        VertexBuffer* GetVertexBuffer(VertexBufferHandle buffer);
        void DestroyVertexBuffer(VertexBufferHandle buffer);

        IndexBufferHandle CreateIndexBuffer();
        IndexBuffer* GetIndexBuffer(IndexBufferHandle buffer);
        void DestroyIndexBuffer(IndexBufferHandle buffer);

        VertexShaderHandle CreateVertexShader();
        VertexShader* GetVertexShader(VertexShaderHandle shader);
        void DestroyVertexShader(VertexShaderHandle shader);

        FragmentShaderHandle CreateFragmentShader();
        FragmentShader* GetFragmentShader(FragmentShaderHandle shader);
        void DestroyFragmentShader(FragmentShaderHandle shader);

        ShaderProgramHandle CreateShaderProgram();
        ShaderProgram* GetShaderProgram(ShaderProgramHandle program);
        void DestroyShaderProgram(ShaderProgramHandle program);

        UniformHandle CreateUniform(const char *name, UniformType type);
        UniformHandle CreateGlobalUniform(const char *name, UniformType type);
        Uniform* GetUniform(UniformHandle uniform);
        void DestroyUniform(UniformHandle uniform);
        void AddRefUniform(UniformHandle uniform);
        void DecRefUniform(UniformHandle uniform);

        void AddProgramUniform(ShaderProgramHandle program, UniformHandle uniform);

    private:
        void InitState();

    private:
        struct State
        {
            TextureHandle       texture[MAX_TEXTURE_UNITS];
            VertexBufferHandle  vertexBuffer;
            IndexBufferHandle   indexBuffer;
            ShaderProgramHandle shaderProgram;
        } m_bind, m_state;

        Pool<Texture>       m_textures;
        Pool<VertexBuffer>  m_vertexBuffers;
        Pool<IndexBuffer>   m_indexBuffers;
        Pool<VertexShader>  m_vertexShaders;
        Pool<FragmentShader>m_fragmentShaders;
        Pool<ShaderProgram> m_shaderPrograms;
        Pool<Uniform>       m_uniforms;

        bool m_initialized;
        bool m_hasDebugOutput;

        struct Viewport
        {
            int x, y;
            int w, h;
        } m_viewport;
    };

} // rob

#endif // H_ROB_GRAPHICS_H

