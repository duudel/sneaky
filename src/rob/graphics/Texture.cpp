
#include "Texture.h"

#include "GLCheck.h"
#include <GL/glew.h>

namespace rob
{

    Texture::Texture()
        : m_object()
        , m_width(0)
        , m_height(0)
        , m_format(FMT_RGB)
    {
        ::glGenTextures(1, &m_object);
        GL_CHECK;
    }

    Texture::~Texture()
    {
        ::glDeleteTextures(1, &m_object);
        GL_CHECK;
    }

    GLuint Texture::GetObject() const
    { return m_object; }

    static const GLenum gl_formats[] = {
        [0] = GL_RGB,
        [1] = GL_RGB,
        [2] = GL_RGB,
        [Texture::FMT_RGB] = GL_RGB,
        [Texture::FMT_RGBA] = GL_RGBA,
    };

    void Texture::TexImage(size_t w, size_t h, Format fmt, const void * const data)
    {
        m_width = w;
        m_height = h;
        m_format = fmt;

        const GLint internalFmt = static_cast<GLint>(fmt);
        const GLenum format = gl_formats[fmt];
        ::glTexImage2D(GL_TEXTURE_2D, 0, internalFmt, w, h, 0, format, GL_UNSIGNED_BYTE, data);
        GL_CHECK;
        ::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);//GL_NEAREST);
        GL_CHECK;
        ::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        GL_CHECK;
    }

    size_t Texture::GetWidth() const
    { return m_width; }

    size_t Texture::GetHeight() const
    { return m_height; }

} // rob
