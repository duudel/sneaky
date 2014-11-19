
#ifndef H_ROB_TEXTURE_H
#define H_ROB_TEXTURE_H

#include "GLTypes.h"
#include "../Types.h"

namespace rob
{

    class Texture
    {
    public:
        enum Format
        {
            FMT_RGB = 3,
            FMT_RGBA = 4
        };
    public:
        Texture();
        ~Texture();

        GLuint GetObject() const;

        /// Sets the texture image using the data in internal format fmt.
        /// If data is null, no data is written to the image, but the storage for
        /// the image is reserved.
        /// \pre This texture must be bind to the graphics context before calling this method.
        void TexImage(size_t w, size_t h, Format fmt, const void * const data);

        size_t GetWidth() const;
        size_t GetHeight() const;

    private:
        GLuint m_object;
        size_t m_width;
        size_t m_height;
        Format m_format;
    };

} // rob

#endif // H_ROB_TEXTURE_H

