
#include "TextureCache.h"
#include "../graphics/Graphics.h"
#include "../graphics/Texture.h"
#include "../memory/LinearAllocator.h"

#include "../Log.h"

#include <fstream>

namespace rob
{

    TextureCache::TextureCache(Graphics *graphics)
        : m_graphics(graphics)
    { }

    ROB_DEFINE_RESOURCE_CACHE_DTOR(TextureCache)

    bool TextureCache::Load(const char * const filename, TextureHandle &texture)
    {
        std::ifstream in(filename, std::ios_base::binary);
        if (!in.is_open())
        {
            log::Error("Could not open texture file ", filename);
            return InvalidHandle;
        }

        size_t width = 0;
        size_t height = 0;
        size_t st_format = 0;
        in.read(reinterpret_cast<char*>(&width), sizeof(size_t));
        in.read(reinterpret_cast<char*>(&height), sizeof(size_t));
        in.read(reinterpret_cast<char*>(&st_format), sizeof(size_t));
        if (!in)
        {
            log::Error("Invalid texture file ", filename);
            texture = InvalidHandle;
            return false;
        }
        const size_t imageSize = width * height * st_format;

        const size_t IMAGE_DATA_ALIGN = 4;
        LinearAllocator alloc(imageSize + IMAGE_DATA_ALIGN - 1);
        char *imageData = static_cast<char*>(alloc.Allocate(imageSize, IMAGE_DATA_ALIGN));

        in.read(imageData, imageSize);

        texture = m_graphics->CreateTexture();
        m_graphics->BindTexture(0, texture);
        Texture *t = m_graphics->GetTexture(texture);
        t->TexImage(width, height, static_cast<Texture::Format>(st_format), imageData);
        m_graphics->BindTexture(0, InvalidHandle);

        return true;
    }

    void TextureCache::Unload(TextureHandle texture)
    {
        m_graphics->DestroyTexture(texture);
    }

} // rob
