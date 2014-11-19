
#ifndef H_ROB_TEXTURE_CACHE_H
#define H_ROB_TEXTURE_CACHE_H

#include "ResourceCache.h"
#include "../graphics/GraphicsTypes.h"

namespace rob
{

    class Graphics;

    class TextureCache : private ResourceCache<TextureHandle>
    {
    public:
        explicit TextureCache(Graphics *graphics);
        TextureCache(const TextureCache&) = delete;
        TextureCache& operator = (const TextureCache&) = delete;
        ~TextureCache();

        using ResourceCache::Get;
        using ResourceCache::UnloadAll;

        bool Load(const char * const filename, TextureHandle &texture);
        void Unload(TextureHandle texture);

    private:
        Graphics *m_graphics;
    };

} // rob

#endif // H_ROB_TEXTURE_CACHE_H

