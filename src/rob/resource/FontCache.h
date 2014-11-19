
#ifndef H_ROB_FONT_CACHE_H
#define H_ROB_FONT_CACHE_H

#include "ResourceCache.h"
#include "../renderer/Font.h"

namespace rob
{

    class Graphics;
    class MasterCache;

    class FontCache : private ResourceCache<Font>
    {
    public:
        explicit FontCache(Graphics *graphics, MasterCache *m_cache);
        FontCache(const FontCache&) = delete;
        FontCache& operator = (const FontCache&) = delete;
        ~FontCache();

        using ResourceCache::Get;
        using ResourceCache::UnloadAll;

        bool Load(const char * const filename, Font &font);
        void Unload(Font font);

    private:
        Graphics *m_graphics;
        MasterCache *m_cache;
    };

} // rob

#endif // H_ROB_FONT_CACHE_H

