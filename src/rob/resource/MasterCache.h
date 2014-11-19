
#ifndef H_ROB_MASTER_CACHE_H
#define H_ROB_MASTER_CACHE_H

#include "ResourceID.h"
#include "TextureCache.h"
#include "SoundCache.h"
#include "FontCache.h"

#include <unordered_map>

namespace rob
{

    class Graphics;
    class AudioSystem;

    class MasterCache
    {
    public:
        MasterCache(Graphics *graphics, AudioSystem *audio, LinearAllocator &alloc);

        TextureHandle GetTexture(ResourceID id);
        SoundHandle GetSound(ResourceID id);
        Font GetFont(ResourceID id);

    private:
        TextureCache m_textures;
        SoundCache m_sounds;
        FontCache m_fonts;

        struct Resource
        {
            char m_filepath[260];
        };
        std::unordered_map<uint32_t, Resource> m_resources;

    private:
        void Scan(const char * const dir);
        bool FindResource(ResourceID id, const Resource **resource) const;
        void ReportInvalidResource(ResourceID id) const;
    };

} // rob

#endif // H_ROB_MASTER_CACHE_H

