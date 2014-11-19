
#ifndef H_ROB_SOUND_CACHE_H
#define H_ROB_SOUND_CACHE_H

#include "ResourceCache.h"
#include "../audio/AudioSystem.h"

namespace rob
{

    class SoundCache : private ResourceCache<SoundHandle>
    {
    public:
        explicit SoundCache(AudioSystem *audio);
        SoundCache(const SoundCache&) = delete;
        SoundCache& operator = (const SoundCache&) = delete;
        ~SoundCache();

        using ResourceCache::Get;
        using ResourceCache::UnloadAll;

        bool Load(const char * const filename, SoundHandle &sound);
        void Unload(SoundHandle sound);

    private:
        AudioSystem *m_audio;
    };

} // rob

#endif // H_ROB_SOUND_CACHE_H
