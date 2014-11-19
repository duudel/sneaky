
#include "SoundCache.h"

#include "../Log.h"

namespace rob
{

    SoundCache::SoundCache(AudioSystem *audio)
        : m_audio(audio)
    { }

    ROB_DEFINE_RESOURCE_CACHE_DTOR(SoundCache)

    bool SoundCache::Load(const char * const filename, SoundHandle &sound)
    {
        return (sound = m_audio->LoadSound(filename)) != InvalidSound;
    }

    void SoundCache::Unload(SoundHandle sound)
    {
        m_audio->UnloadSound(sound);
    }

} // rob
