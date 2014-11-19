
#ifndef H_ROB_AUDIO_SYSTEM_H
#define H_ROB_AUDIO_SYSTEM_H

#include "../memory/Pool.h"

#include <AL/alc.h>

namespace rob
{

    class LinearAllocator;

    struct Sound;
    struct Channel;

    typedef unsigned int SoundHandle;
    static const SoundHandle InvalidSound = ~0;

    static const size_t MAX_CHANNELS = 16;

    class AudioSystem
    {
    public:
        AudioSystem(LinearAllocator &alloc);
        ~AudioSystem();

        SoundHandle LoadSound(const char * const filename);
        void UnloadSound(SoundHandle sound);

        void SetMasterVolume(float volume);

        void SetMute(bool mute);
        void ToggleMute();
        bool IsMuted() const;

        void PlaySound(SoundHandle sound, float volume, float x, float y, Time_t currentTime);

        void StopAllSounds();

        void Update();

    private:
        ALCdevice *m_device;
        ALCcontext *m_context;
        Pool<Sound> m_sounds;
        Pool<Channel> m_channelPool;
        Channel *m_channels[MAX_CHANNELS];
        float m_masterVolume;
        bool m_muted;
    };

} // rob

#endif // H_ROB_AUDIO_SYSTEM_H
