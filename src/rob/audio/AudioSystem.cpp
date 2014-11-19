
#include "AudioSystem.h"
#include "ALCheck.h"

#include "../memory/LinearAllocator.h"

#include "../Log.h"

#include <AL/al.h>
#include <SDL2/SDL_audio.h>

namespace rob
{

    struct Sound
    {
        ALuint buffer;

        Sound()
        {
            buffer = 0;
            alGenBuffers(1, &buffer);
            AL_CHECK;
        }

        ~Sound()
        {
            alDeleteBuffers(1, &buffer);
            AL_CHECK;
        }
    };

    struct Channel
    {
        ALuint source;
        Sound *playingSound;
        uint32_t startTime;

        Channel()
        {
            source = 0;
            alGenSources(1, &source);
            AL_CHECK;
            playingSound = nullptr;
            startTime = 0;
        }

        void PlaySound(Sound *sound, float volume, float x, float y, uint32_t currentTime)
        {
            playingSound = sound;
            startTime = currentTime;

            alSourcef(source, AL_PITCH, 1.0f);
            AL_CHECK;
            alSourcef(source, AL_GAIN, volume);
            AL_CHECK;
            alSource3f(source, AL_POSITION, x, y, 0.0f);
            AL_CHECK;
            alSource3f(source, AL_VELOCITY, 0.0f, 0.0f, 0.0f);
            AL_CHECK;
            alSourcei(source, AL_LOOPING, AL_FALSE);
            AL_CHECK;
            alSourcef(source, AL_MIN_GAIN, volume);
            AL_CHECK;

//            alSourceQueueBuffers(source, 1, &sound->buffer);
            alSourcei(source, AL_BUFFER, sound->buffer);
            AL_CHECK;

            alSourcePlay(source);
            AL_CHECK;
        }

        void Stop()
        { alSourceStop(source); }

        void Update()
        {
            if (playingSound == nullptr) return;

//            ALint processed = 0;
//            alGetSourceiv(source, AL_BUFFERS_PROCESSED, &processed);
            ALint state = 0;
            alGetSourcei(source, AL_SOURCE_STATE, &state);
            AL_CHECK;

//            if (processed > 0)
            if (state == AL_STOPPED)
            {
//                ALuint buffer = 0;
//                alSourceUnqueueBuffers(source, 1, &buffer);
                alSourcei(source, AL_BUFFER, 0);
                AL_CHECK;

                playingSound = nullptr;
            }
        }

        bool IsFree() const
        { return playingSound == nullptr; }

        uint32_t TimePlayed(uint32_t currentTime)
        { return currentTime - startTime; }
    };

    AudioSystem::AudioSystem(LinearAllocator& alloc)
        : m_device(nullptr)
        , m_context(nullptr)
        , m_sounds()
        , m_channelPool()
        , m_channels()
        , m_masterVolume(1.0f)
        , m_muted(false)
    {
        m_device = alcOpenDevice(nullptr);
        AL_CHECK;

        if (m_device == nullptr)
        {
            log::Error("Could not open audio device.");
            return;
        }

        m_context = alcCreateContext(m_device, nullptr);
        AL_CHECK;

        if (m_context == nullptr)
        {
            log::Error("Could not create audio context.");
            return;
        }

        alcMakeContextCurrent(m_context);
        AL_CHECK;

        const size_t allocSize = 1024;
        m_sounds.SetMemory(alloc.Allocate(allocSize, alignof(Sound)), allocSize);
        const size_t channelPoolSize = GetArraySize<Channel>(MAX_CHANNELS);
        m_channelPool.SetMemory(alloc.AllocateArray<Channel>(MAX_CHANNELS), channelPoolSize);

        for (size_t i = 0; i < MAX_CHANNELS; i++)
        {
            m_channels[i] = m_channelPool.Obtain();
        }

        SetMasterVolume(m_masterVolume);
        alListener3f(AL_POSITION, 0.0f, 0.0f, -2.0f);
        AL_CHECK;
    }

    AudioSystem::~AudioSystem()
    {
        for (size_t i = 0; i < MAX_CHANNELS; i++)
        {
            m_channelPool.Return(m_channels[i]);
        }

        if (m_context) alcDestroyContext(m_context);
        if (m_device) alcCloseDevice(m_device);
    }

    ALenum GetALFormat(const SDL_AudioSpec &spec)
    {
        int bits = SDL_AUDIO_BITSIZE(spec.format);
        int chan = spec.channels;
        if (bits == 8)
        {
            if (chan == 1) return AL_FORMAT_MONO8;
            if (chan == 2) return AL_FORMAT_STEREO8;
        }
        else if (bits == 16)
        {
            if (chan == 1) return AL_FORMAT_MONO16;
            if (chan == 2) return AL_FORMAT_STEREO16;
        }
        return 0;
    }

    SoundHandle AudioSystem::LoadSound(const char * const filename)
    {
        SDL_AudioSpec spec;
        Uint32 len;
        Uint8 *wav;

        if (SDL_LoadWAV(filename, &spec, &wav, &len) == nullptr)
        {
            log::Error("Could not load sound ", filename, ": ", SDL_GetError());
            return InvalidSound;
        }

        ALenum format = GetALFormat(spec);
        if (format == 0)
        {
            log::Error("Could not load sound ", filename, ": Unsupported data format");
            SDL_FreeWAV(wav);
            return InvalidSound;
        }

        Sound *sound = m_sounds.Obtain();

        alBufferData(sound->buffer, format, wav, len, spec.freq);
        AL_CHECK;

        SDL_FreeWAV(wav);

        return m_sounds.IndexOf(sound);
    }

    void AudioSystem::UnloadSound(SoundHandle sound)
    {
        if (sound == InvalidSound) return;

        Sound *s = m_sounds.Get(sound);
        m_sounds.Return(s);
    }

    void AudioSystem::SetMute(bool mute)
    { m_muted = mute; }

    void AudioSystem::ToggleMute()
    { SetMute(!IsMuted()); }

    bool AudioSystem::IsMuted() const
    { return m_muted; }

    void AudioSystem::SetMasterVolume(float volume)
    {
        m_masterVolume = volume;
        alListenerf(AL_GAIN, volume);
        AL_CHECK;
    }

    void AudioSystem::PlaySound(SoundHandle sound, float volume, float x, float y, Time_t currentTime)
    {
        if (IsMuted()) return;
        if (sound == InvalidSound) return;

        uint32_t timeMillis = currentTime / 1000ull;

        size_t longestPlayIndex = 0;
        uint32_t longestPlayTime = 0;

        for (size_t i = 0; i < MAX_CHANNELS; i++)
        {
            if (m_channels[i]->IsFree())
            {
                Sound *s = m_sounds.Get(sound);
                m_channels[i]->PlaySound(s, volume, x, y, timeMillis);
                return;
            }

            uint32_t playTime = m_channels[i]->TimePlayed(timeMillis);

            if (playTime > longestPlayTime)
            {
                longestPlayIndex = i;
                longestPlayTime = playTime;
            }
        }

        Sound *s = m_sounds.Get(sound);
        m_channels[longestPlayIndex]->Stop();
        m_channels[longestPlayIndex]->PlaySound(s, volume, x, y, timeMillis);
    }

    void AudioSystem::StopAllSounds()
    {
        for (size_t i = 0; i < MAX_CHANNELS; i++)
        {
            m_channels[i]->Stop();
        }
    }

    void AudioSystem::Update()
    {
        for (size_t i = 0; i < MAX_CHANNELS; i++)
        {
            m_channels[i]->Update();
        }
    }

} // rob
