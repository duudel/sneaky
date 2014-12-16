
#ifndef H_DUCK_SOUND_PLAYER_H
#define H_DUCK_SOUND_PLAYER_H

#include "rob/audio/AudioSystem.h"
#include "rob/resource/MasterCache.h"

namespace sneaky
{

    using namespace rob;

    constexpr float PositionScale = 0.025f;

    struct SoundSet
    {
        static const size_t MAX_SOUNDS = 4;
        SoundHandle m_sounds[MAX_SOUNDS];
        size_t m_soundCount;

        SoundSet() : m_soundCount(0) { }

        void AddSound(SoundHandle sound)
        {
            ROB_ASSERT(m_soundCount < MAX_SOUNDS);
            m_sounds[m_soundCount++] = sound;
        }

        SoundHandle GetRandom(Random &rand) const
        {
            if (m_soundCount == 0) return InvalidSound;
            return m_sounds[rand.GetInt(0, m_soundCount - 1)];
        }
    };

    class SoundPlayer
    {
    public:
        SoundPlayer()
            : m_audio(nullptr)
            , m_cake(InvalidSound)
            , m_punch(InvalidSound)
            , m_footsteps()
        { }

        void Init(AudioSystem &audio, MasterCache &cache)
        {
            m_audio = &audio;

            m_cake = cache.GetSound("Powerup.wav");
            m_punch = cache.GetSound("Hit_Hurt34.wav");
            m_footsteps.AddSound(cache.GetSound("footstep01.wav"));
            m_footsteps.AddSound(cache.GetSound("footstep02.wav"));
            m_footsteps.AddSound(cache.GetSound("footstep03.wav"));
        }

        void UpdateTime(const GameTime &gameTime)
        {
            m_currentTime = gameTime.GetTotalMicroseconds();
        }

        void PlayCakeSound(const vec2f &pos)
        {
            PlaySound(m_cake, 0.5f, pos);
        }

        void PlayPunchSound(const vec2f &pos)
        {
            PlaySound(m_punch, 0.5f, pos);
        }

        void PlayFootstep(const vec2f &pos, float volume, Random &rand)
        {
            PlaySound(m_footsteps, volume, pos, rand);
        }

    private:
        void PlaySound(SoundHandle sound, float volume, const vec2f &pos)
        {
            float x = pos.x * PositionScale;
            float y = pos.y * PositionScale;
            m_audio->PlaySound(sound, volume, x, y, m_currentTime);
        }

        void PlaySound(const SoundSet &soundSet, float volume, const vec2f &pos, Random &rand)
        {
            SoundHandle sound = soundSet.GetRandom(rand);
            PlaySound(sound, volume, pos);
        }

    private:
        AudioSystem *m_audio;
        Time_t m_currentTime;

        SoundHandle m_cake;
        SoundHandle m_punch;
        SoundSet m_footsteps;
    };

} // sneaky

#endif // H_DUCK_SOUND_PLAYER_H
