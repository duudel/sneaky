
#ifndef H_DUCK_SOUND_PLAYER_H
#define H_DUCK_SOUND_PLAYER_H

#include "rob/audio/AudioSystem.h"
#include "rob/resource/MasterCache.h"

namespace sneaky
{

    using namespace rob;

    constexpr float PositionScale = 0.025f;

    class SoundPlayer
    {
    public:
        SoundPlayer()
            : m_audio(nullptr)
            , m_cake(InvalidSound)
            , m_punch(InvalidSound)
        { }

        void Init(AudioSystem &audio, MasterCache &cache)
        {
            m_audio = &audio;

            m_cake = cache.GetSound("Powerup.wav");
            m_punch = cache.GetSound("Hit_Hurt34.wav");
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

    private:
        void PlaySound(SoundHandle sound, float volume, const vec2f &pos)
        {
            float x = pos.x * PositionScale;
            float y = pos.y * PositionScale;
            m_audio->PlaySound(sound, volume, x, y, m_currentTime);
        }

    private:
        AudioSystem *m_audio;
        Time_t m_currentTime;

        SoundHandle m_cake;
        SoundHandle m_punch;
    };

} // sneaky

#endif // H_DUCK_SOUND_PLAYER_H
