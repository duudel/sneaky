
#ifndef H_DUCK_SOUND_PLAYER_H
#define H_DUCK_SOUND_PLAYER_H

#include "rob/audio/AudioSystem.h"
#include "rob/resource/MasterCache.h"

namespace sneaky
{

    using namespace rob;

    constexpr float PositionScale = 0.125f;

    class SoundPlayer
    {
    public:
        SoundPlayer()
            : m_audio(nullptr)
//            , m_cache(nullptr)
            , m_currentTime(0)
            , m_burning(InvalidSound)
            , m_water(InvalidSound)
        { }

        void Init(AudioSystem &audio, MasterCache &cache)
        {
            m_audio = &audio;
//            m_burning = cache.GetSound("Explosion32.wav");
//            m_burning = cache.GetSound("flame3.wav");
            m_burning = cache.GetSound("exp_flame2.wav");
//            m_water = cache.GetSound("Randomize55.wav");
            m_water = cache.GetSound("splash.wav");
            m_wash = cache.GetSound("wash.wav");
            m_saved = cache.GetSound("Powerup122.wav");
            m_bird = cache.GetSound("kraah3.wav");
//            m_dying = cache.GetSound("Powerup.wav");
            m_dying = cache.GetSound("kraah2.wav");
            m_score = cache.GetSound("Pickup_Coin.wav");
        }

        void UpdateTime(const GameTime &gameTime)
        {
            m_currentTime = gameTime.GetTotalMicroseconds();
        }

        void PlayBurningSound(const vec2f &pos)
        {
            PlaySound(m_burning, 1.0f, pos);
        }

        void PlayWaterSound(const vec2f &pos)
        {
            PlaySound(m_water, 0.1f, pos);
        }

        void PlayWashSound(const vec2f &pos)
        {
            PlaySound(m_wash, 0.35f, pos);
        }

        void PlaySavedSound(const vec2f &pos)
        {
            PlaySound(m_saved, 0.15f, pos);
        }

        void PlayBirdSound(const vec2f &pos)
        {
            PlaySound(m_bird, 0.15f, pos);
        }

        void PlayDyingSound(const vec2f &pos)
        {
            PlaySound(m_dying, 0.5f, pos);
        }

        void PlayScoreSound(const vec2f &pos)
        {
            PlaySound(m_score, 0.15f, pos);
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
//        MasterCache *m_cache;
        Time_t m_currentTime;

        SoundHandle m_burning;
        SoundHandle m_water;
        SoundHandle m_wash;
        SoundHandle m_saved;
        SoundHandle m_bird;
        SoundHandle m_dying;
        SoundHandle m_score;
    };

} // sneaky

#endif // H_DUCK_SOUND_PLAYER_H
