
#ifndef H_SNEAKY_STATE_H
#define H_SNEAKY_STATE_H

#include "rob/application/GameState.h"
#include "rob/application/GameTime.h"
#include "rob/renderer/Renderer.h"
#include "rob/memory/Pool.h"
#include "rob/math/Random.h"

#include "GameData.h"
#include "GameObject.h"
#include "FadeEffect.h"
#include "SoundPlayer.h"
#include "Sensor.h"
#include "Input.h"
#include "Navigation.h"

namespace sneaky
{

    using rob::Time_t;
    using rob::GameTime;

    class DebugDraw;

    class SneakyState : public rob::GameState
    {
    public:
        SneakyState(GameData &gameData);
        ~SneakyState();

        bool Initialize() override;
        void CreateWorld();

        void Navigate(const vec2f &start, const vec2f &end);

        GameObject* CreateObject(GameObject *prevLink = nullptr);
        GameObject* CreateStaticBox(const vec2f &position, float angle, float w, float h);
        GameObject* CreateWall(const vec2f &position, float angle, float w, float h);
        GameObject* CreateHouse(const vec2f &position, float angle, float w, float h);

        GameObject* CreateCharacter(const vec2f &position, uint16_t categoryBits);
        GameObject* CreatePlayer(const vec2f &position);
        GameObject* CreateGuard(const vec2f &position);
        GameObject* CreateCake(const vec2f &position);

        void DestroyObject(GameObject *object);
        void DestroyLinkedObjects(GameObject *object);

        void DestroyObjectList(GameObject *object, GameObject *last);
        void DestroySingleObject(GameObject *object);
        void DestroyAllObjects();

        void CakeEaten();
        void PlayerCaught(const vec2f &pos);
        bool IsGameOver() const;

        void RecalcProj();
        void OnResize(int w, int h) override;

        void RealtimeUpdate(const Time_t deltaMicroseconds) override;

        void Update(const GameTime &gameTime) override;
        void RenderGameOver(const char *bigText, const char *message);
        void RenderParticleSystem(b2ParticleSystem *ps);
        void Render() override;

        void OnKeyDown(rob::Keyboard::Key key, rob::Keyboard::Scancode scancode, rob::uint32_t mods) override;
        void OnKeyUp(rob::Keyboard::Key key, rob::Keyboard::Scancode scancode, rob::uint32_t mods) override;

        void OnKeyPress(rob::Keyboard::Key key, rob::Keyboard::Scancode scancode, rob::uint32_t mods) override;
        void OnMouseDown(rob::MouseButton button, int x, int y) override;
    private:
        void AddDrawable(const Drawable *drawable);
        void DrawDrawables();

    private:
        GameData &m_gameData;
        rob::View m_view;
        b2World *m_world;
        DebugDraw *m_debugDraw;
        bool m_drawBox2D;

        bool m_inUpdate;
        bool m_gameWon;
        bool m_gameOver;

        vec2f m_mouseWorld;

        rob::Pool<GameObject> m_objectPool;
        GameObject **m_objects;
        size_t m_objectCount;

        GameObject *m_cake;

        const Drawable **m_drawables;
        size_t m_drawableCount;

        Input m_input;

        Navigation m_nav;
        bool m_debugAi;

        NavPath *m_path;
        vec2f m_pathStart, m_pathEnd;
        bool m_drawNav;

        SensorListener m_sensorListener;
        FadeEffect m_fadeEffect;

        SoundPlayer m_sounds;
        rob::Random m_random;
    };

} // sneaky

#endif // H_SNEAKY_STATE_H

