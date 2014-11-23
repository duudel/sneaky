
#include "SneakyState.h"
#include "GameState.h"
#include "Brain.h"
#include "GuardBrain.h"

#include "B2DebugDraw.h"

#include "rob/application/Window.h"
#include "rob/renderer/Renderer.h"
#include "rob/renderer/TextLayout.h"
#include "rob/graphics/Graphics.h"
#include "rob/resource/MasterCache.h"
#include "rob/math/Projection.h"


namespace sneaky
{

    using namespace rob;

    static const float PLAY_AREA_W      = 96.0f;
    static const float PLAY_AREA_H      = PLAY_AREA_W * 0.75f;
    static const float PLAY_AREA_LEFT   = -PLAY_AREA_W / 2.0f;
    static const float PLAY_AREA_RIGHT  = -PLAY_AREA_LEFT;
    static const float PLAY_AREA_BOTTOM = -PLAY_AREA_H / 2.0f;
    static const float PLAY_AREA_TOP    = -PLAY_AREA_BOTTOM;

    float g_zoom = 1.0f;

    struct Rect
    {
        float left, right;
        float bottom, top;
        bool IsInside(const vec2f &p) const
        {
            return (p.x >= left && p.x < right) &&
                (p.y >= bottom && p.y < top);
        }
    } g_playArea = {
        PLAY_AREA_LEFT, PLAY_AREA_RIGHT,
        PLAY_AREA_BOTTOM, PLAY_AREA_TOP
    };

    static const size_t MAX_OBJECTS = 1000;

    SneakyState::SneakyState(GameData &gameData)
        : m_gameData(gameData)
        , m_view()
        , m_world(nullptr)
        , m_debugDraw(nullptr)
        , m_drawBox2D(false)
        , m_inUpdate(false)
        , m_gameWon(false)
        , m_gameOver(false)
        , m_mouseJoint(nullptr)
        , m_mouseWorld(0.0f, 0.0f)
        , m_objectPool()
        , m_objects(nullptr)
        , m_objectCount(0)
        , m_input()
        , m_nav()
        , m_path(nullptr)
        , m_pathStart(0.0f, 0.0f)
        , m_pathEnd(0.0f, 0.0f)
        , m_drawNav(false)
        , m_sensorListener()
        , m_fadeEffect(Color(0.04f, 0.01f, 0.01f))
        , m_random()
    {
        m_gameData.m_score = 0;
        m_random.Seed(GetTicks());
        GetWindow().GrabMouse();
    }

    SneakyState::~SneakyState()
    {
        DestroyAllObjects();
        GetAllocator().del_object(m_world);
        GetAllocator().del_object(m_debugDraw);
        GetAudio().StopAllSounds();
        GetAudio().Update();
        GetWindow().UnGrabMouse();

        m_nav.ReturnNavPath(m_path);
    }

    bool SneakyState::Initialize()
    {
        m_objectPool.SetMemory(GetAllocator().AllocateArray<GameObject>(MAX_OBJECTS), GetArraySize<GameObject>(MAX_OBJECTS));
        m_objects = GetAllocator().AllocateArray<GameObject*>(MAX_OBJECTS);

        m_debugDraw = GetAllocator().new_object<DebugDraw>(&GetRenderer());
        int32 flags = 0;
        flags += b2Draw::e_shapeBit;
        flags += b2Draw::e_jointBit;
        flags += b2Draw::e_aabbBit;
        flags += b2Draw::e_centerOfMassBit;
        flags += b2Draw::e_particleBit;
        m_debugDraw->SetFlags(flags);

        m_world = GetAllocator().new_object<b2World>(b2Vec2(0.0f, 0.0f));
        m_world->SetDebugDraw(m_debugDraw);
        m_world->SetContactListener(&m_sensorListener);

        m_sounds.Init(GetAudio(), GetCache());

        m_input.SetView(&m_view);

        CreateWorld();
        return true;
    }

    //float g_theta;

    void SneakyState::CreateWorld()
    {
        const int city_w = 5;
        const int city_h = 4;
        bool lots[city_w][city_h] = {0};

        const int buildings = 14;

        const float lot_w = PLAY_AREA_W / city_w;
        const float lot_h = PLAY_AREA_H / city_h;
        const float x0 = PLAY_AREA_LEFT;
        const float y0 = PLAY_AREA_BOTTOM;

        //const float theta = m_random.GetReal(-1.0f, 1.0f) * 10.0f * rob::DEG2RAD;
        //g_theta = 0.0f; //m_random.GetReal(-1.0f, 1.0f) * 10.0f * rob::DEG2RAD;

        for (int i = 0; i < buildings; i++)
        {
            int lot_x, lot_y;
            do
            {
                lot_x = m_random.GetInt(0, city_w - 1);
                lot_y = m_random.GetInt(0, city_h - 1);
            } while (lots[lot_x][lot_y]);
            lots[lot_x][lot_y] = true;

            float margin = 2.0f;
            float margin2 = margin / 2.0f;

            float x = x0 + lot_w * lot_x;
            float y = y0 + lot_h * lot_y;
            float w = m_random.GetReal(6.0f, lot_w - margin);
            float h = m_random.GetReal(6.0f, lot_h - margin);
            float w2 = w / 2.0f;
            float h2 = h / 2.0f;
            float delta_x = lot_w - w - margin;
            float delta_y = lot_h - h - margin;
            x += margin2 + m_random.GetReal(0.0f, delta_x);
            y += margin2 + m_random.GetReal(0.0f, delta_y);

            CreateStaticBox(vec2f(x + w2, y + h2), /*theta + */m_random.GetReal(-1.0f, 1.0f) * 2.5f * rob::DEG2RAD, w2, h2);
        }

        const float wallSize = 4.0f;
        const float wallSize2 = wallSize / 2.0f;
        const float wallSize3 = wallSize / 3.0f;
        CreateStaticBox(vec2f(0.0f, PLAY_AREA_BOTTOM - wallSize3), 0.0f, PLAY_AREA_W / 2.0f, wallSize2); // Floor
        CreateStaticBox(vec2f(0.0f, PLAY_AREA_TOP + wallSize3), 0.0f, PLAY_AREA_W / 2.0f, wallSize2); // Ceiling
        CreateStaticBox(vec2f(PLAY_AREA_LEFT - wallSize3, 0.0f), 0.0f, wallSize2, PLAY_AREA_H / 2.0f); // Left wall
        CreateStaticBox(vec2f(PLAY_AREA_RIGHT + wallSize3, 0.0f), 0.0f, wallSize2, PLAY_AREA_H / 2.0f); // Right wall

        m_nav.CreateNavMesh(GetAllocator(), m_world, PLAY_AREA_W / 2.0f, PLAY_AREA_H / 2.0f, 2.0f);
        m_path = m_nav.ObtainNavPath();

        m_pathStart = vec2f(-PLAY_AREA_W, -PLAY_AREA_W);
        m_pathEnd = vec2f(PLAY_AREA_W, PLAY_AREA_W);
        Navigate(m_pathStart, m_pathEnd);

        for (size_t i = 0; i < 10; i++)
        {
            CreateGuard(m_nav.GetRandomNavigableWorldPoint(m_random));
        }


        GameObject *pl = CreateObject(nullptr);

        b2BodyDef pldef;
//        pldef.type = b2_kinematicBody;
        pldef.type = b2_dynamicBody;
        pldef.userData = pl;
        pldef.position = ToB2(m_nav.GetRandomNavigableWorldPoint(m_random));
        b2Body *plBody = m_world->CreateBody(&pldef);

        pl->SetBody(plBody);
        pl->SetTexture(GetCache().GetTexture("player.tex"));

        b2CircleShape shape;
        shape.m_radius = 1.0f;
        b2FixtureDef fixDef;
        fixDef.shape = &shape;
        fixDef.density = 1.0f;
        fixDef.filter.categoryBits = PlayerBit;
        plBody->CreateFixture(&fixDef);

        PlayerBrain *brain = GetAllocator().new_object<PlayerBrain>(this, &m_input);

        pl->SetBrain(brain);

        m_cake = CreateCake(m_nav.GetRandomNavigableWorldPoint(m_random));
    }

    void SneakyState::Navigate(const vec2f &start, const vec2f &end)
    {
        m_pathStart = start;
        m_pathEnd = end;
        m_nav.Navigate(start, end, m_path);
    }

    GameObject* SneakyState::CreateObject(GameObject *prevLink /*= nullptr*/)
    {
        ROB_ASSERT(m_objectCount < MAX_OBJECTS);

        GameObject *object = m_objectPool.Obtain();
        if (prevLink) prevLink->SetNext(object);
        m_objects[m_objectCount++] = object;
        return object;
    }

    GameObject* SneakyState::CreateStaticBox(const vec2f &position, float angle, float w, float h)
    {
        GameObject *object = m_objectPool.Obtain();

        b2BodyDef def;
        def.type = b2_staticBody;
        def.position = ToB2(position);
        def.angle = angle;
        b2Body *body = m_world->CreateBody(&def);
        object->SetBody(body);
//        object->SetTexture(GetCache().GetTexture("wall.tex"));
        object->SetColor(Color(0.65f, 0.63f, 0.65f));

        b2PolygonShape shape;
        shape.SetAsBox(w, h);
        b2Fixture *fix = body->CreateFixture(&shape, 1.0f);
        b2Filter filter;
        filter.categoryBits = StaticBit;
        fix->SetFilterData(filter);

        m_objects[m_objectCount++] = object;
        return object;
    }

    GameObject* SneakyState::CreateGuard(const vec2f &position)
    {
        GameObject *guard = CreateObject(nullptr);

        b2BodyDef bodyDef;
//        bodyDef.type = b2_kinematicBody;
        bodyDef.type = b2_dynamicBody;
        bodyDef.userData = guard;
        bodyDef.position = ToB2(position);
        b2Body *body = m_world->CreateBody(&bodyDef);

        guard->SetBody(body);
        guard->SetTexture(GetCache().GetTexture("player.tex"));

        b2CircleShape shape;
        shape.m_radius = 1.0f;
        b2FixtureDef fixDef;
        fixDef.shape = &shape;
        fixDef.density = 1.0f;
        fixDef.filter.categoryBits = GuardBit;
        body->CreateFixture(&fixDef);

        Brain *brain = GetAllocator().new_object<GuardBrain>(this, &m_nav, m_random);

        guard->SetBrain(brain);
        return guard;
    }

    GameObject *SneakyState::CreateCake(const vec2f &position)
    {
        GameObject *cake = CreateObject(nullptr);

        b2BodyDef bodyDef;
        bodyDef.type = b2_kinematicBody;
        bodyDef.userData = cake;
        bodyDef.position = ToB2(position);
        b2Body *body = m_world->CreateBody(&bodyDef);

        cake->SetBody(body);
        cake->SetTexture(GetCache().GetTexture("cake.tex"));

        b2CircleShape shape;
        shape.m_radius = 1.5f;
        b2FixtureDef fixDef;
        fixDef.shape = &shape;
        fixDef.density = 1.0f;
        fixDef.filter.categoryBits = CakeBit;
        body->CreateFixture(&fixDef);

        return cake;
    }

    void SneakyState::DestroyObject(GameObject *object)
    {
        if (m_inUpdate)
            object->SetDestroyed(true);
        else
            DestroySingleObject(object);
    }

    void SneakyState::DestroyObjectList(GameObject *object, GameObject *last)
    {
        GameObject *next = object->GetNext();
        DestroyObject(object);
        if (next && next != last) DestroyObjectList(next, last);
    }

    void SneakyState::DestroySingleObject(GameObject *object)
    {
        for (size_t i = 0; i < m_objectCount; i++)
        {
            if (m_objects[i] == object)
            {
                m_objects[i] = (i < MAX_OBJECTS - 1) ?
                    m_objects[m_objectCount - 1] : nullptr;
                m_objectCount--;

                m_world->DestroyBody(object->GetBody());
                m_objectPool.Return(object);
                return;
            }
        }
        ROB_ASSERT(0);
    }

    void SneakyState::DestroyLinkedObjects(GameObject *object)
    {
        GameObject *next = object->GetNext();
        DestroyObject(object);
        if (next) DestroyObjectList(next, object);
    }

    void SneakyState::DestroyAllObjects()
    {
        for (size_t i = 0; i < m_objectCount; i++)
        {
            m_world->DestroyBody(m_objects[i]->GetBody());
            m_objectPool.Return(m_objects[i]);
            m_objects[i] = nullptr;
        }
        m_objectCount = 0;
    }

    void SneakyState::CakeEaten()
    {
        if (m_gameOver) return;

        m_sounds.PlayCakeSound(m_cake->GetPosition());

        DestroyObject(m_cake);
        m_gameOver = true;
        m_gameWon = true;
        m_input.SetEnabled(false);
    }

    float g_punchTimer = 0.0f;

    void SneakyState::PlayerCaught(const vec2f &pos)
    {
        if (g_punchTimer <= 0.0f)
        {
            m_sounds.PlayPunchSound(pos);
            g_punchTimer = m_random.GetReal(0.2f, 0.8f);
        }
        m_gameOver = true;
        m_input.SetEnabled(false);
    }

    bool SneakyState::IsGameOver() const
    { return m_gameOver; }

    void SneakyState::RecalcProj()
    {
        m_view.m_projection = Projection_Orthogonal_lh(PLAY_AREA_LEFT * g_zoom,
                                                       PLAY_AREA_RIGHT * g_zoom,
                                                       PLAY_AREA_BOTTOM * g_zoom,
                                                       PLAY_AREA_TOP * g_zoom, -1.0f, 1.0f);

//        float s, c;
//        rob::SinCos(g_theta, s, c);
//
//        mat4f viewMat( c, s, 0, 0,
//                      -s, c, 0, 0,
//                       0, 0, 1, 0,
//                       0, 0, 0, 1 );
//
//        m_view.m_projection *= viewMat;
    }

    void SneakyState::OnResize(int w, int h)
    {
        const float x_scl = w / PLAY_AREA_W;
        const float y_scl = h / PLAY_AREA_H;
        const float scale = (x_scl < y_scl) ? x_scl : y_scl;

        const int vpW = scale * PLAY_AREA_W;
        const int vpH = scale * PLAY_AREA_H;
        m_view.SetViewport((w - vpW) / 2, (h - vpH) / 2, vpW, vpH);
        RecalcProj();
    }

    void SneakyState::RealtimeUpdate(const Time_t deltaMicroseconds)
    {
        const float deltaTime = float(deltaMicroseconds) / 1e6f;
        m_fadeEffect.Update(deltaTime);
    }

    void SneakyState::Update(const GameTime &gameTime)
    {
        m_inUpdate = true;
        const float deltaTime = gameTime.GetDeltaSeconds();

        if (g_punchTimer > 0.0f)
            g_punchTimer -= deltaTime;

        m_input.UpdateMouse();
        m_sounds.UpdateTime(gameTime);

        if (IsGameOver())
        {
            m_fadeEffect.Activate(0.5f);
            m_fadeEffect.Update(deltaTime);
        }

        m_world->Step(deltaTime, 8, 8, 1);


        size_t deadCount = 0;
        GameObject *dead[MAX_OBJECTS];

        for (size_t i = 0; i < m_objectCount; i++)
        {
            if (!m_objects[i]->IsDestroyed())
            {
                m_objects[i]->Update(gameTime);
            }

            if (m_objects[i]->IsDestroyed())
                dead[deadCount++] = m_objects[i];
        }
        for (size_t i = 0; i < deadCount; i++)
        {
            DestroySingleObject(dead[i]);
        }

        m_inUpdate = false;
    }

    void SneakyState::RenderGameOver(const char *bigText, const char *message)
    {
        Renderer &renderer = GetRenderer();
        renderer.SetColor(Color(1.0f, 1.0f, 1.0f));
        renderer.BindFontShader();

        const Viewport vp = renderer.GetView().m_viewport;

        TextLayout layout(renderer, vp.w / 2.0f, vp.h / 3.0f);

        renderer.SetFontScale(4.0f);
        layout.AddTextAlignC(bigText, 0.0f);
        layout.AddLine();
        renderer.SetFontScale(2.0f);

        renderer.SetColor(Color(1.0f, 1.0f, 1.0f));
        layout.AddTextAlignC(message, 0.0f);
        layout.AddLine();
        layout.AddLine();

        renderer.SetColor(Color(1.0f, 1.0f, 1.0f));
        renderer.SetFontScale(1.0f);
        layout.AddTextAlignC("Press [space] to continue", 0.0f);
    }

    void SneakyState::RenderParticleSystem(b2ParticleSystem *ps)
    {
        Renderer &renderer = GetRenderer();
        renderer.SetModel(mat4f::Identity);
        renderer.BindColorShader();

        const b2Vec2 *positions = ps->GetPositionBuffer();
        const b2ParticleColor *colors = ps->GetColorBuffer();
        const float radius = ps->GetRadius();
        for (int i = 0; i < ps->GetParticleCount(); i++)
        {
            const b2Color col = colors[i].GetColor();
            renderer.SetColor(Color(col.r, col.g, col.b, colors[i].a / 255.0f));
            renderer.DrawFilledCircle(positions[i].x, positions[i].y, radius);
        }
    }

    void SneakyState::Render()
    {
        Renderer &renderer = GetRenderer();
        renderer.SetView(m_view);
        renderer.SetModel(mat4f::Identity);

        renderer.SetColor(Color(0.18f, 0.14f, 0.14f));
        renderer.SetColor(Color(60 / 255.0f, 71 / 255.0f, 49 / 255.0f));
        renderer.BindColorShader();
        renderer.DrawFilledRectangle(PLAY_AREA_LEFT, PLAY_AREA_BOTTOM, PLAY_AREA_RIGHT, PLAY_AREA_TOP);

//        renderer.SetModel(mat4f::Identity);
//        renderer.GetGraphics()->BindTexture(0, GetCache().GetTexture("bg.tex"));
//        renderer.GetGraphics()->SetUniform(renderer.GetGlobals().texture0, 0);
//        renderer.SetColor(Color(1.0f, 1.0f, 1.0f, 1.0f));
//        renderer.BindTextureShader();
//        renderer.DrawTexturedRectangle(PLAY_AREA_LEFT, PLAY_AREA_BOTTOM, PLAY_AREA_RIGHT, PLAY_AREA_TOP);

        int maxLayer = 0, layer;
        for (layer = 0; layer < maxLayer + 1 && layer < 2; layer++)
        {
            for (size_t i = 0; i < m_objectCount; i++)
            {
                const int l = m_objects[i]->GetLayer();
                if (l == layer)
                    m_objects[i]->Render(&renderer);
                if (l > maxLayer)
                    maxLayer = l;
            }
        }

        for (; layer < maxLayer + 1; layer++)
        {
            for (size_t i = 0; i < m_objectCount; i++)
            {
                const int l = m_objects[i]->GetLayer();
                if (l == layer)
                    m_objects[i]->Render(&renderer);
                if (l > maxLayer)
                    maxLayer = l;
            }
        }

        if (m_drawBox2D)
        {
            renderer.SetModel(mat4f::Identity);
            renderer.BindColorShader();
            m_world->DrawDebugData();
        }

        if (m_drawNav)
        {
            m_nav.RenderMesh(&renderer);
            m_nav.RenderPath(&renderer, m_path);
        }

        renderer.SetModel(mat4f::Identity);
        m_fadeEffect.Render(&renderer);

        renderer.SetView(GetDefaultView());
        renderer.BindFontShader();
        renderer.SetColor(Color::White);

        if (m_gameWon)
            RenderGameOver("Congratulations!", "You ate the cake and it was tasty.");
        else if (IsGameOver())
            RenderGameOver("Game over", "You were caught by the guards.");
    }


    void SneakyState::OnKeyDown(rob::Keyboard::Key key, rob::Keyboard::Scancode scancode, rob::uint32_t mods)
    { m_input.SetKey(scancode, true); }

    void SneakyState::OnKeyUp(rob::Keyboard::Key key, rob::Keyboard::Scancode scancode, rob::uint32_t mods)
    { m_input.SetKey(scancode, false); }

    void SneakyState::OnKeyPress(Keyboard::Key key, Keyboard::Scancode scancode, uint32_t mods)
    {
        switch (key)
        {
            case Keyboard::Key::P:
            {
                if (IsGameOver()) break;
                if (m_time.IsPaused())
                {
                    m_time.Resume();
                    m_fadeEffect.Reset();
                }
                else
                {
                    m_time.Pause();
                    m_fadeEffect.Activate(1.0f);
                }
                break;
            }
            case Keyboard::Key::Space:
            {
                if (IsGameOver())
                    ChangeState(STATE_MainMenu);
                break;
            }
            case Keyboard::Key::Escape:
            {
                ChangeState(STATE_MainMenu);
                break;
            }
            default: break;
        }

        // TODO: These are for debugging
        {
            if (key == Keyboard::Key::G)
                m_drawNav = !m_drawNav;
            if (key == Keyboard::Key::Tab)
                m_drawBox2D = !m_drawBox2D;
            if (key == Keyboard::Key::Space && !IsGameOver())
                ChangeState(STATE_Game);
            if (key == Keyboard::Key::Kp_Plus)
            {
                g_zoom = Clamp(g_zoom / 1.5f, 0.4444f, 4.0f);
                RecalcProj();
            }
            else if (key == Keyboard::Key::Kp_Minus)
            {
                g_zoom = Clamp(g_zoom * 1.5f, 0.4444f, 4.5f);
                RecalcProj();
            }
        }
    }

    vec2f ScreenToWorld(const View &view, int x, int y)
    {
        const Viewport &viewport = view.m_viewport;
        const vec4f vp = vec4f(viewport.w, -viewport.h, 1.0f, 1.0f);
        const vec4f vp_pos = vec4f(x - viewport.x, y - viewport.y, 0.0f, 1.0f);

        const vec4f ndc = (vp_pos * 2.0f - vec4f(viewport.w, viewport.h, 0.0f, 0.0f)) / vp;
        const vec4f wp = Unproject_Orthogonal_lh(view.m_projection, ndc);
        return vec2f(wp.x, wp.y);
    }

    void SneakyState::OnMouseDown(MouseButton button, int x, int y)
    {
        m_mouseWorld = ScreenToWorld(m_view, x, y);

        if (button == MouseButton::Left)
            Navigate(m_mouseWorld, m_pathEnd);
        else if (button == MouseButton::Right)
            Navigate(m_pathStart, m_mouseWorld);
    }

} // duck
