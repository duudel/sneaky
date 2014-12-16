
#include "GuardBrain.h"
#include "Physics.h"
#include "GameObject.h"
#include "Navigation.h"
#include "SneakyState.h"

#include "rob/application/GameTime.h"

namespace sneaky
{

    GuardBrain::GuardBrain(SneakyState *game, Navigation *nav, rob::Random &rand)
        : Brain()
        , m_game(game)
        , m_nav(nav)
        , m_path(nullptr)
        , m_pathPos(0)
        , m_rand(rand)
        , m_rightSensor()
        , m_leftSensor()
        , m_visionSensor()
        , m_stuckMeter(0.0f)
        , m_prevPosition(0.0f, 0.0f)
        , m_state(State::Watch)
        , m_stateTimer()
        , m_watchTimer(0.0f)
        , m_lastKnownPlayerPos()
        , m_soundHeard(false)
        , m_soundSource()
    {
        m_path = m_nav->ObtainNavPath();
    }

    GuardBrain::~GuardBrain()
    {
        m_nav->ReturnNavPath(m_path);
    }

    void GuardBrain::OnInitialize()
    {
        b2PolygonShape rightShape, leftShape;
        rightShape.SetAsBox(0.5f, 1.0f, b2Vec2(0.5f, 2.0f), 0.0f);
        leftShape.SetAsBox(0.5f, 1.0f, b2Vec2(-0.5f, 2.0f), 0.0f);

        m_rightSensor.SetBody(m_owner->GetBody());
        m_rightSensor.SetShape(&rightShape);
        m_leftSensor.SetBody(m_owner->GetBody());
        m_leftSensor.SetShape(&leftShape);

        const float visSize = 15.0f;
        b2PolygonShape visionShape;
        b2Vec2 verts[] = {
            b2Vec2(0.0f, 0.0f),
            b2Vec2(visSize / 3.0f, visSize * (2.0f/3.0f)),
            b2Vec2(2.6f * visSize/16.0f, visSize),
            b2Vec2(-2.6f * visSize/16.0f, visSize),
            b2Vec2(-visSize / 3.0f, visSize * (2.0f/3.0f))
        };
        visionShape.Set(verts, 5);

        m_visionSensor.SetBody(m_owner->GetBody());
        m_visionSensor.SetShape(&visionShape);

        m_prevPosition = m_owner->GetPosition();

        ChangeToWatchState();
        m_stateTimer = m_rand.GetReal(0.0f, 4.0f);
    }

    void GuardBrain::ReportSound(const vec2f &position, const float volume)
    {
        const float sqrDist = rob::Distance2(m_owner->GetPosition(), position);
        if (volume > 0.1f * sqrDist) // volume / sqrDist > 1.0f
        {
            HearSound(position);
        }
    }

    void GuardBrain::HearSound(const vec2f &position)
    {
        if (m_state != State::Chase)
        {
            m_soundHeard = true;
            m_soundSource = position;
//            rob::log::Info("heard!");
        }
    }

    bool GuardBrain::LookForPlayer()
    {
        if (m_visionSensor.PlayerSighted())
        {
            m_owner->SetDebugColor(Color::Orange);
            vec2f playerPos = FromB2(m_visionSensor.GetBody()->GetPosition());
            vec2f rayOrigin = m_owner->GetPosition() + m_owner->GetForward() * 1.5f;
            b2Body *body = m_nav->RayCast(rayOrigin, playerPos, PlayerBit, GuardBit|CakeBit);
            if (body == m_visionSensor.GetBody())
            {
                m_lastKnownPlayerPos = playerPos;
                return true;
            }
        }
        return false;
    }

    void GuardBrain::StartChasingIfPlayerSighted()
    {
        if (LookForPlayer())
            ChangeToChaseState();
    }

    void GuardBrain::StartSuspectingIfHeard()
    {
        if (m_soundHeard)
        {
            ChangeToSuspectState();
            m_soundHeard = false;
        }
    }

    void GuardBrain::ChangeToSuspectState()
    {
        m_owner->SetDebugColor(Color::Green);
        m_stateTimer = m_rand.GetReal(4.0f, 8.0f);
        m_path->Clear();
        m_state = State::Suspect;
    }

    void GuardBrain::ChangeToInspectState()
    {
        m_owner->SetDebugColor(Color::LightBlue);
        m_stateTimer = m_rand.GetReal(2.0f, 4.0f);
        m_state = State::Inspect;
    }

    void GuardBrain::ChangeToWatchState()
    {
        m_owner->SetDebugColor(Color::Yellow);
        m_stateTimer = m_rand.GetReal(4.0f, 8.0f);
        m_path->Clear();
        m_state = State::Watch;
    }

    void GuardBrain::ChangeToPatrolState()
    {
        m_owner->SetDebugColor(Color::Blue);
        NavigateRandom();
        m_state = State::Patrol;
    }

    void GuardBrain::ChangeToChaseState()
    {
        m_owner->SetDebugColor(Color::Red);
        m_stateTimer = 0.5f;
        Navigate(m_lastKnownPlayerPos);
        m_state = State::Chase;
    }

    void GuardBrain::AvoidObstacles()
    {
        const vec2f forward = m_owner->GetForward();
        const vec2f right = m_owner->GetRight();
        if (m_rightSensor.HitsObstacle() && !m_leftSensor.HitsObstacle())
        {
            vec2f off = m_owner->GetPosition() - right * 1.0f + forward * 2.0f;
            m_nav->GetMesh().GetClampedFaceIndex(&off);
            if (!m_path->TryInsertVertex(m_pathPos, off, 2.0f))
            {
                Navigate(m_path->GetDestination());
            }
        }
        else if (m_leftSensor.HitsObstacle() && !m_rightSensor.HitsObstacle())
        {
            vec2f off = m_owner->GetPosition() + right * 1.0f + forward * 2.0f;
            m_nav->GetMesh().GetClampedFaceIndex(&off);
            if (!m_path->TryInsertVertex(m_pathPos, off, 2.0f))
            {
                Navigate(m_path->GetDestination());
            }
        }
    }

    void GuardBrain::Move(float speed, float dt)
    {
//        AvoidObstacles();

        const vec2f target = m_path->GetVertex(m_pathPos);
        const vec2f delta = (target - m_owner->GetPosition());

        const float dist2 = delta.Length2();
        const float maxDist = 1.0f;
        if (dist2 < maxDist*maxDist)
        {
            m_stuckMeter = 0.0f;
            m_pathPos++;
            return;
        }

        const vec2f destDir = delta.SafeNormalized();
        vec2f offset = destDir + FromB2(m_owner->GetBody()->GetLinearVelocity());
//        vec2f offset = Lerp(destDir, FromB2(m_owner->GetBody()->GetLinearVelocity()), 0.5f);

        const vec2f dir = offset.SafeNormalized();
        const vec2f velocity = dir * speed;
        m_owner->GetBody()->SetLinearVelocity(ToB2(velocity));

        m_owner->SetRotation(dir);
        m_owner->GetBody()->SetAngularVelocity(0.0f);

        m_stuckMeter += speed * dt - rob::Distance(m_prevPosition, m_owner->GetPosition());
    }

    bool GuardBrain::IsEndOfPath() const
    { return !(m_pathPos < m_path->GetLength()); }

    void GuardBrain::Inspect(const vec2f &location)
    {
        ChangeToInspectState();
        Navigate(location + m_rand.GetDirection() * m_rand.GetReal(0.5, 2.5));
    }

    void GuardBrain::UpdateSuspect(const rob::GameTime &gameTime)
    {
        b2Body *body = m_owner->GetBody();
        body->SetLinearVelocity(b2Vec2(0.0f, 0.0f));

        const vec2f delta = m_soundSource - m_owner->GetPosition();
        const float angle = b2Atan2(-delta.x, delta.y);

        const float bodyAngle = std::fmod(body->GetAngle(), 2.0f * rob::PI_f);

        float angV = (angle - bodyAngle) / rob::PI_f;
        if (angV > 1.0f) angV -= 1.0f;
        if (angV < -1.0f) angV += 1.0f;

        if (angV > 0.0f)
            body->SetAngularVelocity(3.14f * 0.5f);
        else
            body->SetAngularVelocity(-3.14f * 0.5f);

        if (angV * angV < 0.2f)
        {
            Navigate(m_soundSource);
            m_state = State::Patrol;
            Inspect(m_soundSource);
        }

        if (m_stateTimer <= 0.0f)
            ChangeToPatrolState();

        m_owner->SetDebugColor(Color::Green);

        StartSuspectingIfHeard();
        StartChasingIfPlayerSighted();
    }

    void GuardBrain::UpdateInspect(const rob::GameTime &gameTime)
    {
        if (IsEndOfPath())
            ChangeToWatchState();
        else
        {
            Move(4.0f, gameTime.GetDeltaSeconds());
            if (IsStuck())
                ChangeToPatrolState();
        }
        m_prevPosition = m_owner->GetPosition();

        m_owner->SetDebugColor(Color::LightBlue);

        StartSuspectingIfHeard();
        StartChasingIfPlayerSighted();
    }

    void GuardBrain::UpdateWatch(const rob::GameTime &gameTime)
    {
        b2Body *body = m_owner->GetBody();
        body->SetLinearVelocity(b2Vec2(0.0f, 0.0f));

        if (m_rightSensor.HitsWall() && !m_leftSensor.HitsWall())
            m_watchTimer = m_rand.GetReal(1.0f, 2.0f);
        else if (m_leftSensor.HitsWall() && !m_rightSensor.HitsWall())
            m_watchTimer = -m_rand.GetReal(1.0f, 2.0f);

        if (m_watchTimer > 0.0f)
        {
            body->SetAngularVelocity(3.14f*0.5f);

            m_watchTimer -= gameTime.GetDeltaSeconds();
            if (m_watchTimer <= 0.0f)
                m_watchTimer = -m_rand.GetReal(1.0f, 2.0f);
        }
        else
        {
            body->SetAngularVelocity(-3.14f*0.5f);

            m_watchTimer += gameTime.GetDeltaSeconds();
            if (m_watchTimer >= 0.0f)
                m_watchTimer = m_rand.GetReal(1.0f, 2.0f);
        }

        if (m_stateTimer <= 0.0f)
            ChangeToPatrolState();

        m_owner->SetDebugColor(Color::Yellow);

        StartSuspectingIfHeard();
        StartChasingIfPlayerSighted();
    }

    void GuardBrain::UpdatePatrol(const rob::GameTime &gameTime)
    {
        if (IsEndOfPath())
            ChangeToWatchState();
        else
        {
            Move(4.0f, gameTime.GetDeltaSeconds());
            if (IsStuck())
                NavigateRandom();
        }
        m_prevPosition = m_owner->GetPosition();

        m_owner->SetDebugColor(Color::Blue);

        StartSuspectingIfHeard();
        StartChasingIfPlayerSighted();
    }

    void GuardBrain::UpdateChase(const rob::GameTime &gameTime)
    {
        if (m_visionSensor.PlayerSighted())
        {
            if (rob::Distance(m_owner->GetPosition(), FromB2(m_visionSensor.GetBody()->GetPosition())) < 2.5f)
                m_game->PlayerCaught(m_owner->GetPosition());
        }

        LookForPlayer();

        m_owner->SetDebugColor(Color::Red);

        if (m_stateTimer <= 0.0f)
        {
            Navigate(m_lastKnownPlayerPos);
            m_stateTimer = 0.5f;
        }

        Move(8.0f, gameTime.GetDeltaSeconds());
        if (IsEndOfPath())
        {
//            Navigate(m_lastKnownPlayerPos);
//            Move(8.0f, gameTime.GetDeltaSeconds());
//            if (IsEndOfPath())
                ChangeToWatchState();

            StartSuspectingIfHeard();
        }
        else
        {
            if (IsStuck())
                Navigate(m_lastKnownPlayerPos);
        }
    }

    void GuardBrain::Update(const rob::GameTime &gameTime)
    {
        if (m_stateTimer > 0.0f)
            m_stateTimer -= gameTime.GetDeltaSeconds();

        switch (m_state)
        {
        case State::Watch:
            UpdateWatch(gameTime);
            break;

        case State::Patrol:
            UpdatePatrol(gameTime);
            break;

        case State::Suspect:
            UpdateSuspect(gameTime);
            break;

        case State::Inspect:
            UpdateInspect(gameTime);
            break;

        case State::Chase:
            UpdateChase(gameTime);
            break;
        }
    }

    void GuardBrain::Navigate(const vec2f &pos)
    {
        m_pathPos = 0;
        m_nav->Navigate(m_owner->GetPosition(), pos, m_path);
        m_stuckMeter = 0.0f;
    }

    void GuardBrain::NavigateRandom()
    {
        Navigate(m_nav->GetRandomNavigableWorldPoint(m_rand));
    }

    void GuardBrain::DebugRender(rob::Renderer *renderer) const
    {
        m_nav->RenderPath(renderer, m_path);
        const vec2f pos = m_owner->GetPosition();
        renderer->SetColor(m_owner->GetDebugColor());
        renderer->DrawCircle(pos.x, pos.y, 1.2f);
    }

} // sneaky
