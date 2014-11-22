
#include "GuardBrain.h"
#include "Physics.h"
#include "GameObject.h"
#include "Navigation.h"

#include "rob/application/GameTime.h"

namespace sneaky
{

    GuardBrain::GuardBrain(Navigation *nav, rob::Random &rand)
        : Brain()
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
        rightShape.SetAsBox(0.5f, 2.0f, b2Vec2(0.5f, 2.0f), 0.0f);
        leftShape.SetAsBox(0.5f, 2.0f, b2Vec2(-0.5f, 2.0f), 0.0f);

        m_rightSensor.SetBody(m_owner->GetBody());
        m_rightSensor.SetShape(&rightShape);
        m_leftSensor.SetBody(m_owner->GetBody());
        m_leftSensor.SetShape(&leftShape);

        b2PolygonShape visionShape;
        b2Vec2 verts[] = { b2Vec2(0.0f, 0.0f),
            b2Vec2(7.0f, 17.0f), b2Vec2(4.0f, 23.0f), b2Vec2(-4.0f, 23.0f), b2Vec2(-7.0f, 17.0f) };
        visionShape.Set(verts, 5);

        m_visionSensor.SetBody(m_owner->GetBody());
        m_visionSensor.SetShape(&visionShape);

        m_prevPosition = m_owner->GetPosition();
//        Navigate(m_owner->GetPosition());

        ChangeToWatchState();
        m_stateTimer = m_rand.GetReal(0.0f, 4.0f);
    }

    void GuardBrain::LookForPlayer()
    {
        if (m_visionSensor.PlayerSighted())
        {
            vec2f playerPos = FromB2(m_visionSensor.GetBody()->GetPosition());
            vec2f rayOrigin = m_owner->GetPosition() + m_owner->GetForward() * 1.5f;
            b2Body *body = m_nav->RayCast(rayOrigin, playerPos, PlayerBit/*, ignore cake*/);
            if (body == m_visionSensor.GetBody())
                ChangeToChaseState();
        }
    }

    void GuardBrain::ChangeToWatchState()
    {
        m_owner->SetColor(Color::Yellow);
        m_stateTimer = m_rand.GetReal(4.0f, 8.0f);
        m_path->Clear();
        m_state = State::Watch;
    }

    void GuardBrain::ChangeToPatrolState()
    {
        m_owner->SetColor(Color::Blue);
        NavigateRandom();
        m_state = State::Patrol;
    }

    void GuardBrain::ChangeToChaseState()
    {
        m_owner->SetColor(Color::Red);
        m_state = State::Chase;
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

        LookForPlayer();
    }

    void GuardBrain::UpdatePatrol(const rob::GameTime &gameTime)
    {
        if (m_pathPos < m_path->GetLength())
        {
            vec2f target = m_path->GetVertex(m_pathPos);

            const vec2f forward = FromB2(m_owner->GetBody()->GetWorldVector(b2Vec2(0.0f, 1.0f)));
            const vec2f right = FromB2(m_owner->GetBody()->GetWorldVector(b2Vec2(1.0f, 0.0f)));
            if (m_rightSensor.HitsObstacle() && !m_leftSensor.HitsObstacle())
            {
                const vec2f off = m_owner->GetPosition() - right * 1.0f + forward * 2.0f;
                if (!m_path->TryInsertVertex(m_pathPos, off, 2.0f))
                {
                    Navigate(m_path->GetDestination());
                }
            }
            else if (m_leftSensor.HitsObstacle() && !m_rightSensor.HitsObstacle())
            {
                const vec2f off = m_owner->GetPosition() + right * 1.0f + forward * 2.0f;
                if (!m_path->TryInsertVertex(m_pathPos, off, 2.0f))
                {
                    Navigate(m_path->GetDestination());
                }
            }

            target = m_path->GetVertex(m_pathPos);
            const vec2f delta = (target - m_owner->GetPosition());

            const float dist2 = delta.Length2();
            const float maxDist = 1.0f;
            if (dist2 < maxDist*maxDist)
            {
                m_stuckMeter = 0.0f;
                m_pathPos++;
            }

            const vec2f destDir = delta.SafeNormalized();
            vec2f offset = destDir;

            const float speed = 5.0f;
            const vec2f dir = offset.SafeNormalized();
            const vec2f velocity = dir * speed;
            m_owner->GetBody()->SetLinearVelocity(ToB2(velocity));
            m_owner->SetRotation(dir);

            const float dt = gameTime.GetDeltaSeconds();
            m_stuckMeter += speed * dt - rob::Distance(m_prevPosition, m_owner->GetPosition());
        }
        else
        {
            ChangeToWatchState();
        }
        m_prevPosition = m_owner->GetPosition();

        if (m_stuckMeter > 10.0f)
            NavigateRandom();

        LookForPlayer();
    }

    void GuardBrain::UpdateChase(const rob::GameTime &gameTime)
    {
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

        case State::Chase:
            UpdateChase(gameTime);
            break;
        }
    }

    void GuardBrain::Navigate(const vec2f &pos)
    {
        m_pathPos = 0;
        m_path->Clear();
        m_nav->Navigate(m_owner->GetPosition(), pos, m_path);
        m_stuckMeter = 0.0f;
    }

    void GuardBrain::NavigateRandom()
    {
        Navigate(m_rand.GetDirection() * m_rand.GetReal(5.0, 24.0));
    }

    void GuardBrain::DebugRender(rob::Renderer *renderer) const
    {
        m_nav->RenderPath(renderer, m_path);
    }

} // sneaky
