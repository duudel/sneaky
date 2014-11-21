
#include "Brain.h"
#include "Physics.h"
#include "Input.h"
#include "GameObject.h"
#include "Navigation.h"

#include "rob/application/GameTime.h"

namespace sneaky
{

    void PlayerBrain::Update(const rob::GameTime &gameTime)
    {
//        const float dt = gameTime.GetDeltaSeconds();

        const vec2f target = m_input->GetMouseWorldPosition();
        const vec2f dir = (target - m_owner->GetPosition()).Normalized();
//        const vec2f dir = target.Normalized();
        if (dir.y > 0.001f || dir.y < -0.001f) m_owner->SetRotation(dir);

        vec2f offset = vec2f::Zero;
        if (m_input->KeyDown(Keyboard::Scancode::W))
        {
            offset += vec2f::UnitY;
        }
        else if (m_input->KeyDown(Keyboard::Scancode::S))
        {
            offset -= vec2f::UnitY;
        }
        if (m_input->KeyDown(Keyboard::Scancode::A))
        {
            offset -= vec2f::UnitX;
        }
        else if (m_input->KeyDown(Keyboard::Scancode::D))
        {
            offset += vec2f::UnitX;
        }
        offset.SafeNormalize();

        const float speed = 5.0f;
        const vec2f velocity = offset * speed; // * dt;
//        m_owner->MoveLocal(velocity);
        const vec2f globalVel = FromB2(m_owner->GetBody()->GetWorldVector(ToB2(velocity)));
        m_owner->GetBody()->SetLinearVelocity(ToB2(globalVel));
    }




    GuardBrain::GuardBrain(Navigation *nav, Random &rand)
        : Brain()
        , m_nav(nav)
        , m_path(nullptr)
        , m_pathPos(0)
        , m_rand(rand)
        , m_rightSensor()
        , m_leftSensor()
        , m_stuckMeter(0.0f)
        , m_prevPosition(0.0f, 0.0f)
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

        m_prevPosition = m_owner->GetPosition();
//        Navigate(m_owner->GetPosition());
    }

    void GuardBrain::Update(const rob::GameTime &gameTime)
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

//            const float angle = m_owner->GetBody()->GetAngle();
//            const float destAngle = b2Atan2(-dir.x, dir.y);
//            const float angularVel = (destAngle - angle) * 5.0f;
//            m_owner->GetBody()->SetAngularVelocity(angularVel);
        }
        else
        {
            Navigate(m_rand.GetDirection() * m_rand.GetReal(0.0, 20.0));
        }
        m_prevPosition = m_owner->GetPosition();

        if (m_stuckMeter > 10.0f)
        {
            Navigate(m_path->GetDestination());
        }
    }

    void GuardBrain::Navigate(const vec2f &pos)
    {
        m_pathPos = 0;
        m_path->Clear();
        m_nav->Navigate(m_owner->GetPosition(), pos, m_path);
        m_stuckMeter = 0.0f;
    }

    void GuardBrain::DebugRender(rob::Renderer *renderer) const
    {
        m_nav->RenderPath(renderer, m_path);
    }

} // sneaky
