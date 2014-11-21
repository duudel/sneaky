
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




    GuardBrain::GuardBrain(Navigation *nav)
        : Brain()
        , m_nav(nav)
        , m_path(nullptr)
        , m_pathPos(0)
        , m_rand()
    {
        m_path = m_nav->ObtainNavPath();
//        m_rand.Seed(12312);
    }

    GuardBrain::~GuardBrain()
    {
        m_nav->ReturnNavPath(m_path);
    }

    void GuardBrain::Update(const rob::GameTime &gameTime)
    {
        if (m_pathPos < m_path->GetLength())
        {
            const vec2f &v = m_path->GetVertex(m_pathPos);
            const vec2f delta = (v - m_owner->GetPosition());

            const float dist2 = delta.Length2();
            const float maxDist = 0.5f;
            if (dist2 < maxDist*maxDist)
            {
                m_pathPos++;
            }

            const float speed = 5.0f;
            const vec2f velocity = delta.SafeNormalized() * speed;
            m_owner->GetBody()->SetLinearVelocity(ToB2(velocity));
        }
        else
        {
            Navigate(m_rand.GetDirection() * 20.0f);
        }
    }

    void GuardBrain::Navigate(const vec2f &pos)
    {
        m_pathPos = 0;
        m_path->Clear();
        m_nav->Navigate(m_owner->GetPosition(), pos, m_path);
    }

    void GuardBrain::DebugRender(rob::Renderer *renderer) const
    {
        m_nav->RenderPath(renderer, m_path);
    }

} // sneaky
