
#include "Brain.h"
#include "Physics.h"
#include "Input.h"
#include "GameObject.h"

#include "rob/application/GameTime.h"

namespace sneaky
{

    void PlayerBrain::Update(const rob::GameTime &gameTime)
    {
        const float dt = gameTime.GetDeltaSeconds();

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

} // sneaky
