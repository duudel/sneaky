
#include "Brain.h"
#include "Physics.h"
#include "Input.h"
#include "GameObject.h"

#include "rob/application/GameTime.h"
#include "rob/renderer/Renderer.h"

namespace sneaky
{

    void PlayerBrain::Update(const rob::GameTime &gameTime)
    {
//        const float dt = gameTime.GetDeltaSeconds();
//        const vec2f target = m_input->GetMouseWorldPosition();
//        const vec2f dir = (target - m_owner->GetPosition()).Normalized();

        m_target += m_input->GetMouseDelta() * vec2f(1.0f, -1.0f) * 0.1f;
        ClampVectorLength(m_target, 10.0f);
        const vec2f dir = m_target.SafeNormalized();
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

        float speed = 4.0f;
        if (m_input->KeyDown(Keyboard::Scancode::LShift))
        {
            speed = 8.0f;
        }

        const vec2f velocity = offset * speed; // * dt;
//        m_owner->MoveLocal(velocity);
        const vec2f globalVel = FromB2(m_owner->GetBody()->GetWorldVector(ToB2(velocity)));
        m_owner->GetBody()->SetLinearVelocity(ToB2(globalVel));
    }

    void PlayerBrain::DebugRender(rob::Renderer *renderer) const
    {
        renderer->SetColor(Color(2.0f, 1.0f, 0.6f));
        renderer->SetModel(mat4f::Identity);
        const vec2f dpos = m_owner->GetPosition() + m_target; //ClampedVectorLength(m_target, 1.5f);
        renderer->BindColorShader();
        renderer->DrawFilledCircle(dpos.x, dpos.y, 1.0f * 0.5f, Color(0.2f, 0.5f, 0.5f, 0.5f));
    }

} // sneaky
