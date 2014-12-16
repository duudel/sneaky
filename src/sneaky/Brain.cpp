
#include "Brain.h"
#include "Physics.h"
#include "Input.h"
#include "GameObject.h"
#include "SneakyState.h"

#include "rob/application/GameTime.h"
#include "rob/renderer/Renderer.h"

namespace sneaky
{

    void PlayerBrain::OnInitialize()
    {
        b2CircleShape shape;
        shape.m_radius = 1.1f;
        m_cakeSensor.SetBody(m_owner->GetBody());
        m_cakeSensor.SetShape(&shape);
    }

    struct SoundCallback : public b2QueryCallback
    {
        const vec2f m_position;
        const float m_volume;

        explicit SoundCallback(const vec2f &position, const float volume)
            : m_position(position), m_volume(volume) { }

        bool ReportFixture(b2Fixture *fixture) override
        {
            const b2Filter &filter = fixture->GetFilterData();
            if (filter.categoryBits & GuardBit)
            {
                void *userData = fixture->GetBody()->GetUserData();
                if (userData)
                {
                    GameObject *guard = static_cast<GameObject*>(userData);
                    guard->GetBrain()->ReportSound(m_position, m_volume);
                }
            }
            return true;
        }

        bool ShouldQueryParticleSystem(const b2ParticleSystem *ps) override
        { return false; }
    };

    void PlayerBrain::Update(const rob::GameTime &gameTime)
    {
        if (m_cakeSensor.HitsCake())
            m_game->CakeEaten();

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

        const vec2f velocity = offset * speed;
        const float volume = (offset * speed).Length2();
        SoundCallback soundCb(m_owner->GetPosition(), volume * 0.5f);
        b2CircleShape shape;
        shape.m_radius = 16.0f;
        const b2Transform &tr = m_owner->GetBody()->GetTransform();
        m_owner->GetBody()->GetWorld()->QueryShapeAABB(&soundCb, shape, tr);

        const vec2f globalVel = FromB2(m_owner->GetBody()->GetWorldVector(ToB2(velocity)));
        m_owner->GetBody()->SetLinearVelocity(ToB2(globalVel));
    }

    void PlayerBrain::Render(rob::Renderer *renderer) const
    {
        renderer->SetColor(Color(2.0f, 1.0f, 0.6f));
        renderer->SetModel(mat4f::Identity);
        const vec2f dpos = m_owner->GetPosition() + m_target; //ClampedVectorLength(m_target, 1.5f);
        renderer->BindColorShader();
        renderer->DrawFilledCircle(dpos.x, dpos.y, 1.0f * 0.5f, Color(0.2f, 0.5f, 0.5f, 0.5f));
    }

} // sneaky
