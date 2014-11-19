
#include "Input.h"

#include "rob/renderer/Renderer.h"
#include "rob/math/Math.h"
#include "rob/math/Projection.h"

namespace sneaky
{

    static vec2f ScreenToWorld(const View &view, int x, int y)
    {
        const Viewport &viewport = view.m_viewport;
        const vec4f vp = vec4f(viewport.w, -viewport.h, 1.0f, 1.0f);
        const vec4f vp_pos = vec4f(x - viewport.x, y - viewport.y, 0.0f, 1.0f);

        const vec4f ndc = (vp_pos * 2.0f - vec4f(viewport.w, viewport.h, 0.0f, 0.0f)) / vp;
        const vec4f wp = Unproject_Orthogonal_lh(view.m_projection, ndc);
        return vec2f(wp.x, wp.y);
    }

    vec2f Input::GetMouseWorldPosition() const
    {
         const vec2f screenPos = m_mouse.GetPosition();
         return ScreenToWorld(*m_view, screenPos.x, screenPos.y);
    }

} // sneaky
