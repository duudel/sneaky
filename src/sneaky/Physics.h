
#ifndef H_SNEAKY_PHYSICS_H
#define H_SNEAKY_PHYSICS_H

#include "rob/math/Math.h"
#include "Box2D/Box2D.h"

namespace sneaky
{

    using rob::vec2f;
    using rob::mat4f;

    inline vec2f FromB2(const b2Vec2 &v)
    { return vec2f(v.x, v.y); }

    inline b2Vec2 ToB2(const vec2f &v)
    { return b2Vec2(v.x, v.y); }

    inline mat4f FromB2Transform(const b2Transform &tr)
    {
        return mat4f(tr.q.c, -tr.q.s, 0.0f, tr.p.x,
                     tr.q.s, tr.q.c, 0.0f, tr.p.y,
                     0.0f, 0.0f, 1.0f, 0.0f,
                     0.0f, 0.0f, 0.0f, 1.0f);
    }

} // sneaky

#endif // H_SNEAKY_PHYSICS_H

