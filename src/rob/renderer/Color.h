
#ifndef H_ROB_COLOR_H
#define H_ROB_COLOR_H

#include "../math/Vector4.h"

namespace rob
{

    struct Color
    {
        Color() : r(), g(), b(), a() { }
        explicit Color(float x)
            : r(x), g(x), b(x), a(1.0f) { }
        Color(float ar, float ag, float ab, float aa = 1.0f)
            : r(ar), g(ag), b(ab), a(aa) { }
        explicit Color(const vec4f &vec)
            : r(vec.x), g(vec.y), b(vec.z), a(vec.w) { }

        vec4f ToVec4() const { return vec4f(r, g, b, a); }

        float r, g, b, a;

        static const Color Black;
        static const Color White;

        static const Color Red;
        static const Color Green;
        static const Color Blue;
        static const Color DarkRed;
        static const Color DarkGreen;
        static const Color DarkBlue;
        static const Color LightRed;
        static const Color LightGreen;
        static const Color LightBlue;
        static const Color Yellow;
        static const Color Orange;
        static const Color Magenta;
    };

} // rob

#endif // H_ROB_COLOR_H

