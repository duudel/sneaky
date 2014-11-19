
#ifndef H_ROB_COLOR_H
#define H_ROB_COLOR_H

namespace rob
{

    struct Color
    {
        Color() : r(), g(), b(), a() { }
        explicit Color(float x)
            : r(x), g(x), b(x), a(1.0f) { }
        Color(float ar, float ag, float ab, float aa = 1.0f)
            : r(ar), g(ag), b(ab), a(aa) { }

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

