
#ifndef H_DE_MATH_CONSTANTS_H
#define H_DE_MATH_CONSTANTS_H

#include "Types.h"

namespace rob
{

    constexpr real PI       = real(3.14159265359);
    constexpr float PI_f    = float(3.14159265359);
    constexpr double PI_d   = double(3.14159265359);

    constexpr float DEG2RAD     = PI / real(180.0);
    constexpr float DEG2RAD_f   = PI_f / 180.0f;
    constexpr double DEG2RAD_d  = PI_d / 180.0;

    #define DECL_OP(T, op) \
        friend T operator op (const PI_t &pi, T x) \
        { return T(pi) op x; } \
        friend T operator op (T x, const PI_t &pi) \
        { return x op T(pi); }

    // An idea for PI constant implementation
//    constexpr struct PI_t
//    {
//        operator float() const { return 3.14159265359f; }
//        operator double() const { return 3.14159265359; }
//
//        DECL_OP(float, +)
//        DECL_OP(float, -)
//        DECL_OP(float, *)
//        DECL_OP(float, /)
//
//        DECL_OP(double, +)
//        DECL_OP(double, -)
//        DECL_OP(double, *)
//        DECL_OP(double, /)
//    } PI;

    #undef DECL_OP

} // rob

#endif // H_DE_MATH_CONSTANTS_H

