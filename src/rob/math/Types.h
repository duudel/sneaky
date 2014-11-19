
#ifndef H_ROB_MATH_TYPES_H
#define H_ROB_MATH_TYPES_H

#include "../Types.h"

namespace rob
{

    template <class T>
    class Vector2;

    template <class T, size_t S = 0>
    class Vector4;

    template <class T, size_t S = 0>
    class Matrix4;


    typedef float   float32;
    typedef double  float64;
    typedef float32 real;

    typedef Vector2<float32>    vec2f;
    typedef Vector2<int32_t>    vec2i;
    typedef Vector4<float32>    vec4f;
    typedef Vector4<int32_t>    vec4i;
    typedef Matrix4<float32>    mat4f;
    typedef Matrix4<int32_t>    mat4i;

} // rob

#endif // H_ROB_MATH_TYPES_H

