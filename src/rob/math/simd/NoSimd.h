
#ifndef H_ROB_MATH_SIMD_NO_SIMD_H
#define H_ROB_MATH_SIMD_NO_SIMD_H

#include <cmath>
#include "../Functions.h"
#include "../../Types.h"

namespace rob
{
namespace simd
{

    constexpr union float_bits_t {
        float full;
        uint32_t full_i;
        constexpr float_bits_t() : full_i(0xffffffff) { }
    } float_bits;


    template <class T, size_t S = 0>
    struct Simd
    {
        struct v4
        {
            T v[4];
            inline T& operator [](size_t i) { return v[i]; }
            inline T operator [](size_t i) const { return v[i]; }
        };
        typedef const v4&   v4_arg;
        typedef v4&         v4_argr;
        typedef T           type;
        typedef typename sized_int<sizeof(T)>::utype    utype;

        static ROB_SIMD_NON_NATIVE v4 Set(type s)
        {
            return v4{ s, s, s, s };
        }

        static ROB_SIMD_NON_NATIVE v4 Set(type x, type y, type z, type w)
        {
            return v4{ x, y, z, w };
        }

        static ROB_SIMD_NON_NATIVE v4 MovAxyBxy(v4_arg a, v4_arg b)
        {
            return v4{ a[0], a[1], b[0], b[1] };
        }

        static ROB_SIMD_NON_NATIVE v4 MovBzwAzw(v4_arg a, v4_arg b)
        {
            return v4{ b[2], b[3], a[2], a[3] };
        }

        template <uint8_t ix, uint8_t iy, uint8_t iz, uint8_t iw>
        static ROB_SIMD_NON_NATIVE v4 Shuffle(v4_arg a, v4_arg b)
        {
            return v4{ a[ix], a[iy], b[iz], b[iw] };
        }

        static ROB_SIMD_NON_NATIVE type GetX(v4_arg a)
        { return a[0]; }

        static ROB_SIMD_NON_NATIVE type GetY(v4_arg a)
        { return a[1]; }

        static ROB_SIMD_NON_NATIVE type GetZ(v4_arg a)
        { return a[2]; }

        static ROB_SIMD_NON_NATIVE type GetW(v4_arg a)
        { return a[3]; }

        static ROB_SIMD_NON_NATIVE v4 SplatX(v4_arg a)
        { return v4{ a[0], a[0], a[0], a[0] }; }

        static ROB_SIMD_NON_NATIVE v4 SplatY(v4_arg a)
        { return v4{ a[1], a[1], a[1], a[1] }; }

        static ROB_SIMD_NON_NATIVE v4 SplatZ(v4_arg a)
        { return v4{ a[2], a[2], a[2], a[2] }; }

        static ROB_SIMD_NON_NATIVE v4 SplatW(v4_arg a)
        { return v4{ a[3], a[3], a[3], a[3] }; }


        static ROB_SIMD_NON_NATIVE v4 And(v4_arg a, v4_arg b)
        {
            v4 r;
            const type *ap = a.v;
            const utype *ua = reinterpret_cast<const utype*>(ap);
            const type *bp = b.v;
            const utype *ub = reinterpret_cast<const utype*>(bp);
            type *rp = r.v;
            utype *ur = reinterpret_cast<utype*>(rp);
            ur[0] = ua[0] & ub[0];
            ur[1] = ua[1] & ub[1];
            ur[2] = ua[2] & ub[2];
            ur[3] = ua[3] & ub[3];
            return r;
        }
        static ROB_SIMD_NON_NATIVE v4 Or(v4_arg a, v4_arg b)
        {
            v4 r;
            const type *ap = a.v;
            const utype *ua = reinterpret_cast<const utype*>(ap);
            const type *bp = b.v;
            const utype *ub = reinterpret_cast<const utype*>(bp);
            type *rp = r.v;
            utype *ur = reinterpret_cast<utype*>(rp);
            ur[0] = ua[0] | ub[0];
            ur[1] = ua[1] | ub[1];
            ur[2] = ua[2] | ub[2];
            ur[3] = ua[3] | ub[3];
            return r;
        }
        static ROB_SIMD_NON_NATIVE v4 Xor(v4_arg a, v4_arg b)
        {
            v4 r;
            const type *ap = a.v;
            const utype *ua = reinterpret_cast<const utype*>(ap);
            const type *bp = b.v;
            const utype *ub = reinterpret_cast<const utype*>(bp);
            type *rp = r.v;
            utype *ur = reinterpret_cast<utype*>(rp);
            ur[0] = ua[0] ^ ub[0];
            ur[1] = ua[1] ^ ub[1];
            ur[2] = ua[2] ^ ub[2];
            ur[3] = ua[3] ^ ub[3];
            return r;
        }

        static ROB_SIMD_NON_NATIVE v4 Add(v4_arg a, v4_arg b)
        {
            return v4{ a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3] };
        }

        static ROB_SIMD_NON_NATIVE v4 Sub(v4_arg a, v4_arg b)
        {
            return v4{ a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3] };
        }

        static ROB_SIMD_NON_NATIVE v4 Mul(v4_arg a, type s)
        {
            return v4{ a[0] * s, a[1] * s, a[2] * s, a[3] * s };
        }

        static ROB_SIMD_NON_NATIVE v4 Mul(v4_arg a, v4_arg b)
        {
            return v4{ a[0] * b[0], a[1] * b[1], a[2] * b[2], a[3] * b[3] };
        }

        static ROB_SIMD_NON_NATIVE v4 Div(v4_arg a, type s)
        {
            return v4{ a[0] / s, a[1] / s, a[2] / s, a[3] / s };
        }

        static ROB_SIMD_NON_NATIVE v4 Div(v4_arg a, v4_arg b)
        {
            return v4{ a[0] / b[0], a[1] / b[1], a[2] / b[2], a[3] / b[3] };
        }

        static ROB_SIMD_NON_NATIVE v4 Abs(v4_arg a)
        {
            return v4{ Abs(a[0]), Abs(a[1]), Abs(a[2]), Abs(a[3]) };
        }

        static ROB_SIMD_NON_NATIVE v4 Neg(v4_arg a)
        {
            return v4{ -a[0], -a[1], -a[2], -a[3] };
        }

        static ROB_SIMD_NON_NATIVE v4 Dot(v4_arg a, v4_arg b)
        {
            const v4 m = Mul(a, b);
            const type s = m[0] + m[1] + m[2] + m[3];
            return v4{ s, s, s, s };
        }

        static ROB_SIMD_NON_NATIVE v4 CrossRh(v4_arg a, v4_arg b)
        {
            const v4 r = Sub( Mul(b, Shuffle(a, a, 1, 2, 0, 3)),
                        Mul(a, Shuffle(b, b, 1, 2, 0, 3)) );
            return Shuffle(r, r, 1, 2, 0, 3);
        }

        static ROB_SIMD_NON_NATIVE v4 CrossLh(v4_arg a, v4_arg b)
        {
            const v4 r = Sub( Mul(a, Shuffle<1, 2, 0, 3>(b, b)),
                        Mul(b, Shuffle<1, 2, 0, 3>(a, a)) );
            return Shuffle<1, 2, 0, 3>(r, r);
        }

        static ROB_SIMD_NON_NATIVE v4 Max(v4_arg a, v4_arg b)
        {
            return v4{ Max(a[0], b[0]), Max(a[1], b[1]), Max(a[2], b[2]), Max(a[3], b[3]) };
        }

        static ROB_SIMD_NON_NATIVE v4 Min(v4_arg a, v4_arg b)
        {
            return v4{ Min(a[0], b[0]), Min(a[1], b[1]), Min(a[2], b[2]), Min(a[3], b[3]) };
        }

        static ROB_SIMD_NON_NATIVE v4 Sqrt(v4_arg a)
        {
            return v4{ std::sqrt(a[0]), std::sqrt(a[1]), std::sqrt(a[2]), std::sqrt(a[3]) };
        }

        static ROB_SIMD_NON_NATIVE v4 FastInvSqrt(v4_arg a)
        {
            return v4{ type(1) / std::sqrt(a[0]), type(1) / std::sqrt(a[1]), type(1) / std::sqrt(a[2]), type(1) / std::sqrt(a[3]) };
        }

        static ROB_SIMD_NON_NATIVE void Transpose4(v4_argr r0, v4_argr r1, v4_argr r2, v4_argr r3)
        {
            type temp;
            temp = r1[0]; r1[0] = r0[1]; r0[1] = temp;
            temp = r2[0]; r2[0] = r0[2]; r0[2] = temp;
            temp = r3[0]; r3[0] = r0[3]; r0[3] = temp;
            temp = r2[1]; r2[1] = r1[2]; r1[2] = temp;
            temp = r3[1]; r3[1] = r1[3]; r1[3] = temp;
            temp = r3[2]; r3[2] = r2[3]; r2[3] = temp;
        }
    };

} // simd
} // rob

#endif // H_ROB_MATH_SIMD_NO_SIMD_H

