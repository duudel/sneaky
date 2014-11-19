
#ifndef H_ROB_MATH_SIMD_SSE2_H
#define H_ROB_MATH_SIMD_SSE2_H

#include <xmmintrin.h>

namespace rob
{
namespace simd
{

    template <>
    struct Simd<float>
    {
        typedef __m128  v4 __attribute__((aligned(16)));
        typedef v4      v4_arg;
        typedef v4&     v4_argr;
        typedef float   type;

        static ROB_SIMD_NATIVE v4 Set(type s)
        {
            return _mm_set_ps1(s);
        }

        static ROB_SIMD_NATIVE v4 Set(type x, type y, type z, type w)
        {
            return _mm_set_ps(w, z, y, x);
        }

        static ROB_SIMD_NATIVE v4 MovAxyBxy(v4_arg a, v4_arg b)
        {
            return _mm_movelh_ps(a, b);
        }

        static ROB_SIMD_NATIVE v4 MovBzwAzw(v4_arg a, v4_arg b)
        {
            return _mm_movehl_ps(a, b);
        }

        template <uint8_t ix, uint8_t iy, uint8_t iz, uint8_t iw>
        static ROB_SIMD_NATIVE v4 Shuffle(v4_arg a, v4_arg b)
        {
            return _mm_shuffle_ps(a, b, _MM_SHUFFLE(iw, iz, iy, ix));
        }

        static ROB_SIMD_NON_NATIVE type GetX(v4_arg a)
        {
            type x;
            _mm_store_ss(&x, a);
            return x;
        }

        static ROB_SIMD_NON_NATIVE type GetY(v4_arg a)
        {
            type y;
            const v4 s = _mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 1, 1, 1));
            _mm_store_ss(&y, s);
            return y;
        }

        static ROB_SIMD_NON_NATIVE type GetZ(v4_arg a)
        {
            type z;
            const v4 s = _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 2, 2, 2));
            _mm_store_ss(&z, s);
            return z;
        }

        static ROB_SIMD_NON_NATIVE type GetW(v4_arg a)
        {
            type w;
            const v4 s = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 3, 3, 3));
            _mm_store_ss(&w, s);
            return w;
        }

        static ROB_SIMD_NATIVE v4 SplatX(v4_arg a)
        {
            return _mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 0, 0, 0));
        }

        static ROB_SIMD_NATIVE v4 SplatY(v4_arg a)
        {
            return _mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 1, 1, 1));
        }

        static ROB_SIMD_NATIVE v4 SplatZ(v4_arg a)
        {
            return _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 2, 2, 2));
        }

        static ROB_SIMD_NATIVE v4 SplatW(v4_arg a)
        {
            return _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 3, 3, 3));
        }

        static ROB_SIMD_NON_NATIVE v4 And(v4_arg a, v4_arg b)
        {
            return _mm_and_ps(a, b);
        }
        static ROB_SIMD_NON_NATIVE v4 Or(v4_arg a, v4_arg b)
        {
            return _mm_or_ps(a, b);
        }
        static ROB_SIMD_NON_NATIVE v4 Xor(v4_arg a, v4_arg b)
        {
            return _mm_xor_ps(a, b);
        }

        static ROB_SIMD_NATIVE v4 Add(v4_arg a, v4_arg b)
        {
            return _mm_add_ps(a, b);
        }

        static ROB_SIMD_NATIVE v4 Sub(v4_arg a, v4_arg b)
        {
            return _mm_sub_ps(a, b);
        }

        static ROB_SIMD_NATIVE v4 Mul(v4_arg a, type s)
        {
            return _mm_mul_ps(a, _mm_set_ps1(s));
        }

        static ROB_SIMD_NATIVE v4 Mul(v4_arg a, v4_arg b)
        {
            return _mm_mul_ps(a, b);
        }

        static ROB_SIMD_NATIVE v4 Div(v4_arg a, type s)
        {
            return _mm_div_ps(a, _mm_set_ps1(s));
        }

        static ROB_SIMD_NATIVE v4 Div(v4_arg a, v4_arg b)
        {
            return _mm_div_ps(a, b);
        }

        static ROB_SIMD_NON_NATIVE v4 Abs(v4_arg a)
        {
            const v4 b = Neg(a);
            return Max(a, b);
//            return _mm_andnot_ps(a, SIGN_MASK_X1Y1Z1W1);
        }

        static ROB_SIMD_NON_NATIVE v4 Neg(v4_arg a)
        {
            const v4 zero = _mm_set_ps1(0.0f);
            return _mm_sub_ps(zero, a);
        }

        static ROB_SIMD_NON_NATIVE v4 Dot(v4_arg a, v4_arg b)
        {
            const v4 m = _mm_mul_ps(a, b);
            const v4 mm = _mm_shuffle_ps(m, m, _MM_SHUFFLE(2, 3, 0, 1));
            const v4 t = _mm_add_ps(m, mm);
            const v4 tt = _mm_shuffle_ps(t, t, _MM_SHUFFLE(0, 1, 2, 3));
            return _mm_add_ps(t, tt);
        }

        static ROB_SIMD_NON_NATIVE v4 CrossRh(v4_arg a, v4_arg b)
        {
            const v4 aa = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1));
            const v4 bb = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1));
            const v4 r = _mm_sub_ps( _mm_mul_ps(b, aa), _mm_mul_ps(a, bb) );
            return _mm_shuffle_ps(r, r, _MM_SHUFFLE(3, 0, 2, 1));
        }

        static ROB_SIMD_NON_NATIVE v4 CrossLh(v4_arg a, v4_arg b)
        {
            const v4 aa = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1));
            const v4 bb = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1));
            const v4 r = _mm_sub_ps( _mm_mul_ps(a, bb), _mm_mul_ps(b, aa) );
            return _mm_shuffle_ps(r, r, _MM_SHUFFLE(3, 0, 2, 1));
        }

        static ROB_SIMD_NATIVE v4 Max(v4_arg a, v4_arg b)
        {
            return _mm_max_ps(a, b);
        }

        static ROB_SIMD_NATIVE v4 Min(v4_arg a, v4_arg b)
        {
            return _mm_min_ps(a, b);
        }

        static ROB_SIMD_NATIVE v4 Sqrt(v4_arg a)
        {
            return _mm_sqrt_ps(a);
        }

        static ROB_SIMD_NATIVE v4 FastInvSqrt(v4_arg a)
        {
            return _mm_rsqrt_ps(a);
        }

        static ROB_SIMD_NATIVE void Transpose4(v4_argr r0, v4_argr r1, v4_argr r2, v4_argr r3)
        {
            _MM_TRANSPOSE4_PS(r0, r1, r2, r3);
        }
    };

} // simd
} // rob

#endif // H_ROB_MATH_SIMD_SSE2_H

