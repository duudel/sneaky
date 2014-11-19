
#ifndef H_ROB_MATH_SIMD_SIMD_H
#define H_ROB_MATH_SIMD_SIMD_H

#ifdef __SSE2__
#define ROB_USE_SIMD            1
#define ROB_PLATFORM_SIMD_SSE2  1
#endif // __SSE2__

#define ROB_SIMD_NON_NATIVE  inline
#define ROB_SIMD_NATIVE      inline

#include "../../Types.h"

namespace rob
{
namespace simd
{

    // The S template parameter is for debugging. It can be used to instantiate both
    // scalar and SIMD versions.
    template <class T, size_t S>
    struct Simd;

} // simd
} // rob


#include "NoSimd.h"

#ifdef ROB_USE_SIMD

#ifdef ROB_PLATFORM_SIMD_SSE2
    #include "SSE2.h"
#else
    #error SIMD not supported
#endif

#endif // USE_SIMD

namespace rob
{
namespace simd
{

    // The S template parameter is for debugging. It can be used to instantiate both
    // scalar and SIMD versions.
    template <class T, size_t S>
    struct Constants
    {
        typedef typename Simd<T, S>::v4 v4;
        static const v4 MASK_X;
        static const v4 MASK_Y;
        static const v4 MASK_Z;
        static const v4 MASK_W;
    };

    template <class T, size_t S>
    const typename Constants<T, S>::v4 Constants<T, S>::MASK_X = Simd<T, S>::Set(float_bits.full, 0.0f, 0.0f, 0.0f);
    template <class T, size_t S>
    const typename Constants<T, S>::v4 Constants<T, S>::MASK_Y = Simd<T, S>::Set(0.0f, float_bits.full, 0.0f, 0.0f);
    template <class T, size_t S>
    const typename Constants<T, S>::v4 Constants<T, S>::MASK_Z = Simd<T, S>::Set(0.0f, 0.0f, float_bits.full, 0.0f);
    template <class T, size_t S>
    const typename Constants<T, S>::v4 Constants<T, S>::MASK_W = Simd<T, S>::Set(0.0f, 0.0f, 0.0f, float_bits.full);

} // simd
} // rob

#endif // H_ROB_MATH_SIMD_SIMD_H

