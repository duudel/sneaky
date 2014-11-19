
#ifndef H_ROB_MATH_VECTOR4_H
#define H_ROB_MATH_VECTOR4_H

#include "simd/Simd.h"

namespace rob
{

    template <class T, size_t S>
    struct Vector4
    {
        typedef typename simd::Simd<T, S> simd_;
        typedef typename simd::Simd<T, S>::v4 v4;

        union {
            v4 v;
            struct { T x, y, z, w; };
        };

        static Vector4 Zero;
        static Vector4 Unit;
        static Vector4 UnitX;
        static Vector4 UnitY;
        static Vector4 UnitZ;
        static Vector4 UnitW;

        inline Vector4() { }
        inline Vector4(T x) { v = simd_::Set(x); }
        inline Vector4(T x, T y, T z, T w) { v = simd_::Set(x, y, z, w); }
        inline Vector4(const Vector4 &vec) { v = vec.v; }

        void CopyTo(float (&vec)[4]) const
        {
            vec[0] = x; vec[1] = y; vec[2] = z; vec[3] = w;
        }

        inline Vector4& operator = (const Vector4 &vec)
        {
            v = vec.v;
            return *this;
        }

        inline Vector4& operator += (const Vector4 &vec)
        {
            v = simd_::Add(v, vec.v);
            return *this;
        }

        inline Vector4& operator -= (const Vector4 &vec)
        {
            v = simd_::Sub(v, vec.v);
            return *this;
        }

        inline Vector4& operator *= (const Vector4 &vec)
        {
            v = simd_::Mul(v, vec.v);
            return *this;
        }

        inline Vector4& operator *= (T s)
        {
            v = simd_::Mul(v, s);
            return *this;
        }

        inline Vector4& operator /= (const Vector4 &vec)
        {
            v = simd_::Div(v, vec.v);
            return *this;
        }

        inline Vector4& operator /= (T s)
        {
            v = simd_::Div(v, s);
            return *this;
        }

        inline Vector4 operator - () const
        {
            Vector4 r;
            r.v = simd_::Neg(v);
            return r;
        }

        inline friend Vector4 operator + (const Vector4 &v1, const Vector4 &v2)
        {
            Vector4 r;
            r.v = simd_::Add(v1.v, v2.v);
            return r;
        }

        inline friend Vector4 operator - (const Vector4 &v1, const Vector4 &v2)
        {
            Vector4 r;
            r.v = simd_::Sub(v1.v, v2.v);
            return r;
        }

        inline friend Vector4 operator * (const Vector4 &v1, T s)
        {
            Vector4 r;
            r.v = simd_::Mul(v1.v, s);
            return r;
        }

        inline friend Vector4 operator * (T s, const Vector4 &v1)
        {
            Vector4 r;
            r.v = simd_::Mul(v1.v, s);
            return r;
        }

        inline friend Vector4 operator * (const Vector4 &v1, const Vector4 &v2)
        {
            Vector4 r;
            r.v = simd_::Mul(v1.v, v2.v);
            return r;
        }

        inline friend Vector4 operator / (const Vector4 &v1, T s)
        {
            Vector4 r;
            r.v = simd_::Div(v1.v, s);
            return r;
        }

        inline friend Vector4 operator / (const Vector4 &v1, const Vector4 &v2)
        {
            Vector4 r;
            r.v = simd_::Div(v1.v, v2.v);
            return r;
        }

        inline T Dot(const Vector4 &vec) const
        {
            v4 len2 = simd_::Dot(v, vec.v);
            return simd_::GetX(len2);
        }

        inline Vector4 DotV(const Vector4 &vec) const
        {
            Vector4 r;
            r.v = simd_::Dot(v, vec.v);
            return r;
        }

        inline Vector4 CrossRh(const Vector4 &vec) const
        {
            Vector4 r;
            r.v = simd_::CrossRh(v, vec.v);
            return r;
        }

        inline Vector4 CrossLh(const Vector4 &vec) const
        {
            Vector4 r;
            r.v = simd_::CrossLh(v, vec.v);
            return r;
        }

        inline T Length2() const
        {
            v4 len2 = simd_::Dot(v, v);
            return simd_::GetX(len2);
        }

        inline Vector4 Length2V() const
        {
            Vector4 len2;
            len2.v = simd_::Dot(v, v);
            return len2;
        }

        inline T Length() const
        {
            v4 len2 = simd_::Dot(v, v);
            v4 len = simd_::Sqrt(len2);
            return simd_::GetX(len);
        }

        inline Vector4 LengthV() const
        {
            Vector4 len;
            v4 len2 = simd_::Dot(v, v);
            len.v = simd_::Sqrt(len2);
            return len;
        }

        inline void Normalize()
        {
            v = simd_::Div(v, LengthV().v);
        }

        inline void NormalizeFast()
        {
            v = simd_::Mul(v, simd_::FastInvSqrt(Length2V().v));
        }

        inline Vector4 Normalized() const
        {
            Vector4 r;
            r.v = simd_::Div(v, LengthV().v);
            return r;
        }

        inline Vector4 NormalizedFast() const
        {
            Vector4 r;
            r.v = simd_::Mul(v, simd_::FastInvSqrt(Length2V().v));
            return r;
        }

        inline Vector4 xxxx() const
        {
            Vector4 r;
            r.v = simd_::SplatX(v);
            return r;
        }

        inline Vector4 yyyy() const
        {
            Vector4 r;
            r.v = simd_::SplatY(v);
            return r;
        }

        inline Vector4 zzzz() const
        {
            Vector4 r;
            r.v = simd_::SplatZ(v);
            return r;
        }

        inline Vector4 wwww() const
        {
            Vector4 r;
            r.v = simd_::SplatW(v);
            return r;
        }

        template <uint8_t ix, uint8_t iy, uint8_t iz, uint8_t iw>
        inline Vector4 Swizzle() const
        {
            Vector4 r;
            r.v = simd_::Shuffle<ix, iy, iz, iw>(v, v);
            return r;
        }
    };

    template <class T, size_t S>
    Vector4<T, S> Vector4<T, S>::Zero(0);

    template <class T, size_t S>
    Vector4<T, S> Vector4<T, S>::Unit(1);

    template <class T, size_t S>
    Vector4<T, S> Vector4<T, S>::UnitX(1, 0, 0, 0);

    template <class T, size_t S>
    Vector4<T, S> Vector4<T, S>::UnitY(0, 1, 0, 0);

    template <class T, size_t S>
    Vector4<T, S> Vector4<T, S>::UnitZ(0, 0, 1, 0);

    template <class T, size_t S>
    Vector4<T, S> Vector4<T, S>::UnitW(0, 0, 0, 1);

} // rob

#endif // H_ROB_MATH_VECTOR4_H

