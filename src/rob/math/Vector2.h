
#ifndef H_ROB_MATH_VECTOR2_H
#define H_ROB_MATH_VECTOR2_H

#include "simd/Simd.h"

namespace rob
{

    template <class T>
    struct Vector2
    {
        union {
            struct { T x, y; } v;
            struct { T x, y; };
        };

        static Vector2 Zero;
        static Vector2 Unit;
        static Vector2 UnitX;
        static Vector2 UnitY;

        inline Vector2() { }
        inline Vector2(T s) { x = s; y = s; }
        inline Vector2(T s, T t) { x = s; y = t; }
        inline Vector2(const Vector2 &vec) { v = vec.v; }

        void CopyTo(float (&vec)[2]) const
        {
            vec[0] = x; vec[1] = y;
        }

        inline Vector2& operator = (const Vector2 &vec)
        {
            v = vec.v;
            return *this;
        }

        inline Vector2& operator += (const Vector2 &vec)
        {
            x += vec.x; y += vec.y;
            return *this;
        }

        inline Vector2& operator -= (const Vector2 &vec)
        {
            x -= vec.x; y -= vec.y;
            return *this;
        }

        inline Vector2& operator *= (const Vector2 &vec)
        {
            x *= vec.x; y *= vec.y;
            return *this;
        }

        inline Vector2& operator *= (T s)
        {
            x *= s; y *= s;
            return *this;
        }

        inline Vector2& operator /= (const Vector2 &vec)
        {
            x /= vec.x; y /= vec.y;
            return *this;
        }

        inline Vector2& operator /= (T s)
        {
            x /= s; y /= s;
            return *this;
        }

        inline Vector2 operator - () const
        {
            return Vector2(-x, -y);
        }

        inline friend Vector2 operator + (const Vector2 &v1, const Vector2 &v2)
        {
            return Vector2(v1.x + v2.x, v1.y + v2.y);
        }

        inline friend Vector2 operator - (const Vector2 &v1, const Vector2 &v2)
        {
            return Vector2(v1.x - v2.x, v1.y - v2.y);
        }

        inline friend Vector2 operator * (const Vector2 &v1, T s)
        {
            return Vector2(v1.x * s, v1.y * s);
        }

        inline friend Vector2 operator * (T s, const Vector2 &v1)
        {
            return Vector2(v1.x * s, v1.y * s);
        }

        inline friend Vector2 operator * (const Vector2 &v1, const Vector2 &v2)
        {
            return Vector2(v1.x * v2.x, v1.y * v2.y);
        }

        inline friend Vector2 operator / (const Vector2 &v1, T s)
        {
            return Vector2(v1.x / s, v1.y / s);
        }

        inline friend Vector2 operator / (const Vector2 &v1, const Vector2 &v2)
        {
            return Vector2(v1.x / v2.x, v1.y / v2.y);
        }

        inline T Dot(const Vector2 &vec) const
        {
            return Vector2(x * vec.x, y * vec.y);
        }

        inline Vector2 DotV(const Vector2 &vec) const
        {
            return Vector2(Dot(vec));
        }

        inline T Length2() const
        {
            return x * x + y * y;
        }

        inline Vector2 Length2V() const
        {
            return Vector2(x * x + y * y);
        }

        inline T Length() const
        {
            T len2 = Length2();
            return Sqrt(len2);
        }

        inline Vector2 LengthV() const
        {
            T len2 = Length2();
            return Vector2(Sqrt(len2));
        }

        inline void Normalize()
        {
            *this /= Length();
        }

        inline Vector2 Normalized() const
        {
            return *this / Length();
        }

        inline void SafeNormalize()
        {
            T len = Length();
            if (len > 0.000001) *this /= len;
        }

        inline Vector2 SafeNormalized() const
        {
            T len = Length();
            return (len > 0.000001) ? *this / len : *this;
        }
    };

    template <class T>
    Vector2<T> Vector2<T>::Zero(0);

    template <class T>
    Vector2<T> Vector2<T>::Unit(1);

    template <class T>
    Vector2<T> Vector2<T>::UnitX(1, 0);

    template <class T>
    Vector2<T> Vector2<T>::UnitY(0, 1);

} // rob

#endif // H_ROB_MATH_VECTOR2_H
