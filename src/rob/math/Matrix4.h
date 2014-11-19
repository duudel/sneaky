
#ifndef H_ROB_MATH_MATRIX4_H
#define H_ROB_MATH_MATRIX4_H

#include "simd/Simd.h"

namespace rob
{

    template <class T, size_t S>
    struct Vector4;

    /// Row major 4x4 matrix.
    template <class T, size_t S>
    struct Matrix4
    {
        typedef simd::Simd<T, S> simd_;
        typedef typename simd::Simd<T, S>::v4 v4;

        typedef Vector4<T, S> vec4;

        union {
            struct
            {
                v4 row0;
                v4 row1;
                v4 row2;
                v4 row3;
            };
            struct
            {
                T m00, m01, m02, m03;
                T m10, m11, m12, m13;
                T m20, m21, m22, m23;
                T m30, m31, m32, m33;
            };
        };

        static Matrix4 Identity;
        static Matrix4 Zero;

        inline Matrix4() { }

        inline Matrix4(T x)
        {
            row0 = simd_::Set(x);
            row1 = simd_::Set(x);
            row2 = simd_::Set(x);
            row3 = simd_::Set(x);
        }

        inline Matrix4(T a00, T a01, T a02, T a03,
                       T a10, T a11, T a12, T a13,
                       T a20, T a21, T a22, T a23,
                       T a30, T a31, T a32, T a33)
        {
            row0 = simd_::Set(a00, a01, a02, a03);
            row1 = simd_::Set(a10, a11, a12, a13);
            row2 = simd_::Set(a20, a21, a22, a23);
            row3 = simd_::Set(a30, a31, a32, a33);
        }

        inline Matrix4(const Matrix4 &mat)
        {
            row0 = mat.row0;
            row1 = mat.row1;
            row2 = mat.row2;
            row3 = mat.row3;
        }

        void CopyTo(float (&mat)[16]) const
        {
            mat[0]  = m00; mat[1]  = m01; mat[2]  = m02; mat[3]  = m03;
            mat[4]  = m10; mat[5]  = m11; mat[6]  = m12; mat[7]  = m13;
            mat[8]  = m20; mat[9]  = m21; mat[10] = m22; mat[11] = m23;
            mat[12] = m30; mat[13] = m31; mat[14] = m32; mat[15] = m33;
        }

        inline Matrix4& operator = (const Matrix4 &mat)
        {
            row0 = mat.row0;
            row1 = mat.row1;
            row2 = mat.row2;
            row3 = mat.row3;
            return *this;
        }

        inline Matrix4& operator += (const Matrix4 &mat)
        {
            row0 = simd_::Add(row0, mat.row0);
            row1 = simd_::Add(row1, mat.row1);
            row2 = simd_::Add(row2, mat.row2);
            row3 = simd_::Add(row3, mat.row3);
            return *this;
        }

        inline Matrix4& operator -= (const Matrix4 &mat)
        {
            row0 = simd_::Sub(row0, mat.row0);
            row1 = simd_::Sub(row1, mat.row1);
            row2 = simd_::Sub(row2, mat.row2);
            row3 = simd_::Sub(row3, mat.row3);
            return *this;
        }

        inline Matrix4& operator *= (T s)
        {
            row0 = simd_::Mul(row0, s);
            row1 = simd_::Mul(row1, s);
            row2 = simd_::Mul(row2, s);
            row3 = simd_::Mul(row3, s);
            return *this;
        }

        inline Matrix4& operator *= (const Matrix4 &mat)
        {
            return *this = *this * mat;
        }

        inline Matrix4& operator /= (T s)
        {
            row0 = simd_::Div(row0, s);
            row1 = simd_::Div(row1, s);
            row2 = simd_::Div(row2, s);
            row3 = simd_::Div(row3, s);
            return *this;
        }

        inline Matrix4 operator - () const
        {
            Matrix4 r;
            r.row0 = simd_::Neg(row0);
            r.row1 = simd_::Neg(row1);
            r.row2 = simd_::Neg(row2);
            r.row3 = simd_::Neg(row3);
            return r;
        }

        inline friend Matrix4 operator + (const Matrix4 &m1, const Matrix4 &m2)
        {
            Matrix4 r;
            r.row0 = simd_::Add(m1.row0, m2.row0);
            r.row1 = simd_::Add(m1.row1, m2.row1);
            r.row2 = simd_::Add(m1.row2, m2.row2);
            r.row3 = simd_::Add(m1.row3, m2.row3);
            return r;
        }

        inline friend Matrix4 operator - (const Matrix4 &m1, const Matrix4 &m2)
        {
            Matrix4 r;
            r.row0 = simd_::Sub(m1.row0, m2.row0);
            r.row1 = simd_::Sub(m1.row1, m2.row1);
            r.row2 = simd_::Sub(m1.row2, m2.row2);
            r.row3 = simd_::Sub(m1.row3, m2.row3);
            return r;
        }

        inline friend Matrix4 operator * (const Matrix4 &m1, T s)
        {
            Matrix4 r;
            r.row0 = simd_::Mul(m1.row0, s);
            r.row1 = simd_::Mul(m1.row1, s);
            r.row2 = simd_::Mul(m1.row2, s);
            r.row3 = simd_::Mul(m1.row3, s);
            return r;
        }

        inline friend vec4 operator * (const Matrix4 &mat, const vec4 &vec)
        {
            vec4 r;
            v4 temp0 = simd_::Dot(mat.row0, vec.v);
            v4 temp1 = simd_::Dot(mat.row1, vec.v);
            v4 temp2 = simd_::Dot(mat.row2, vec.v);
            v4 temp3 = simd_::Dot(mat.row3, vec.v);

    // NOTE: this is for testing both simd optimized and non-simd versions of the method.
    // Non-simd version is faster to do without transpose, but simd is faster with it.
    // This could possibly be abstracted away by introducing a new method:
    //   simd_::MoveMasked(dst, src1, mask2, src2, mask2), or similar.
    #ifdef ROB_USE_SIMD
        if (S)
        {
            r.x = temp0[0];
            r.y = temp1[0];
            r.z = temp2[0];
            r.w = temp3[0];
        }
        else
        {
            simd_::Transpose4(temp0, temp1, temp2, temp3);
            r.v = temp0;

//            r.x = simd_::GetX(temp0);
//            r.y = simd_::GetX(temp1);
//            r.z = simd_::GetX(temp2);
//            r.w = simd_::GetX(temp3);

//            temp0 = simd_::And(temp0, simd::Constants<T, S>::MASK_X);
//            temp1 = simd_::And(temp1, simd::Constants<T, S>::MASK_Y);
//            temp2 = simd_::And(temp2, simd::Constants<T, S>::MASK_Z);
//            temp3 = simd_::And(temp3, simd::Constants<T, S>::MASK_W);

//            temp0 = simd_::Mul(temp0, vec4::UnitX.v);
//            temp1 = simd_::Mul(temp1, vec4::UnitY.v);
//            temp2 = simd_::Mul(temp2, vec4::UnitZ.v);
//            temp3 = simd_::Mul(temp3, vec4::UnitW.v);
//            temp0 = simd_::Add(temp0, temp1);
//            temp2 = simd_::Add(temp2, temp3);
//            r.v = simd_::Add(temp0, temp2);

//            v4 t0011 = simd_::template Shuffle<0, 0, 0, 0>(temp0, temp1);
//            v4 t2233 = simd_::template Shuffle<0, 0, 0, 0>(temp2, temp3);
//            r.v = simd_::template Shuffle<0, 2, 0, 2>(t0011, t2233);
        }
    #else
            r.x = temp0[0];
            r.y = temp1[0];
            r.z = temp2[0];
            r.w = temp3[0];
    #endif // DE_USE_SIMD
            return r;
        }

        inline friend Matrix4 operator * (const Matrix4 &m1, const Matrix4 &m2_)
        {
            Matrix4 m2 = m2_.Transposed();
            Matrix4 res;
            v4 temp0, temp1, temp2, temp3;

            temp0 = simd_::Mul(m1.row0, m2.row0);
            temp1 = simd_::Mul(m1.row0, m2.row1);
            temp2 = simd_::Mul(m1.row0, m2.row2);
            temp3 = simd_::Mul(m1.row0, m2.row3);
            simd_::Transpose4(temp0, temp1, temp2, temp3);
            temp0 = simd_::Add(temp0, temp1);
            temp1 = simd_::Add(temp2, temp3);
            res.row0 = simd_::Add(temp0, temp1);

            temp0 = simd_::Mul(m1.row1, m2.row0);
            temp1 = simd_::Mul(m1.row1, m2.row1);
            temp2 = simd_::Mul(m1.row1, m2.row2);
            temp3 = simd_::Mul(m1.row1, m2.row3);
            simd_::Transpose4(temp0, temp1, temp2, temp3);
            temp0 = simd_::Add(temp0, temp1);
            temp1 = simd_::Add(temp2, temp3);
            res.row1 = simd_::Add(temp0, temp1);

            temp0 = simd_::Mul(m1.row2, m2.row0);
            temp1 = simd_::Mul(m1.row2, m2.row1);
            temp2 = simd_::Mul(m1.row2, m2.row2);
            temp3 = simd_::Mul(m1.row2, m2.row3);
            simd_::Transpose4(temp0, temp1, temp2, temp3);
            temp0 = simd_::Add(temp0, temp1);
            temp1 = simd_::Add(temp2, temp3);
            res.row2 = simd_::Add(temp0, temp1);

            temp0 = simd_::Mul(m1.row3, m2.row0);
            temp1 = simd_::Mul(m1.row3, m2.row1);
            temp2 = simd_::Mul(m1.row3, m2.row2);
            temp3 = simd_::Mul(m1.row3, m2.row3);
            simd_::Transpose4(temp0, temp1, temp2, temp3);
            temp0 = simd_::Add(temp0, temp1);
            temp1 = simd_::Add(temp2, temp3);
            res.row3 = simd_::Add(temp0, temp1);
            return res;
        }

        inline friend Matrix4 operator / (const Matrix4 &m1, T s)
        {
            Matrix4 r;
            r.row0 = simd_::Div(m1.row0, s);
            r.row1 = simd_::Div(m1.row1, s);
            r.row2 = simd_::Div(m1.row2, s);
            r.row3 = simd_::Div(m1.row3, s);
            return r;
        }

        inline void Transpose()
        {
            simd_::Transpose4(row0, row1, row2, row3);
        }

        inline Matrix4 Transposed() const
        {
            Matrix4 r(*this);
            simd_::Transpose4(r.row0, r.row1, r.row2, r.row3);
            return r;
        }



        inline void SetScale(T x, T y, T z)
        {
            row0 = simd_::Set(x, T(0), T(0), T(0));
            row1 = simd_::Set(T(0), y, T(0), T(0));
            row2 = simd_::Set(T(0), T(0), z, T(0));
            row3 = simd_::Set(T(0), T(0), T(0), T(1));
        }

        inline void SetTranslation(T x, T y, T z)
        {
            row0 = simd_::Set(T(1), T(0), T(0), x);
            row1 = simd_::Set(T(0), T(1), T(0), y);
            row2 = simd_::Set(T(0), T(0), T(1), z);
            row3 = simd_::Set(T(0), T(0), T(0), T(1));
        }

    };

    template <class T, size_t S>
    Matrix4<T, S> Matrix4<T, S>::Identity(1, 0, 0, 0,
                                          0, 1, 0, 0,
                                          0, 0, 1, 0,
                                          0, 0, 0, 1);

    template <class T, size_t S>
    Matrix4<T, S> Matrix4<T, S>::Zero(0);

} // rob

#endif // H_ROB_MATH_MATRIX4_H

