
#ifndef H_ROB_MATH_FUNCTIONS_H
#define H_ROB_MATH_FUNCTIONS_H

#include "Constants.h"
#include <cmath>

namespace rob
{

    template <class V, class T>
    inline void ClampVectorLength(V &v, const T len)
    {
        const float l = v.Length();
        if (l > len)
        {
            v /= l;
            v *= len;
        }
    }

    template <class V, class T>
    inline V ClampedVectorLength(const V &v, const T len)
    {
        V cv(v);
        ClampVectorLength(cv, len);
        return cv;
    }

    template <class T>
    inline T Abs(T a)
    {
        return a < T(0) ? -a : a;
    }

    template <class T>
    inline T Max(T a, T b)
    {
        return a > b ? a : b;
    }

    template <class T>
    inline T Min(T a, T b)
    {
        return a < b ? a : b;
    }

    template <class T>
    inline T Clamp(T x, T a, T b)
    {
        return Max(a, Min(x, b));
    }

    template <class T>
    inline T Sqrt(T x)
    {
        return std::sqrt(x);
    }

    template <class T>
    inline T Log2(T x)
    {
        return std::log(x);
    }

    template <class T>
    inline T Log10(T x)
    {
        return std::log10(x);
    }

    template <class T>
    inline T Pow(T x, T y)
    {
        return std::pow(x, y);
    }

    template <class T>
    inline T Sin(T x)
    {
        return std::sin(x);
    }

    template <class T>
    inline T Cos(T x)
    {
        return std::cos(x);
    }

    template <class T>
    inline void SinCos(T x, T &s, T &c)
    {
        s = std::sin(x);
        c = std::cos(x);
    }

namespace fast_sin_cos1
{
    static const int SIN_COS_LUT_RES = 512;
    static const int SIN_COS_LUT_MASK = SIN_COS_LUT_RES - 1;

    static const double _2_PI_OVER_LUT_RES  = 2 * PI_d / SIN_COS_LUT_RES;
    static const double LUT_RES_OVER_2_PI 	= SIN_COS_LUT_RES / (PI_d * 2);
    static const float _2_PI_OVER_LUT_RESf  = 2 * PI_f / SIN_COS_LUT_RES;
    static const float LUT_RES_OVER_2_PIf 	= SIN_COS_LUT_RES / (PI_f * 2);

    extern double g_sinLUT[SIN_COS_LUT_RES];
    extern double g_cosLUT[SIN_COS_LUT_RES];
    extern float g_sinLUTf[SIN_COS_LUT_RES];
    extern float g_cosLUTf[SIN_COS_LUT_RES];

    inline double sin(double angle)
    {
        int index = angle * LUT_RES_OVER_2_PI;
        double h = angle - index * _2_PI_OVER_LUT_RES;
        index &= SIN_COS_LUT_MASK;
        return g_sinLUT[index] + h * g_cosLUT[index];
    }

    inline double cos(double angle)
    {
        int index = angle * LUT_RES_OVER_2_PI;
        double h = angle - index * _2_PI_OVER_LUT_RES;
        index &= SIN_COS_LUT_MASK;
        return g_cosLUT[index] - h * g_sinLUT[index];
    }

    inline void sincos(double angle, double &s, double &c)
    {
        int index = angle * LUT_RES_OVER_2_PI;
        double h = angle - index * _2_PI_OVER_LUT_RES;
        index &= SIN_COS_LUT_MASK;
        s = g_sinLUT[index] + h * g_cosLUT[index];
        c = g_cosLUT[index] - h * g_sinLUT[index];
    }

    inline float sin(float angle)
    {
        int index = angle * LUT_RES_OVER_2_PIf;
        float h = angle - index * _2_PI_OVER_LUT_RESf;
        index &= SIN_COS_LUT_MASK;
        return g_sinLUTf[index] + h * g_cosLUTf[index];
    }

    inline float cos(float angle)
    {
        int index = angle * LUT_RES_OVER_2_PIf;
        float h = angle - index * _2_PI_OVER_LUT_RESf;
        index &= SIN_COS_LUT_MASK;
        return g_cosLUTf[index] - h * g_sinLUTf[index];
    }

    inline void sincos(float angle, float &s, float &c)
    {
        int index = angle * LUT_RES_OVER_2_PIf;
        float h = angle - index * _2_PI_OVER_LUT_RESf;
        index &= SIN_COS_LUT_MASK;
        s = g_sinLUTf[index] + h * g_cosLUTf[index];
        c = g_cosLUTf[index] - h * g_sinLUTf[index];
    }
} // fast_sin_cos1

namespace fast_sin_cos2
{

    extern const double g_sinCosLUT[];
    extern const float g_sinCosLUTf[];

    static const double _32_OVER_PI = 32.0 / PI;
	static const double _PI_OVER_32 = PI / 32.0;
	static const double _INV_FACT_2 = 1.0 / 2.0;
	static const double _INV_FACT_3 = 1.0 / 6.0;
	static const double _INV_FACT_4 = 1.0 / 24.0;
	static const double _INV_FACT_5 = 1.0 / 120.0;
	static const double _INV_FACT_6 = 1.0 / 720.0;
	static const double _INV_FACT_7 = 1.0 / 5040.0;
	static const double _INV_FACT_8 = 1.0 / 40320.0;
	static const double _INV_FACT_9 = 1.0 / 362880.0;

    static const float _32_OVER_PIf = 32.0f / PI;
	static const float _PI_OVER_32f = PI / 32.0f;
	static const float _INV_FACT_2f = 1.0f / 2.0f;
	static const float _INV_FACT_3f = 1.0f / 6.0f;
	static const float _INV_FACT_4f = 1.0f / 24.0f;
	static const float _INV_FACT_5f = 1.0f / 120.0f;
	static const float _INV_FACT_6f = 1.0f / 720.0f;
	static const float _INV_FACT_7f = 1.0f / 5040.0f;
	static const float _INV_FACT_8f = 1.0f / 40320.0f;
	static const float _INV_FACT_9f = 1.0f / 362880.0f;

    inline double sin(double x)
	{
        const double scaledAngle = x * _32_OVER_PI;

        int index = __builtin_ia32_cvtsd2si(__builtin_ia32_loadupd(&scaledAngle));
        //int index = int(scaledAngle);
        const double a = index * _PI_OVER_32;
        index &= 0x0000003F;
        index <<= 1;
        const double d = x - a;
        const double d2 = d * d;
        const double t1 =	  (1.0 + d2 * (d2 * (_INV_FACT_4 + d2 * (d2 * _INV_FACT_8 - _INV_FACT_6)) - _INV_FACT_2));
        const double t2 = d * (1.0 + d2 * (d2 * (_INV_FACT_5 + d2 * (d2 * _INV_FACT_9 - _INV_FACT_7)) - _INV_FACT_3));
        return  g_sinCosLUT[index + 0] * t1 + g_sinCosLUT[index + 1] * t2;
    }

    inline double cos(double x)
	{
        const double scaledAngle = x * _32_OVER_PI;

        int index = __builtin_ia32_cvtsd2si(__builtin_ia32_loadupd(&scaledAngle));
        //int index = int(scaledAngle);
        const double a = index * _PI_OVER_32;
        index += 16;
        index &= 0x0000003F;
        index <<= 1;
        const double d = x - a;
        const double d2 = d * d;
        const double t1 =	  (1.0 + d2 * (d2 * (_INV_FACT_4 + d2 * (d2 * _INV_FACT_8 - _INV_FACT_6)) - _INV_FACT_2));
        const double t2 = d * (1.0 + d2 * (d2 * (_INV_FACT_5 + d2 * (d2 * _INV_FACT_9 - _INV_FACT_7)) - _INV_FACT_3));
        return  g_sinCosLUT[index + 0] * t1 + g_sinCosLUT[index + 1] * t2;
    }

    inline void sincos(double x, double &s, double &c)
	{
        const double scaledAngle = x * _32_OVER_PI;

        int index = __builtin_ia32_cvtsd2si(__builtin_ia32_loadupd(&scaledAngle));
        //int index = int(scaledAngle);
        const double a = index * _PI_OVER_32f;
        int index_c = index + 16;
        index = (index & 0x0000003F) << 1;
        index_c = (index_c & 0x0000003F) << 1;
        const double d = x - a;
        const double d2 = d * d;
        const double t1 =	  (1.0 + d2 * (d2 * (_INV_FACT_4 + d2 * (d2 * _INV_FACT_8 - _INV_FACT_6)) - _INV_FACT_2));
        const double t2 = d * (1.0 + d2 * (d2 * (_INV_FACT_5 + d2 * (d2 * _INV_FACT_9 - _INV_FACT_7)) - _INV_FACT_3));
        s = g_sinCosLUT[index + 0] * t1 + g_sinCosLUT[index + 1] * t2;
        c = g_sinCosLUT[index_c + 0] * t1 + g_sinCosLUT[index_c + 1] * t2;
    }

    inline float sin(float x)
	{
        const float scaledAngle = x * _32_OVER_PIf;

        //int index = __builtin_ia32_cvtss2si(__builtin_ia32_loadups(&scaledAngle));
        int index = int(scaledAngle);
        //int index = std::floor(scaledAngle + 0.5f);
        const float a = index * _PI_OVER_32f;
        index &= 0x0000003F;
        index <<= 1;
        const float d = x - a;
        const float d2 = d * d;
        const float t1 =	 (1.0f + d2 * (d2 * (_INV_FACT_4f + d2 * (d2 * _INV_FACT_8f - _INV_FACT_6f)) - _INV_FACT_2f));
        const float t2 = d * (1.0f + d2 * (d2 * (_INV_FACT_5f + d2 * (d2 * _INV_FACT_9f - _INV_FACT_7f)) - _INV_FACT_3f));
        return g_sinCosLUTf[index + 0] * t1 + g_sinCosLUTf[index + 1] * t2;
    }

    inline float cos(float x)
	{
        const float scaledAngle = x * _32_OVER_PIf;

        //int index = __builtin_ia32_cvtss2si(__builtin_ia32_loadups(&scaledAngle));
        int index = int(scaledAngle);
        const float a = index * _PI_OVER_32f;
        index += 16;
        index &= 0x0000003F;
        index <<= 1;
        const float d = x - a;
        const float d2 = d * d;
        const float t1 =	 (1.0f + d2 * (d2 * (_INV_FACT_4f + d2 * (d2 * _INV_FACT_8f - _INV_FACT_6f)) - _INV_FACT_2f));
        const float t2 = d * (1.0f + d2 * (d2 * (_INV_FACT_5f + d2 * (d2 * _INV_FACT_9f - _INV_FACT_7f)) - _INV_FACT_3f));
        return g_sinCosLUTf[index + 0] * t1 + g_sinCosLUTf[index + 1] * t2;
    }

    inline void sincos(float x, float &s, float &c)
	{
        const float scaledAngle = x * _32_OVER_PIf;

        //int index = __builtin_ia32_cvtss2si(__builtin_ia32_loadups(&scaledAngle));
        int index = int(scaledAngle);
        const float a = index * _PI_OVER_32f;
        int index_c = index + 16;
        index = (index & 0x0000003F) << 1;
        index_c = (index_c & 0x0000003F) << 1;
        const float d = x - a;
        const float d2 = d * d;
        const float t1 =	 (1.0f + d2 * (d2 * (_INV_FACT_4f + d2 * (d2 * _INV_FACT_8f - _INV_FACT_6f)) - _INV_FACT_2f));
        const float t2 = d * (1.0f + d2 * (d2 * (_INV_FACT_5f + d2 * (d2 * _INV_FACT_9f - _INV_FACT_7f)) - _INV_FACT_3f));
        s = g_sinCosLUTf[index + 0] * t1 + g_sinCosLUTf[index + 1] * t2;
        c = g_sinCosLUTf[index_c + 0] * t1 + g_sinCosLUTf[index_c + 1] * t2;
    }
} // fast_sin_cos2

    template <class T>
    inline T FastSin(T x)
    {
        return fast_sin_cos1::sin(x);
    }

    template <class T>
    inline T FastCos(T x)
    {
        return fast_sin_cos1::cos(x);
    }

    template <class T>
    inline void FastSinCos(T x, T &s, T &c)
    {
        fast_sin_cos1::sincos(x, s, c);
    }

    template <class T>
    inline T FastSin2(T x)
    {
        return fast_sin_cos2::sin(x);
    }

    template <class T>
    inline T FastCos2(T x)
    {
        return fast_sin_cos2::cos(x);
    }

    template <class T>
    inline void FastSinCos2(T x, T &s, T &c)
    {
        fast_sin_cos2::sincos(x, s, c);
    }

} // rob

#endif // H_ROB_MATH_FUNCTIONS_H

