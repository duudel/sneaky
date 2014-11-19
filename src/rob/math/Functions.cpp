
#include "Functions.h"

namespace rob
{

namespace fast_sin_cos1
{
    double g_sinLUT[SIN_COS_LUT_RES];
    double g_cosLUT[SIN_COS_LUT_RES];
    float g_sinLUTf[SIN_COS_LUT_RES];
    float g_cosLUTf[SIN_COS_LUT_RES];

    static bool initSinLUT();
    static bool initCosLUT();
    static const bool g_initSinLUT = initSinLUT();
    static const bool g_initCosLUT = initCosLUT();

    static bool initSinLUT()
    {
        for (int i = 0; i < SIN_COS_LUT_RES; i++)
        {
            g_sinLUT[i] = std::sin(i * _2_PI_OVER_LUT_RES);
            g_sinLUTf[i] = std::sin(i * _2_PI_OVER_LUT_RESf);
        }
        return true;
    }

    static bool initCosLUT()
    {
        for (int i = 0; i < SIN_COS_LUT_RES; i++)
        {
            g_cosLUT[i] = std::cos(i * _2_PI_OVER_LUT_RES);
            g_cosLUTf[i] = std::cos(i * _2_PI_OVER_LUT_RESf);
        }
        return true;
    }
} // fast_sin_cos1

namespace fast_sin_cos2
{
    #define SIN_COS_LUT(x) std::sin((x) * PI_d / 32.0), std::cos((x) * PI_d / 32.0)
    #define SIN_COS_LUT4(x) SIN_COS_LUT(x), SIN_COS_LUT(x + 1.0), SIN_COS_LUT(x + 2.0), SIN_COS_LUT(x + 3.0)
    #define SIN_COS_LUT16(x) SIN_COS_LUT4(x), SIN_COS_LUT4(x + 4.0), SIN_COS_LUT4(x + 8.0), SIN_COS_LUT4(x + 12.0)

    const double g_sinCosLUT[] = {
        SIN_COS_LUT16(0.0), SIN_COS_LUT16(16.0),
        SIN_COS_LUT16(32.0), SIN_COS_LUT16(48.0)
    };
    const float g_sinCosLUTf[] = {
        SIN_COS_LUT16(0.0f), SIN_COS_LUT16(16.0f),
        SIN_COS_LUT16(32.0f), SIN_COS_LUT16(48.0f)
    };

    #undef SIN_COS_LUT
    #undef SIN_COS_LUT4
    #undef SIN_COS_LUT16
} // fast_sin_cos2

} // rob
