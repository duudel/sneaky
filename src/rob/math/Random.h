
#ifndef H_ROB_RANDOM_H
#define H_ROB_RANDOM_H

#include "../Types.h"
#include "../time/Time.h"

#include <random>

namespace rob
{

    class Random
    {
    public:
        Random()
            : m_generator()
        {
            Seed(GetTicks());
        }

        void Seed(uint32_t seed)
        {
            m_generator.seed(seed);
        }

        int GetInt()
        {
            std::uniform_int_distribution<int> dis;
            return dis(m_generator);
        }

        int GetInt(int a, int b)
        {
            std::uniform_int_distribution<int> dis(a, b);
            return dis(m_generator);
        }

        double GetReal()
        {
            std::uniform_real_distribution<double> dis;
            return dis(m_generator);
        }

        double GetReal(double a, double b)
        {
            std::uniform_real_distribution<double> dis(a, b);
            return dis(m_generator);
        }

        vec2f GetDirection()
        {
            float x = GetReal(0.0f, 2.0f*PI_f);
            return vec2f(Cos(x), Sin(x));
        }

    private:
        std::mt19937 m_generator;
    };

} // rob

#endif // H_ROB_RANDOM_H

