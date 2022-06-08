#pragma once
#include <Arduino.h>

namespace vectormath
{
    float dot(float *u, float *v, int len)
    {
        float s = 0;
        for (int i = 0; i < len; i++)
        {
            s += u[i] * v[i];
        }
        return s;
    }

    void normalize(float *u, int len)
    {
        auto norm = 1.0 / sqrt(dot(u, u, len));
        for (int i = 0; i < len; i++)
        {
            u[i] *= norm;
        }
    }

    void randomize(float *u, int len)
    {
        for (int i = 0; i < len; i++)
        {
            u[i] = 2.0 * random() / (float)RAND_MAX - 1.0;
        }
    }
};

template <int xlen, int ylen>
void construct_audio_coefficients(float coef[xlen][ylen])
{
    // first basis vector
    vectormath::randomize(coef[0], ylen);
    vectormath::normalize(coef[0], ylen);

    for (int j = 1; j < xlen; j++)
    {
        auto u = coef[j];
        vectormath::randomize(u, ylen);

        // gram-schmidt
        for (int i = 0; i < j; i++)
        {
            auto v = coef[i];
            auto c = vectormath::dot(u, v, ylen);
            for (int k = 0; k < ylen; k++)
            {
                u[k] -= c * v[k];
            }
        }

        vectormath::normalize(u, ylen);
    }
}
