#pragma once

#include <FrequencySensor.h>
#include "params.h"
#include <stddef.h>

class WarpController
{
private:
    size_t size_;
    float *warp_;
    float *warp_d_;
    float *warp_i_;

public:
    WarpController(size_t size) : size_(size)
    {
        warp_ = new float[size];
        warp_d_ = new float[size];
        warp_i_ = new float[size];
        for (size_t i = 0; i < size; i++)
        {
            warp_[i] = 0.;
            warp_d_[i] = 0.;
            warp_i_[i] = 0.;
        }
    }

    void computeWarp(float *input, float pk, float pd, float pi)
    {
        float e, k, dk, ik;

        for (size_t i = 0; i < size_; i++)
        {
            e = input[i] - warp_[i];
            k = pk * 0.1 * e;
            dk = pd * warp_d_[i];
            ik = pi * e + warp_i_[i];

            auto wp = warp_[i];
            warp_[i] = k + dk + ik;
            warp_d_[i] = warp_[i] - wp;
            warp_i_[i] = ik;
        }

        // Serial.printf("warp0: %0.2f %0.2f %0.2f %0.2f\r\n", input[0], warp_[0], warp_d_[0], warp_i_[0]);
    }

    float getWarp(size_t idx) { return warp_[idx]; }
};
