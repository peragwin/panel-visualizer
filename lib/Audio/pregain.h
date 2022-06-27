#pragma once

#include "gaincontrol.h"
#include <vector>

namespace vuzicaudio
{
    class PreGain
    {
    private:
        GainController gc;
        std::vector<float> _gain;

    public:
        PreGain() : gc(1), _gain(1, 1.0) {}

        void apply(std::vector<float> &input, GainControlParams &gp)
        {
            float sum = 0.0;
            for (auto &x : input)
            {
                sum += x * x;
            }
            _gain[0] = sqrtf(sum / input.size());
            gc.apply(_gain, gp);
            auto scale = gc[0];
            for (auto &x : input)
            {
                x *= scale;
            }
        }

        float value() { return _gain[0]; }
    };
};
