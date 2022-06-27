#pragma once

#include "ArduinoJson.h"
#include "Configlets.h"
#include <vector>

namespace vuzicaudio
{

    class FilterParams
    {
    private:
        float _a;
        float _b;
        float _tau;
        float _gain;

    public:
        static const char *dtype() { return "FilterParams"; }

        FilterParams();
        FilterParams(float tau, float gain) { set(tau, gain); }

        void set(float tau, float gain)
        {
            _tau = tau;
            _gain = gain;
            if (tau == 0.0)
            {
                _a = gain;
                _b = 0.0;
                return;
            }
            if (tau < 1.0)
                tau = 1.0;
            auto b = 0.5 * powf(2.0, (tau - 1.0) / tau);
            auto a = 1.0 - b;
            _a = a * gain;
            _b = b * fabs(gain);
        }

        void get(float &a, float &b)
        {
            a = _a;
            b = _b;
        }

        void toJson(JsonObject &json) {}
    };

    struct FilterValues
    {
        std::vector<float> values;

        FilterValues(size_t size) : values(size, 0.0) {}

        void apply(std::vector<float> &input, FilterParams &params)
        {
            float a, b;
            params.get(a, b);
            for (size_t i = 0; i < input.size(); ++i)
            {
                values[i] = a * input[i] + b * values[i];
            }
        }

        inline float operator[](size_t i) { return values[i]; }
    };

};
