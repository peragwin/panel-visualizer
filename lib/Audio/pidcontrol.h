#pragma once

#include <vector>

namespace vuzicaudio
{

    class PIDParams
    {
    private:
        float _kp;
        float _kd;
        float _ki;

    public:
        static const char *dtype() { return "PIDParams"; }

        PIDParams();
        PIDParams(float kp, float kd, float ki)
            : _kp(kp), _kd(kd), _ki(ki) {}

        void toJson(JsonObject &json) {}
    };

    class PIDController
    {
    private:
        std::vector<float> _values;
        std::vector<float> _error;

        static float measure_error(float e)
        {
            e < 1.0 ? 1.0 / ()
        }

    public:
        PIDController(size_t size) : _values(size, 0.0), _error(size, 0.0) {}

        void apply(std::vector<float> input, float target)
        {
        }
    };

};
