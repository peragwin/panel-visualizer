#pragma once

#include "filter.h"
#include "fast_log.h"
#include "Configlets.h"
#include <vector>

namespace vuzicaudio
{
    using namespace std;
    using Configlets::VariableGroup;

    struct GainControlParams
    {
        shared_ptr<FilterParams> filterParams;
        shared_ptr<float> kp;
        shared_ptr<float> ki;
        shared_ptr<float> kd;
        float pregain;

        static const char *dtype() { return "GainControlParams"; }

        GainControlParams() {}
        GainControlParams(string prefix, VariableGroup &vg, FilterParams fp, float _kp, float _ki, float _kd, float pre = 1.0)
            : filterParams(make_shared<FilterParams>(fp)),
              kp(make_shared<float>(_kp)),
              ki(make_shared<float>(_ki)),
              kd(make_shared<float>(_kd)),
              pregain(pre)
        {
            vg.newCustomValue((prefix + ".filter").c_str(), filterParams);
            vg.newFloatValue((prefix + ".kp").c_str(), kp);
            vg.newFloatValue((prefix + ".kd").c_str(), kd);
            vg.newFloatValue((prefix + ".ki").c_str(), ki);
        }

        void toJson(JsonObject &json) {}
    };

    class GainController
    {
    private:
        FilterValues _filterValues;
        std::vector<float> _values;
        std::vector<float> _ierror;
        std::vector<float> _derror;

        static float error_func(float e)
        {
            e += 0.001;
            return -(e < 1.0 ? log2_2521(e) : e);
        }

    public:
        GainController(size_t size)
            : _filterValues(size), _values(size, 0.0), _ierror(size, 0.0), _derror(size, 0.0) {}

        inline float operator[](size_t i) { return _values[i]; }

        void apply(std::vector<float> &input, GainControlParams &gp)
        {
            for (size_t i = 0; i < input.size(); ++i)
            {
                input[i] *= _values[i] * gp.pregain;
            }

            _filterValues.apply(input, *gp.filterParams);
            auto kp = *gp.kp;
            auto kd = *gp.kd;
            auto ki = *gp.ki;

            for (size_t i = 0; i < input.size(); ++i)
            {
                auto e = GainController::error_func(_filterValues[i]);
                if (isnanf(e) || isinff(e))
                {
                    e = 0.0;
                }
                _ierror[i] += e;
                auto v = _values[i] + kp * e + ki * _ierror[i] + kd * (_derror[i] - e);
                _derror[i] = e;
                if (v > 1e3)
                    v = 1e3;
                if (v < 1e-3)
                    v = 1e-3;
                _values[i] = v;
            }
        }
    };

};
