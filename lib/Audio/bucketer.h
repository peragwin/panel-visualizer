#pragma once

#include <Arduino.h>
#include <vector>
#include <cmath>

namespace vuzicaudio
{
    using namespace std;

    class Bucketer
    {
        vector<uint16_t> indices;

        static float to_log_scale(float val) { return log2f(1.0 + val); }
        static float from_log_scale(float val) { return exp2f(val) - 1.0; }

    public:
        Bucketer(uint16_t input_size, uint8_t output_size, float f_min, float f_max) : indices(output_size + 1, 0)
        {
            indices[0] = 0;
            indices[output_size] = input_size;

            auto s_min = to_log_scale(f_min);
            auto s_max = to_log_scale(f_max);
            auto buckets = float(output_size);
            auto insize = float(input_size);
            auto space = (s_max - s_min) / buckets;
            auto delta = (s_max - s_min) / insize;
            int last_idx = 0;
            float offset = 0.0;

            for (int i = 1; i < output_size; i++)
            {
                auto adj = space - delta * offset / buckets;
                auto v = from_log_scale(float(i) * adj + s_min + offset * delta);
                auto idx = int(ceilf(insize * v / f_max));
                if (idx <= last_idx)
                {
                    idx = last_idx + 1;
                    offset += 1.0;
                }
                if (idx >= input_size)
                {
                    idx = input_size - 1;
                }
                indices[i] = idx;
                last_idx = idx;
            }

            Serial.print("Bucket indices: ");
            for (auto v : indices)
            {
                Serial.printf("%d, ", v);
            }
            Serial.println();
        }

        void apply(vector<float> &input, vector<float> &buckets)
        {
            for (size_t i = 0; i < buckets.size(); i++)
            {
                auto start = indices[i];
                auto stop = indices[i + 1];
                float sum = 0.0;
                for (size_t j = start; j < stop; j++)
                {
                    sum += input[j];
                }
                buckets[i] = sum / float(stop - start);
            }
        }
    };
};
