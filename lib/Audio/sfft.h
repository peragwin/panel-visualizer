#pragma once

#include "WindowBuffer.h"
#include "fft.h"
#include "fast_log.h"
#include <vector>

namespace vuzicaudio
{
    using namespace std;

    class SFFT
    {
    private:
        WindowBuffer buffer;
        fft_config_t *fft_config;
        vector<float> fft_input;
        float *window;

        static void init_blackman_window(float *window, int size)
        {
            for (int i = 0; i < size; i++)
            {
                // blackman-harris window
                float c1 = cosf(2.0 * PI * (float)i / (float)(size - 1));
                float c2 = cosf(4.0 * PI * (float)i / (float)(size - 1));
                float c3 = cosf(6.0 * PI * (float)i / (float)(size - 1));
                window[i] = 0.35875 - 0.48829 * c1 + 0.14128 * c2 - 0.01168 * c3;
            }
        }

        // removes the dc component by subtracting the average
        static void remove_dc_component(vector<float> &input)
        {
            float sum = 0.0;
            for (auto v : input)
            {
                sum += v;
            }
            sum /= float(input.size());

            for (auto &v : input)
            {
                v -= sum;
            }
        }

        void apply_window(vector<float> &input)
        {
            for (size_t i = 0; i < input.size(); i++)
            {
                input[i] *= window[i];
            }
        }

        static void apply_log_scale(vector<float> &input)
        {
            for (size_t i = 0; i < input.size(); i += 2)
            {
                auto r = input[i];
                auto c = input[i + 1];
                auto x = log2_2521(1.0 + r * r + c * c);
                input[i / 2] = x;
            }
        }

    public:
        SFFT(size_t buffer_size, size_t fft_size) : buffer(buffer_size), fft_input(fft_size)
        {
            fft_config = fft_init(fft_size, FFT_REAL, FFT_FORWARD, fft_input.data(), (float *)-1);
            window = new float[fft_size];
            init_blackman_window(window, fft_size);
        }

        void apply(vector<float> &input, vector<float> &output)
        {
            buffer.push(input);
            buffer.get(fft_input);
            remove_dc_component(fft_input);
            apply_window(fft_input);
            fft_config->output = output.data();
            fft_execute(fft_config);
            apply_log_scale(output);
        }

        ~SFFT()
        {
            fft_destroy(fft_config);
            delete[] window;
        }
    };
};
