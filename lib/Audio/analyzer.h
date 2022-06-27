#pragma once

#include "Configlets.h"
#include "pregain.h"
#include "bucketer.h"
#include "sfft.h"
#include <memory>

namespace vuzicaudio
{
    using namespace std;

    struct AnalyzerParams
    {
        GainControlParams pregain;
        shared_ptr<float> preemphasis;
        GainControlParams normalizer;
        shared_ptr<FilterParams> amp_filter;
        shared_ptr<FilterParams> amp_feedback;
        shared_ptr<FilterParams> diff_filter;
        shared_ptr<FilterParams> diff_feedback;
        shared_ptr<float> energy_gain;
        shared_ptr<float> energy_sync;

        AnalyzerParams(Registry &reg)
        {
            auto vg = Configlets::VariableGroup("audio", reg);

            pregain = GainControlParams(
                "input_gain", vg,
                FilterParams(400.0, 1.0),
                0.1, 0.0, 0.0);

            preemphasis = make_shared<float>(1.0);
            vg.newFloatValue("preemphasis", preemphasis, 1.0, 16.0, 0.1);

            normalizer = GainControlParams(
                "normalizer", vg,
                FilterParams(400.0, 1.0),
                0.001, 0.0, 0.0);

            amp_filter = make_shared<FilterParams>(4.0, 1.0);
            amp_feedback = make_shared<FilterParams>(100.0, -1.0);
            vg.newCustomValue("amp_filter", amp_filter);
            vg.newCustomValue("amp_feedback", amp_feedback);

            diff_filter = make_shared<FilterParams>(2.0, 1.0);
            diff_feedback = make_shared<FilterParams>(100.0, -0.25);
            vg.newCustomValue("diff_filter", diff_filter);
            vg.newCustomValue("diff_feedback", diff_feedback);

            energy_gain = make_shared<float>(0.1);
            vg.newFloatValue("enegry_gain", energy_gain, 0.001, 1.0, 0.001);
            energy_sync = make_shared<float>(0.01);
            vg.newFloatValue("energy_sync", energy_sync, 0.001, 0.1, 0.001);
        }
    };

    struct AnalyzerOutput
    {
        vector<float> value;
        vector<float> diff;
        vector<float> energy;

        AnalyzerOutput(size_t size)
            : value(size), diff(size), energy(size) {}
    };

    class Analyzer
    {
    private:
        PreGain pregain;
        SFFT sfft;
        Bucketer bucketer;
        vector<float> fft_buffer;
        vector<float> freq_bins;

        AnalyzerParams params;
        GainController normalizer;
        FilterValues amp_filter;
        FilterValues amp_feedback;
        FilterValues diff_filter;
        FilterValues diff_feedback;

        AnalyzerOutput output;

        bool _have_values = false;
        int _frame_count = 0;

        static bool check_for_nan(vector<float> &input)
        {
            for (auto x : input)
            {
                if (isnanf(x))
                    return true;
            }
            return false;
        }

        void apply_preemphasis(vector<float> &input)
        {
            float incr = (*params.preemphasis - 1.0) / float(input.size());
            for (size_t i = 0; i < input.size(); i++)
            {
                input[i] *= 1.0 + float(i) * incr;
            }
        }

        void apply_filters(vector<float> &input)
        {
            float diff_input[input.size()];
            memcpy(diff_input, amp_filter.values.data(), sizeof(float) * input.size());
            amp_filter.apply(input, *params.amp_filter);
            amp_feedback.apply(input, *params.amp_feedback);
            for (size_t i = 0; i < input.size(); i++)
            {
                diff_input[i] = amp_filter[i] - diff_input[i];
            }
            diff_filter.apply(input, *params.diff_filter);
            diff_feedback.apply(input, *params.diff_feedback);
        }

        void make_output()
        {
            auto size = output.value.size();
            float diff;
            auto sync = (1.0 - *params.energy_sync);
            auto egain = *params.energy_gain;
            for (size_t i = 0; i < size; i++)
            {
                output.value[i] = amp_filter[i] + amp_feedback[i];
                output.diff[i] = diff = diff_filter[i] + diff_feedback[i];
                output.energy[i] = sync * (output.energy[i] + egain * diff);
            }
        }

    public:
        Analyzer(size_t buffer_size, size_t fft_size, size_t bins, Registry &reg)
            : sfft(buffer_size, fft_size),
              bucketer(fft_size / 2, bins, 32.0, 16000.0),
              fft_buffer(fft_size),
              freq_bins(bins),
              params(reg),
              normalizer(bins),
              amp_filter(bins),
              amp_feedback(bins),
              diff_filter(bins),
              diff_feedback(bins),
              output(bins)
        {
        }

        void process(vector<float> &input)
        {
            pregain.apply(input, params.pregain);
            sfft.apply(input, fft_buffer);
            bucketer.apply(fft_buffer, freq_bins);
            if (check_for_nan(freq_bins))
            {
                Serial.println("nan frame");
                return;
            }
            float bins[freq_bins.size()];
            memcpy(bins, freq_bins.data(), sizeof(float) * freq_bins.size());

            apply_preemphasis(freq_bins);
            normalizer.apply(freq_bins, params.normalizer);
            apply_filters(freq_bins);

            make_output();

            _have_values = true;
            _frame_count++;
            if (_frame_count % 128 == 0)
            {
                Serial.println("---");

                Serial.printf("Pregain: %f\r\n", pregain.value());

                Serial.print("Freq bins: ");
                for (size_t i = 0; i < freq_bins.size(); i++)
                {
                    Serial.printf("%0.4f, ", bins[i]);
                }
                Serial.println();

                Serial.print("Normalizer: ");
                for (size_t i = 0; i < freq_bins.size(); i++)
                {
                    Serial.printf("%0.4f, ", normalizer[i]);
                }
                Serial.println();

                Serial.print("Amp values: ");
                for (size_t i = 0; i < freq_bins.size(); i++)
                {
                    Serial.printf("%0.4f, ", output.value[i]);
                }
                Serial.println();
            }
        };

        bool ready() const { return _have_values; }

        AnalyzerOutput &getOutput() { return output; }
    };
};
