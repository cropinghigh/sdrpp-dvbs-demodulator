#include "dvbs2_pll.h"

namespace dsp {
    namespace dvbs2 {
        void S2PLLBlock::init(stream<complex_t>* in, float loop_bw) {
            float damping = sqrtf(2.0f) / 2.0f;
            float denom = (1.0 + 2.0 * damping * loop_bw + loop_bw * loop_bw);

            alpha = (4 * damping * loop_bw) / denom;
            beta = (4 * loop_bw * loop_bw) / denom;
            base_type::init(in);
        }

        void S2PLLBlock::reset() {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            freq = 0;
            phase = 0;
            base_type::tempStart();
        }

        void S2PLLBlock::setParams(float loop_bw) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            float damping = sqrtf(2.0f) / 2.0f;
            float denom = (1.0 + 2.0 * damping * loop_bw + loop_bw * loop_bw);

            alpha = (4 * damping * loop_bw) / denom;
            beta = (4 * loop_bw * loop_bw) / denom;
            base_type::tempStart();
        }

        int S2PLLBlock::process(int count, complex_t* in, complex_t* out) {
            float errorsum = 0;
            for (int i = 0; i < (frame_slot_count + 1) * 90 + pilot_cnt * 36; i++) {
                tmp_val = in[i] * complex_t{cosf(-phase), sinf(-phase)}; //DISABLED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                // tmp_val = in[i];

                float error = 0;
                if (i >= 90) {
                    constellation->demod_soft_lut(tmp_val, nullptr, &error);
                    out[i] = tmp_val;
                } else {
                    if (i < 26) // Use known symbols for SOF
                        error = (tmp_val * sof.symbols[i].conj()).phase();
                    else // And also use known Modcod and config to synchronize onto the PLS
                        error = (tmp_val * pls.symbols[pls_code][i - 26].conj()).phase();

                    // We're done, convert to proper 45 degs BPSK
                    out[i] = (i & 1) ? complex_t{-tmp_val.re, tmp_val.im} : complex_t{tmp_val.im, tmp_val.re};
                }
                errorsum = error;

                // Compute new freq and phase.
                freq += beta * error;
                phase += freq + alpha * error;

                // Wrap phase
                while (phase > (2 * M_PI))
                    phase -= 2 * M_PI;
                while (phase < (-2 * M_PI))
                    phase += 2 * M_PI;

                // Clamp freq
                if (freq > 0.25)
                    freq = 0.25;
                if (freq < -0.25)
                    freq = -0.25;
            }
            error = errorsum / ((float)(frame_slot_count + 1) * 90 + pilot_cnt * 36);

            return (frame_slot_count + 1) * 90 + pilot_cnt * 36;
        }
    }
}
