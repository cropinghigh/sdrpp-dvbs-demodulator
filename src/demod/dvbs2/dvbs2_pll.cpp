#include "dvbs2_pll.h"

namespace dsp {
    namespace dvbs2 {
        void S2PLLBlock::init(stream<complex_t>* in, float loop_bw, s2_sof* esof, s2_plscodes* epls, dvbs2::S2Scrambling* edescrambler) {
            sof = esof;
            pls = epls;
            descrambler = edescrambler;
            float alpha, beta;
            loop::PhaseControlLoop<float>::criticallyDamped(loop_bw, alpha, beta);
            pcl.init(alpha, beta, 0, -FL_M_PI, FL_M_PI, 0, -0.01f*FL_M_PI, 0.01f*FL_M_PI);
            base_type::init(in);
        }

        void S2PLLBlock::reset() {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            pcl.phase = 0;
            pcl.freq = 0;
            base_type::tempStart();
        }

        void S2PLLBlock::setParams(float loop_bw) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            float alpha, beta;
            loop::PhaseControlLoop<float>::criticallyDamped(loop_bw, alpha, beta);
            pcl.setCoefficients(alpha, beta);
            base_type::tempStart();
        }

        int S2PLLBlock::process(int count, complex_t* in, complex_t* out) {
            float errorsum = 0;
            int pilotctr = 0;
            descrambler->reset();
            for (int i = 0; i < (frame_slot_count + 1) * 90 + pilot_cnt * 36; i++) {
                tmp_val = in[i] * math::phasor(-pcl.phase);

                float error = 0;
                if (i >= 90) {
                    complex_t descr = descrambler->descramble(tmp_val);
                    if(pilot_cnt == 0) {
                        constellation->demod_soft_lut(tmp_val, nullptr, &error);
                        out[i] = descr;
                    } else {
                        if(pilotctr >= 0) {
                            constellation->demod_soft_lut(tmp_val, nullptr, &error);
                            out[i] = descr;
                            pilotctr++;
                            if(pilotctr >= 16 * 90) {
                                pilotctr = -1;
                                // printf("START PILOTS\n");
                            }
                        } 
                        if(pilotctr < 0) {
                            error = (descr * (complex_t{descr.re > 0 ? 0.707f : -0.707f, descr.im > 0 ? 0.707f : -0.707f}).conj()).phase() / 10.0f;
                            // error = 0;
                            out[i] = descr;
                            pilotctr--;
                            if(pilotctr <= -36) {
                                pilotctr = 0;
                                // printf("END PILOTS\n");
                            }
                            // printf("PH=%f ERR=%f(IN (%f %f), OUT (%f %f), ID (%f %f))\n", phase, error, tmp_val.re, tmp_val.im, descr.re, descr.im, (complex_t{descr.re > 0 ? 0.707 : -0.707, descr.im > 0 ? 0.707 : -0.707}).re, (complex_t{descr.re > 0 ? 0.707 : -0.707, descr.im > 0 ? 0.707 : -0.707}).im);
                            // error = 0;
                        }
                    }
                } else {
                    if (i < 26) // Use known symbols for SOF
                        error = (tmp_val * sof->symbols[i].conj()).phase();
                    else // And also use known Modcod and config to synchronize onto the PLS
                        error = (tmp_val * pls->symbols[pls_code][i - 26].conj()).phase();

                    // We're done, convert to proper 45 degs BPSK
                    out[i] = (i & 1) ? complex_t{-tmp_val.re, tmp_val.im} : complex_t{tmp_val.im, tmp_val.re};
                }
                errorsum += error;

                pcl.advance(error);
            }
            error = errorsum / ((float)(frame_slot_count + 1) * 90 + pilot_cnt * 36); //class variable, not local

            return (frame_slot_count + 1) * 90 + pilot_cnt * 36;
        }
    }
}
