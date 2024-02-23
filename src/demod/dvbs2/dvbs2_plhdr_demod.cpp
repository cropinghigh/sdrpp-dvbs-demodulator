#include "dvbs2_plhdr_demod.h"

namespace dsp {
    namespace dvbs2 {
        void S2PLHDRDemod::init(stream<complex_t>* in, float loop_bw, s2_sof* esof, s2_plscodes* epls) {
            sof = esof;
            pls = epls;
            float alpha, beta;
            loop::PhaseControlLoop<float>::criticallyDamped(loop_bw, alpha, beta);
            pcl.init(alpha, beta, 0, -FL_M_PI, FL_M_PI, 0, -0.2f*FL_M_PI, 0.2f*FL_M_PI);
            base_type::init(in);
        }

        void S2PLHDRDemod::reset() {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            pcl.phase = 0;
            pcl.freq = 0;
            base_type::tempStart();
        }

        void S2PLHDRDemod::setParams(float loop_bw) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            float alpha, beta;
            loop::PhaseControlLoop<float>::criticallyDamped(loop_bw, alpha, beta);
            pcl.setCoefficients(alpha, beta);
            base_type::tempStart();
        }

        int S2PLHDRDemod::process(int count, complex_t* in, complex_t* out) {
            for(int i = 0; i < 90; i++) {
                complex_t tmp_val = in[i] * math::phasor(-pcl.phase);
                float error = 0;
                if (i < 26) // Use known symbols for SOF
                    error = (tmp_val * sof->symbols[i].conj()).phase();
                else // Use QPSK costas loop since we don't know MODCOD yet
                    error = (math::step(tmp_val.re) * tmp_val.im) - (math::step(tmp_val.im) * tmp_val.re);

                // We're done, convert to proper 45 degs BPSK
                out[i] = ((i & 1) ? complex_t{-tmp_val.re, tmp_val.im} : complex_t{tmp_val.im, tmp_val.re});
                pcl.advance(error);
            }
            // Decode PLS. TODO: check if it actually does R-M decoding
            int best_header = 0;
            {
                uint64_t plheader = 0;
                for (int y = 0; y < 64; y++) {
                    bool value = (out[26 + y] * complex_t{cos(-M_PI / 4), sin(-M_PI / 4)}).re > 0;
                    plheader = plheader << 1 | !value;
                }

                int header_diffs = 64;
                for (int c = 0; c < 128; c++) {
                    int differences = checkSyncMarker(pls->codewords[c], plheader);
                    if (differences < header_diffs) {
                        best_header = c;
                        header_diffs = differences;
                    }
                }
            }

            detect_modcod = (best_header >> 2) & 0b11111;
            detect_shortframes = (best_header & 2) >> 1;
            detect_pilots = best_header & 1;
            // last_plhdr = 0;
            return count;
        }

        int S2PLHDRDemod::checkSyncMarker(uint64_t marker, uint64_t totest) {
            int errors = 0;
            for (int i = 59; i >= 0; i--) {
                bool markerBit, testBit;
                markerBit = getBit<uint64_t>(marker, i);
                testBit = getBit<uint64_t>(totest, i);
                if (markerBit != testBit)
                    errors++;
            }
            return errors;
        }

    }
}
