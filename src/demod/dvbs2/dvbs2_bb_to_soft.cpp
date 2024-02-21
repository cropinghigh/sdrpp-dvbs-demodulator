#include "dvbs2_bb_to_soft.h"


namespace dsp {
    namespace dvbs2 {

        void S2BBToSoft::reset() {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            detect_modcod = 0;
            detect_shortframes = 0;
            detect_pilots = 0;
            base_type::tempStart();
        }

        int S2BBToSoft::process(int count, complex_t* in, int8_t* out) {
            // Decode PLS. This should be moved to pl_sync later!
            int best_header = 0;
            {
                uint64_t plheader = 0;
                for (int y = 0; y < 64; y++) {
                    bool value = (in[26 + y] * complex_t{cos(-M_PI / 4), sin(-M_PI / 4)}).re > 0;
                    plheader = plheader << 1 | !value;
                }

                int header_diffs = 64;
                for (int c = 0; c < 128; c++) {
                    int differences = checkSyncMarker(pls.codewords[c], plheader);
                    if (differences < header_diffs) {
                        best_header = c;
                        header_diffs = differences;
                    }
                }
            }

            detect_modcod = best_header >> 2;
            detect_shortframes = best_header & 2;
            detect_pilots = best_header & 1;

            int pilots_offset = 0;

            // // Derandomize and decode slots
            descrambler.reset();
            for (int i = 0; i < frame_slot_count * 90; i++) {
                if (i % 1476 == 0 && i != 0 && pilots)
                    pilots_offset += 36;
                complex_t descr = descrambler.descramble(in[90 + i]);
                constellation->demod_soft_lut(descr, &soft_slots_buffer[(i - pilots_offset) * constellation->getBitsCnt()]);
                // printf("(%f %f) -> (%d %d)\n", in[90 + i].re, in[90 + i].im, (&soft_slots_buffer[(i - pilots_offset) * constellation->getBitsCnt()])[0], (&soft_slots_buffer[(i - pilots_offset) * constellation->getBitsCnt()])[1]);
            }
            // for (int i = 0; i < frame_slot_count * 90; i++) {
            //     if (i % 1476 == 0 && i != 0 && pilots)
            //         pilots_offset += 36;
            //     complex_t descr = descrambler.descramble(in[90 + i]);
            //     // complex_t descr = in[90 + i];
            //     (&soft_slots_buffer[(i - pilots_offset) * constellation->getBitsCnt()])[0] = (descr.im > 0) ? 126 : -126;
            //     (&soft_slots_buffer[(i - pilots_offset) * constellation->getBitsCnt()])[1] = (descr.re > 0) ? 126 : -126;
            //     printf("(%f %f) -> (%d %d)\n", descr.re, descr.im, (&soft_slots_buffer[(i - pilots_offset) * constellation->getBitsCnt()])[0], (&soft_slots_buffer[(i - pilots_offset) * constellation->getBitsCnt()])[1]);
            // }
            

            // Deinterleave
            deinterleaver->deinterleave(soft_slots_buffer, out);

            return frame_slot_count * 90 * constellation->getBitsCnt();
        }
    }
}
