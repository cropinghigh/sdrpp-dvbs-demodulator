#include "dvbs2_bb_to_soft.h"


namespace dsp {
    namespace dvbs2 {

        int S2BBToSoft::process(int count, complex_t* in, int8_t* out) {
            int pilots_offset = 0;

            // // Derandomize and decode slots
            for (int i = 0; i < frame_slot_count * 90; i++) {
                if (i % 1476 == 0 && i != 0 && pilots)
                    pilots_offset += 36;
                complex_t descr = in[90 + i];
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
