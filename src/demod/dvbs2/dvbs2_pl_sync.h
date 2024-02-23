#pragma once

#include <dsp/processor.h>
#include "s2_defs.h"

/*
PLHeader synchronization, for DVB-S2 clock-recovered symbols
*/
namespace dsp {
    namespace dvbs2 {
        class S2PLSyncBlock : public Processor<complex_t, complex_t> {
            using base_type = Processor<complex_t, complex_t>;
        public:
            S2PLSyncBlock() {}
            S2PLSyncBlock(stream<complex_t>* input, int slot_num, bool pilots, s2_sof* sof, s2_plscodes* pls) { init(input, slot_num, pilots, sof, pls);}
            ~S2PLSyncBlock();
            void init(stream<complex_t>* input, int slot_num, bool pilots, s2_sof* sof, s2_plscodes* pls);
            void reset();
            void setParams(int slot_num, bool pilots);
            int process(int count, complex_t* in, complex_t* out);

            int run() {
                int count = base_type::_in->read();
                if (count < 0) { return -1; }

                int outCount = process(count, base_type::_in->readBuf, base_type::out.writeBuf);

                // Swap if some data was generated
                base_type::_in->flush();
                if (outCount) {
                    if (!base_type::out.swap(outCount)) { return -1; }
                }
                return outCount;
            }

            int slot_number;
            int raw_frame_size;

            int current_position = -1;
            float thresold = 0.6;
            double best_match = 0;
            // float freq = 0;

        private:
            s2_sof* sof;
            s2_plscodes* pls;

            int internal_process(complex_t* out);

            // Utils
            complex_t correlate_sof_diff(complex_t *diffs);
            complex_t correlate_plscode_diff(complex_t *diffs);
            // Return conj(u)*v
            complex_t conjprod(const complex_t &u, const complex_t &v) {
                return complex_t{u.re * v.re + u.im * v.im,
                                u.re * v.im - u.im * v.re};
            }

            // Variables and such
            complex_t *correlation_buffer;
            complex_t in_buffer[STREAM_BUFFER_SIZE];
            int in_buffer_ptr = 0;
            int in_buffer_lim = 0;
            int in_buffer_state = 0;
            int best_pos = 0;
        };
    }
}
