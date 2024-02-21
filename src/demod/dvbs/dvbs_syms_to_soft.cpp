#include "dvbs_syms_to_soft.h"
#include "dvbs_defines.h"

namespace dsp {
    namespace dvbs {
        // Util
        int8_t clamp(float x) {
            if (x < -127.0)
                return -127;
            if (x > 127.0)
                return 127;
            return x;
        }

        DVBSymToSoftBlock::~DVBSymToSoftBlock() {
            delete[] sym_buffer;
        }

        void DVBSymToSoftBlock::init(stream<complex_t>* in, int bufsize) {
            sym_buffer = new int8_t[bufsize];
            base_type::init(in);
        }

        int DVBSymToSoftBlock::process(int count, complex_t* in, int8_t* out) {

            // Call callback
            syms_callback(in, count);

            // To soft
            int curroutidx = 0;
            for (int i = 0; i < count; i++) {
                sym_buffer[in_sym_buffer + 0] = clamp(in[i].re * 100);
                sym_buffer[in_sym_buffer + 1] = clamp(in[i].im * 100);
                in_sym_buffer += 2;
                if(in_sym_buffer >= VIT_BUF_SIZE) {
                    memcpy(&out[curroutidx], sym_buffer, VIT_BUF_SIZE);
                    curroutidx += VIT_BUF_SIZE; // Swap buffers ready to use for the viterbi
                    in_sym_buffer -= VIT_BUF_SIZE;
                }
            }
            return curroutidx;
        }

    }
}
