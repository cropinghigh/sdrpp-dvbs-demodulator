#pragma once

#include <dsp/processor.h>
#include <functional>

namespace dsp {
    namespace dvbs {
        class DVBSymToSoftBlock : public Processor<complex_t, int8_t> {
            using base_type = Processor<complex_t, int8_t>;
        public:
            DVBSymToSoftBlock() {}
            DVBSymToSoftBlock(stream<complex_t>* in, int bufsize) { init(in, bufsize);}
            ~DVBSymToSoftBlock();
            void init(stream<complex_t>* in, int bufsize);

            int process(int count, complex_t* in, int8_t* out);
            std::function<void(complex_t *, int)> syms_callback;

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
        private:
            int in_sym_buffer = 0;
            int8_t* sym_buffer;

        };
    }
}
