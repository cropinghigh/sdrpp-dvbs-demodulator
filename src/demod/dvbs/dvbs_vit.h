#pragma once

#include <dsp/processor.h>
#include "viterbi_all.h"

namespace dsp {
    namespace dvbs {
        class DVBSVitBlock : public Processor<int8_t, uint8_t> {
            using base_type = Processor<int8_t, uint8_t>;
        public:
            viterbi::Viterbi_DVBS *viterbi;
            int process(int count, int8_t* in, uint8_t* out);

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
        };
    }
}
