#pragma once

#include <dsp/processor.h>
#include <math.h>

#include "dvbs_ts_deframer.h"

namespace dsp {
    namespace dvbs {
        class DVBSDefra : public Processor<uint8_t, uint8_t> {
            using base_type = Processor<uint8_t, uint8_t>;
        public:
            int process(int count, uint8_t* in, uint8_t* out);

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

            deframing::DVBS_TS_Deframer *ts_deframer;
        };
    }
}
