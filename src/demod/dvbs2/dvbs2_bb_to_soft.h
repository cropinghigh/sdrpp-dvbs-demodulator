#pragma once

#include <dsp/processor.h>
#include "s2_defs.h"
#include "common/dsp/demod/constellation.h"
#include "dvbs2/codings/s2_scrambling.h"
#include "dvbs2/codings/s2_deinterleaver.h"
#include "common/utils.h"

/*
DVB-S2 PLL, meant to frequency & phase-recover
a synchronized DVB-S2 frame.
*/
namespace dsp {
    namespace dvbs2 {
        class S2BBToSoft : public Processor<complex_t, int8_t> {
            using base_type = Processor<complex_t, int8_t>;
        public:
            int process(int count, complex_t* in, int8_t* out);

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

            bool pilots = false;

            int frame_slot_count;
            std::shared_ptr<dsp::constellation_t> constellation;
            std::shared_ptr<dvbs2::S2Deinterleaver> deinterleaver;
        private:

            int8_t soft_slots_buffer[64800];

        };
    }
}
