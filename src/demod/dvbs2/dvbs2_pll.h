#pragma once

#include <dsp/processor.h>
#include "s2_defs.h"
#include "common/dsp/demod/constellation.h"

/*
DVB-S2 PLL, meant to frequency & phase-recover
a synchronized DVB-S2 frame.
*/
namespace dsp {
    namespace dvbs2 {
        class S2PLLBlock : public Processor<complex_t, complex_t> {
            using base_type = Processor<complex_t, complex_t>;
        public:
            S2PLLBlock() {}
            S2PLLBlock(stream<complex_t>* in, float loop_bw) { init(in, loop_bw);}
            void init(stream<complex_t>* in, float loop_bw);
            void reset();
            void setParams(float loop_bw);

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

            float getFreq() { return freq; }
            float getPh() { return phase; }

            int pls_code;
            int frame_slot_count;
            bool pilots;

            void update() {
                pilot_cnt = 0;
                if (pilots) {
                    int raw_size = (frame_slot_count - 90) / 90;
                    pilot_cnt = 1;
                    raw_size -= 16; // First pilot is 16 slots after the SOF

                    while (raw_size > 16) {
                        raw_size -= 16; // The rest is 32 symbols further
                        pilot_cnt++;
                    }
                }
            }

            std::shared_ptr<dsp::constellation_t> constellation;
            dsp::constellation_t constellation_pilots = dsp::constellation_t(dsp::QPSK);

            int pilot_cnt = 0;

            float error = 0;

        private:
            float phase = 0, freq = 0;
            float alpha, beta;

            s2_sof sof;
            s2_plscodes pls;

            complex_t tmp_val;

        };
    }
}
