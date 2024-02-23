#pragma once

#include <dsp/processor.h>
#include <dsp/loop/phase_control_loop.h>
#include <dsp/math/phasor.h>
#include <dsp/math/step.h>
#include "s2_defs.h"
#include "common/utils.h"
#include "dvbs2/codings/s2_scrambling.h"

namespace dsp {
    namespace dvbs2 {
        class S2PLHDRDemod : public Processor<complex_t, complex_t> {
            using base_type = Processor<complex_t, complex_t>;
        public:
            S2PLHDRDemod() {}
            S2PLHDRDemod(stream<complex_t>* in, float loop_bw, s2_sof* sof, s2_plscodes* pls) { init(in, loop_bw, sof, pls);}
            void init(stream<complex_t>* in, float loop_bw, s2_sof* sof, s2_plscodes* pls);
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

            // uint8_t last_plhdr = 0;
            int detect_modcod = 0;
            bool detect_shortframes = false;
            bool detect_pilots = false;

        private:
            int checkSyncMarker(uint64_t marker, uint64_t totest);

            loop::PhaseControlLoop<float> pcl;
            s2_sof* sof;
            s2_plscodes* pls;
        };
    }
}
