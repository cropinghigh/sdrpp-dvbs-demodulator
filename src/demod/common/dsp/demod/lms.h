#pragma once
#include <dsp/processor.h>

#include <fstream>
#include <iomanip>
#include <sstream>

#include <dsp/loop/phase_control_loop.h>
#include <dsp/taps/windowed_sinc.h>
#include <dsp/multirate/polyphase_bank.h>
#include <dsp/math/step.h>
#include <dsp/loop/costas.h>
#include <dsp/clock_recovery/mm.h>
#include <dsp/taps/root_raised_cosine.h>
#include <dsp/filter/fir.h>
#include <dsp/loop/fast_agc.h>
#include <dsp/loop/costas.h>
#include <dsp/clock_recovery/mm.h>
#include <math.h>

namespace dsp {

    namespace equalizer {
        class LMS : public Processor<complex_t, complex_t> {
            using base_type = Processor<complex_t, complex_t> ;
        public:
            LMS() {}
            LMS(stream<complex_t>* in, int tapCnt, float stepSize) { init(in, tapCnt, stepSize); }
            ~LMS();

            void init(stream<complex_t>* in, int tapCnt, float stepSize);
            void setTapCnt(int tapCnt);
            void setStepSize(float stepSize);
            void reset();

            int process(int count, const complex_t* in, complex_t* out);

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

        protected:
            tap<complex_t> eqTaps;
            complex_t* buffer;

            int _tapCnt;
            float _stepSize;
        };
    }
}
