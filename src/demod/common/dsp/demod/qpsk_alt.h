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

#include "fll.h"
#include "complex_fd.h"

namespace dsp {
    namespace demod {
        class QPSK_ALT : public Processor<complex_t, complex_t> {
            using base_type = Processor<complex_t, complex_t>;
        public:
            QPSK_ALT() {}

            QPSK_ALT(stream<complex_t>* in, double symbolrate, double samplerate, int rrcTapCount, double rrcBeta, double agcRate, double costasBandwidth, double fllBandwidth, double omegaGain, double muGain, double omegaRelLimit = 0.01) {
                init(in, symbolrate, samplerate, rrcTapCount, rrcBeta, agcRate, costasBandwidth, fllBandwidth, omegaGain, muGain, omegaRelLimit);
            }

            ~QPSK_ALT();

            void init(stream<complex_t>* in, double symbolrate, double samplerate, int rrcTapCount, double rrcBeta, double agcRate, double costasBandwidth, double fllBandwidth, double omegaGain, double muGain, double omegaRelLimit = 0.01);
            void setSymbolrate(double symbolrate);
            void setSamplerate(double samplerate);
            void setRRCParams(int rrcTapCount, double rrcBeta);
            void setRRCTapCount(int rrcTapCount);
            void setRRCBeta(int rrcBeta);
            void setAGCRate(double agcRate);
            void setCostasBandwidth(double bandwidth);
            void setFllBandwidth(double fllBandwidth);
            void setMMParams(double omegaGain, double muGain, double omegaRelLimit = 0.01);
            void setOmegaGain(double omegaGain);
            void setMuGain(double muGain);
            void setOmegaRelLimit(double omegaRelLimit);
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
            double _symbolrate;
            double _samplerate;
            int _rrcTapCount;
            double _rrcBeta;

            loop::FLL fll;
            tap<float> rrcTaps;
            filter::FIR<complex_t, float> rrc;
            loop::FastAGC<complex_t> agc;
            loop::Costas<4> costas;
            clock_recovery::COMPLEX_FD recov;
            // clock_recovery::MM<complex_t> recov;
        };
    }
}
