#pragma once

#include <dsp/processor.h>
#include <dsp/math/hz_to_rads.h>

/*
Frequency shifer
*/
namespace dsp {
    class FreqShiftBlock : public Processor<complex_t, complex_t> {
            using base_type = Processor<complex_t, complex_t>; 
    public:
        void init(stream<complex_t>* in, double samplerate, double shift);

        void set_freq(double samplerate, double freq);
        void set_freq_raw(double freq); // Allows using this as a manual frequency correction block, eg, as a pre-PLL tool

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

    private:
        std::mutex mtx;
        complex_t phase_delta;
        complex_t phase;
    };
}
