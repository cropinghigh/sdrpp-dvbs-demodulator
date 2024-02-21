#include "freq_shift.h"
#include <volk/volk.h>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846 /* pi */
#endif

namespace dsp
{
    void FreqShiftBlock::init(stream<complex_t>* in, double samplerate, double shift) {
        set_freq(samplerate, shift);
        base_type::init(in);
    }

    int FreqShiftBlock::process(int count, const complex_t* in, complex_t* out) {
        mtx.lock();
        lv_32fc_t phd_volk = lv_cmake(phase_delta.re, phase_delta.im);
        volk_32fc_s32fc_x2_rotator_32fc((lv_32fc_t *)out, (lv_32fc_t *)in, phd_volk, (lv_32fc_t *)&phase, count);
        mtx.unlock();

        return count;
    }

    void FreqShiftBlock::set_freq(double samplerate, double shift) {
        mtx.lock();
        phase = complex_t{1, 0};
        phase_delta = complex_t{cos(math::hzToRads(shift, samplerate)), sin(math::hzToRads(shift, samplerate))};
        mtx.unlock();
    }

    void FreqShiftBlock::set_freq_raw(double freq) {
        mtx.lock();
        phase_delta = complex_t{cosf(freq), sinf(freq)};
        mtx.unlock();
    }
}
