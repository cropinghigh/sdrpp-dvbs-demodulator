#include "lms.h"

namespace dsp {
    namespace equalizer {
        LMS::~LMS() {
            if (!base_type::_block_init) { return; }
            base_type::stop();
            taps::free(eqTaps);
            buffer::free(buffer);
        }

        void LMS::init(stream<complex_t>* in, int tapCnt, float stepSize) {
            _tapCnt = tapCnt;
            _stepSize = stepSize;
            eqTaps = taps::alloc<complex_t>(_tapCnt);
            buffer = buffer::alloc<complex_t>(_tapCnt);
            for(int i = 0; i < _tapCnt; i++) {
                eqTaps.taps[i] = complex_t{(i == _tapCnt/2) ? 1.0f : 0.0f, 0};
                buffer[i] = {0, 0};
            }
            base_type::init(in);
        }

        void LMS::setTapCnt(int tapCnt) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            _tapCnt = tapCnt;
            // taps::free(eqTaps);
            // eqTaps = taps::alloc<complex_t>(_tapCnt);
            // for(int i = 0; i < _tapCnt; i++) {
            //     eqTaps.taps[i] = {(i == _tapCnt/2) ? 1.0f : 0.0f, 0};
            // }
            // buffer::free(buffer);
            // buffer = buffer::alloc<complex_t>(_tapCnt);
            base_type::tempStart();
        }

        void LMS::setStepSize(float stepSize) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            _stepSize = stepSize;
            base_type::tempStart();
        }

        void LMS::reset() {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            // for(int i = 0; i < _tapCnt; i++) {
            //     eqTaps.taps[i] = {(i == _tapCnt/2) ? 1.0f : 0.0f, 0};
            // }
            base_type::tempStart();
        }

        int LMS::process(int count, const complex_t* in, complex_t* out) {
            for(int i = 0; i < count; i++) {
                memmove(&buffer[0], &buffer[1], (_tapCnt-1)*sizeof(complex_t));
                buffer[_tapCnt-1] = in[i];
                out[i] = {0, 0};
                for(int k = 0; k < _tapCnt; k++) {
                    out[i] += buffer[k] * eqTaps.taps[k];
                }
                complex_t guessed_symbol = {out[i].re > 0 ? 0.707f : -0.707f, out[i].im > 0 ? 0.707f : -0.707f}; //SIMPLE QPSK
                complex_t error = guessed_symbol - out[i];
                for(int k = 0; k < _tapCnt; k++) {
                    eqTaps.taps[k] = (eqTaps.taps[k].conj() + error.conj() * buffer[k] * 2.0f * _stepSize).conj();
                }
            }
            return count;
        }
    }
}
