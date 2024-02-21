#pragma once

// #include "modules/demod/module_demod_base.h"
// #include "common/dsp/filter/fir.h"
// #include "common/dsp/clock_recovery/clock_recovery_mm.h"

// #include "common/dsp/utils/freq_shift.h"

#include <dsp/processor.h>
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

#include "dvbs2_pl_sync.h"
#include "dvbs2_pll.h"
#include "dvbs2_bb_to_soft.h"

#include "dvbs2/codings/bbframe_descramble.h"
#include "dvbs2/codings/bbframe_bch.h"
#include "dvbs2/codings/bbframe_ldpc.h"
#include "dvbs2/codings/modcod_to_cfg.h"

#include "common/dsp/demod/constellation.h"

#include "symbol_extractor.h"

// #include "common/widgets/constellation_s2.h"
// #include "common/widgets/value_plot.h"



#include <utils/flog.h>

namespace dsp {
    namespace dvbs2 {
        class DVBS2Demod : public Processor<complex_t, uint8_t> {
            using base_type = Processor<complex_t, uint8_t>;
        public:
            DVBS2Demod() {}
            DVBS2Demod(stream<complex_t>* in, double symbolrate, double samplerate, float agc_rate, float rrc_alpha, int rrc_taps, float loop_bw, float fll_bw, double omegaGain, double muGain, void (*handler)(complex_t* data, int count, void* ctx), void* ctx, int modcod, bool shortframes, bool pilots, float sof_thresold, int max_ldpc_trials, float freq_prop_factor, double omegaRelLimit = 0.01) { init(in,  symbolrate, samplerate, agc_rate, rrc_alpha, rrc_taps, loop_bw, fll_bw, omegaGain, muGain, handler, ctx, modcod, shortframes, pilots, sof_thresold, max_ldpc_trials, freq_prop_factor, omegaRelLimit); }
            ~DVBS2Demod();
            void init(stream<complex_t>* in, double symbolrate, double samplerate, float agc_rate, float rrc_alpha, int rrc_taps, float loop_bw, float fll_bw, double omegaGain, double muGain, void (*handler)(complex_t* data, int count, void* ctx), void* ctx, int modcod, bool shortframes, bool pilots, float sof_thresold, int max_ldpc_trials, float freq_prop_factor, double omegaRelLimit = 0.01);
            void reset();

            void setDemodParams(int modcod, bool shortframes, bool pilots, float sof_thresold, int max_ldpc_trials);
            void setSymbolrate(double symbolrate);
            void setSamplerate(double samplerate);

            int process(int count, const complex_t* in, uint8_t* out);

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

            int getKBCH() { return bch_decoder->dataSize();}

            int detected_modcod = -1;
            bool detected_shortframes = false;
            bool detected_pilots = false;
            float pl_sync_best_match = 0;
            float ldpc_trials = -1;
            float bch_corrections = -1;

        private:
            // loop::FLL fll;
            tap<float> rrcTaps;
            loop::FastAGC<complex_t> agc;
            filter::FIR<complex_t, float> rrc;
            loop::EXT_PLL pll;
            // clock_recovery::MM<complex_t> recov; //works worse than FD
            clock_recovery::COMPLEX_FD recov;
            dvbs2::S2PLSyncBlock pl_sync;
            dvbs2::S2PLLBlock s2_pll;
            dvbs2::S2BBToSoft s2_bb_to_soft;

            complex_t tmp[STREAM_BUFFER_SIZE];
            complex_t tmp2[STREAM_BUFFER_SIZE];
            int8_t tmp_frame[STREAM_BUFFER_SIZE];
            uint8_t simd_packer[STREAM_BUFFER_SIZE];
            int8_t sym_buffer[64800 * 32];
            uint8_t repacker_buffer[64800 * 32];
            int simd_packer_ptr = 0;

            float d_rrc_alpha;
            int d_rrc_taps = 31;
            float d_loop_bw;
            float d_fll_bw;
            float d_symbolrate;
            float d_samplerate;
            float d_agcr;
            float d_freqprop;

    // This default will NOT work for 32-APSK, maybe we should tune per-requirements?
    #define REC_ALPHA 1.7e-3
            float d_clock_gain_omega = pow(REC_ALPHA, 2) / 4.0;
            float d_clock_gain_mu = REC_ALPHA;
            float d_clock_omega_relative_limit = 0.005f;

            int d_modcod;
            bool d_shortframes = false;
            bool d_pilots = false;
            float d_sof_thresold = 0.6;
            int d_max_ldpc_trials = 10;

            // UI
            void (*d_handler)(complex_t* data, int count, void* ctx);
            void* d_ctx;

            // DVB-S2 Stuff
            int frame_slot_count;
            dvbs2::dvbs2_constellation_t s2_constellation;
            dsp::constellation_type_t s2_constel_obj_type;
            dvbs2::dvbs2_framesize_t s2_framesize;
            dvbs2::dvbs2_code_rate_t s2_coderate;

            dvbs2::BBFrameLDPC* ldpc_decoder;
            dvbs2::BBFrameBCH* bch_decoder;
            dvbs2::BBFrameDescrambler* descramber;

        };
    }
}
