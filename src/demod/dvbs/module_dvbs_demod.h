#pragma once

#include "viterbi_all.h"
#include "dvbs_ts_deframer.h"
#include "dvbs_syms_to_soft.h"
#include "dvbs_vit.h"
#include "dvbs_defra.h"
#include "dvbs_interleaving.h"
#include "dvbs_reedsolomon.h"
#include "dvbs_scrambling.h"
#include "dvbs_defines.h"
#include <dsp/demod/psk.h>
#include "common/dsp/demod/qpsk_alt.h"
#include <math.h>

namespace dsp {
    namespace dvbs {
        class DVBSDemod : public Processor<complex_t, uint8_t> {
            using base_type = Processor<complex_t, uint8_t>;
        public:
            DVBSDemod() {}
            DVBSDemod(stream<complex_t>* in, double symbolrate, double samplerate, float agc_rate, float rrc_alpha, int rrc_taps, float loop_bw, float fll_bw, double omegaGain, double muGain, void (*handler)(complex_t* data, int count, void* ctx), void* ctx, double omegaRelLimit = 0.01) { init(in,  symbolrate, samplerate, agc_rate, rrc_alpha, rrc_taps, loop_bw, fll_bw, omegaGain, muGain, handler, ctx, omegaRelLimit); }
            ~DVBSDemod();
            void init(stream<complex_t>* in, double symbolrate, double samplerate, float agc_rate, float rrc_alpha, int rrc_taps, float loop_bw, float fll_bw, double omegaGain, double muGain, void (*handler)(complex_t* data, int count, void* ctx), void* ctx, double omegaRelLimit = 0.01);
            void reset();

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
        // UI Stuff
        float stats_viterbi_ber = 0;
        int stats_viterbi_lock = 0;
        std::string stats_viterbi_rate = "";
        float stats_rs_avg = 0;
        int stats_deframer_err = 0;

        private:
            demod::QPSK_ALT demod;
            complex_t tmp[STREAM_BUFFER_SIZE];
            uint8_t tmp_frame[STREAM_BUFFER_SIZE];
            uint8_t tmp_deinterleaved_frame[204 * 8];

            DVBSymToSoftBlock sts;
            DVBSVitBlock vit;
            DVBSDefra def;

            float d_symbolrate;
            float d_samplerate;
            float d_agcr;
            float d_rrc_alpha = 0.35;
            int d_rrc_taps = 31;
            float d_loop_bw;
            float d_fll_bw;
            float d_clock_gain_omega = pow(8.7e-3, 2) / 4.0;
            float d_clock_gain_mu = 8.7e-3;
            float d_clock_omega_relative_limit = 0.005f;
            void (*d_handler)(complex_t* data, int count, void* ctx);
            void* d_ctx;

            viterbi::Viterbi_DVBS* viterbi;
            deframing::DVBS_TS_Deframer* ts_deframer;
            DVBSInterleaving dvb_interleaving;
            DVBSReedSolomon reed_solomon;
            DVBSScrambling scrambler;

            int errors[8];

        };
    }
}
