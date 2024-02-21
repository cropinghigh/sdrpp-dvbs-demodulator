#include "module_dvbs_demod.h"

namespace dsp {
    namespace dvbs {
        DVBSDemod::~DVBSDemod() {
            delete viterbi;
            delete ts_deframer;
        }
        void DVBSDemod::init(stream<complex_t>* in, double symbolrate, double samplerate, float agc_rate, float rrc_alpha, int rrc_taps, float loop_bw, float fll_bw, double omegaGain, double muGain, void (*handler)(complex_t* data, int count, void* ctx), void* ctx, double omegaRelLimit) {
            d_symbolrate = symbolrate;
            d_samplerate = samplerate;
            d_rrc_taps = rrc_taps;
            d_rrc_alpha = rrc_alpha;
            d_agcr = agc_rate;
            d_loop_bw = loop_bw;
            d_fll_bw = fll_bw;
            d_clock_gain_omega = omegaGain;
            d_clock_gain_mu = muGain;
            d_clock_omega_relative_limit = omegaRelLimit;
            d_handler = handler;
            d_ctx = ctx;

            viterbi = new viterbi::Viterbi_DVBS(0.15, 20, VIT_BUF_SIZE, {PHASE_0, PHASE_90});
            ts_deframer = new deframing::DVBS_TS_Deframer();

            demod.init(NULL, d_symbolrate, d_samplerate, d_rrc_taps, d_rrc_alpha, d_agcr, d_loop_bw, d_fll_bw, d_clock_gain_omega, d_clock_gain_mu, d_clock_omega_relative_limit);
            demod.out.free();


            // Samples to soft
            sts.init(NULL, STREAM_BUFFER_SIZE);
            sts.out.free();
            sts.syms_callback = [this](complex_t *buf, int size)
            {
                // Push into constellation
                d_handler(buf, size, d_ctx);
            };

            // Viterbi
            vit.init(NULL);
            vit.out.free();
            vit.viterbi = viterbi;

            // Deframer
            def.init(NULL);
            def.ts_deframer = ts_deframer;
            def.out.free();

            base_type::init(in);
        }

        void DVBSDemod::reset() {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            demod.reset();
            base_type::tempStart();
        }

        void DVBSDemod::setSymbolrate(double symbolrate) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            d_symbolrate = symbolrate;
            demod.setSymbolrate(d_symbolrate);
            base_type::tempStart();
        }

        void DVBSDemod::setSamplerate(double samplerate) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            d_samplerate = samplerate;
            demod.setSamplerate(d_samplerate);
            base_type::tempStart();
        }

        int DVBSDemod::process(int count, const complex_t* in, uint8_t* out) {
            int cnt = demod.process(count, in, tmp);
            cnt = sts.process(cnt, tmp, (int8_t*)tmp_frame);
            cnt = vit.process(cnt, (int8_t*)tmp_frame, out);
            cnt = def.process(cnt, out, tmp_frame);
            int frm_cnt = cnt / (204 * 8);

            int outidx = 0;
            for (int k = 0; k < frm_cnt; k++) {
                uint8_t *current_frame = &tmp_frame[k * 204];

                dvb_interleaving.deinterleave(current_frame, tmp_deinterleaved_frame);

                for (int i = 0; i < 8; i++) {
                    errors[i] = reed_solomon.decode(&tmp_deinterleaved_frame[204 * i]);
                }

                scrambler.descramble(tmp_deinterleaved_frame);
                for (int i = 0; i < 8; i++) {
                    memcpy(&out[outidx], &tmp_deinterleaved_frame[204 * i], 188);
                    outidx += 188;
                }
            }
            stats_viterbi_ber = viterbi->ber();
            stats_viterbi_lock = viterbi->getState();
            // Get rate
            if (viterbi->rate() == viterbi::RATE_1_2)
                stats_viterbi_rate = "1/2";
            else if (viterbi->rate() == viterbi::RATE_2_3)
                stats_viterbi_rate = "2/3";
            else if (viterbi->rate() == viterbi::RATE_3_4)
                stats_viterbi_rate = "3/4";
            else if (viterbi->rate() == viterbi::RATE_5_6)
                stats_viterbi_rate = "5/6";
            else if (viterbi->rate() == viterbi::RATE_7_8)
                stats_viterbi_rate = "7/8";
            stats_rs_avg = (errors[0] + errors[1] + errors[2] + errors[3] + errors[4] + errors[5] + errors[6] + errors[7]) / 8;
            stats_deframer_err = std::min(ts_deframer->errors_nor, ts_deframer->errors_inv);
            return outidx;
        }
    }
}
