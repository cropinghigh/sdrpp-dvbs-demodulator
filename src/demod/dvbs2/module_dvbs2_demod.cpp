#include "module_dvbs2_demod.h"

// #include <fstream>

namespace dsp {
    namespace dvbs2 {
        void DVBS2Demod::init(stream<complex_t>* in, double symbolrate, double samplerate, float agc_rate, float rrc_alpha, int rrc_taps, float loop_bw, float fll_bw, double omegaGain, double muGain, void (*handler)(complex_t* data, int count, void* ctx), void* ctx, int modcod, bool shortframes, bool pilots, float sof_thresold, int max_ldpc_trials, float freq_prop_factor, double omegaRelLimit) {
            // Parse params
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
            d_loop_bw = loop_bw;
            d_modcod = modcod;
            d_shortframes = shortframes;
            d_pilots = pilots;
            d_sof_thresold = sof_thresold;
            d_max_ldpc_trials = max_ldpc_trials;
            d_freqprop = freq_prop_factor;

            float g1 = 0, g2 = 0;

            // Parse modcod number
            auto cfg = dvbs2::get_dvbs2_cfg(d_modcod, d_shortframes, d_pilots);

            frame_slot_count = cfg.frame_slot_count;
            s2_constellation = cfg.constellation;
            s2_constel_obj_type = cfg.constel_obj_type;
            s2_framesize = cfg.framesize;
            s2_coderate = cfg.coderate;
            g1 = cfg.g1;
            g2 = cfg.g2;

            // fll.init(NULL, d_fll_bw, d_symbolrate, d_samplerate, d_rrc_taps, d_rrc_alpha, 0, -FL_M_PI/2.0f, FL_M_PI/2.0f);
            rrcTaps = taps::rootRaisedCosine<float>(d_rrc_taps, d_rrc_alpha, d_symbolrate, d_samplerate);
            rrc.init(NULL, rrcTaps);
            agc.init(NULL, 1.0, 10e6, d_agcr);
            pll.init(NULL, d_fll_bw, d_symbolrate, d_samplerate, d_rrc_taps, d_rrc_alpha, 0, -FL_M_PI/2.0f, FL_M_PI/2.0f);
            recov.init(NULL, d_samplerate / d_symbolrate,  d_clock_gain_omega, d_clock_gain_mu, d_clock_omega_relative_limit);

            // PL (SOF) Synchronization
            pl_sync.init(NULL, frame_slot_count, d_pilots);
            pl_sync.thresold = d_sof_thresold;

            // PLL
            s2_pll.init(NULL, d_loop_bw);
            s2_pll.pilots = d_pilots;
            s2_pll.constellation = std::make_shared<dsp::constellation_t>(s2_constel_obj_type, g1, g2);
            s2_pll.constellation->make_lut(256);
            s2_pll.frame_slot_count = frame_slot_count;
            s2_pll.pls_code = d_modcod << 2 | d_shortframes << 1 | d_pilots;
            s2_pll.update();

            // BB to soft syms
            s2_bb_to_soft.init(NULL);
            s2_bb_to_soft.pilots = d_pilots;
            s2_bb_to_soft.constellation = std::make_shared<dsp::constellation_t>(s2_constel_obj_type, g1, g2);
            s2_bb_to_soft.constellation->make_lut(256);
            s2_bb_to_soft.frame_slot_count = frame_slot_count;
            s2_bb_to_soft.deinterleaver = std::make_shared<dvbs2::S2Deinterleaver>(s2_constellation, s2_framesize, s2_coderate);

            // Init the rest
            ldpc_decoder = new dvbs2::BBFrameLDPC(s2_framesize, s2_coderate);
            bch_decoder = new dvbs2::BBFrameBCH(s2_framesize, s2_coderate);
            descramber = new dvbs2::BBFrameDescrambler(s2_framesize, s2_coderate);

            rrc.out.free();
            agc.out.free();
            recov.out.free();
            pl_sync.out.free();
            s2_pll.out.free();
            s2_bb_to_soft.out.free();

            base_type::init(in);
        }
        DVBS2Demod::~DVBS2Demod() {
            delete ldpc_decoder;
            delete bch_decoder;
            delete descramber;
        }
        void DVBS2Demod::reset() {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            // fll.reset();
            rrc.reset();
            agc.reset();
            pll.reset();
            recov.reset();
            pl_sync.reset();
            s2_pll.reset();
            s2_bb_to_soft.reset();
            base_type::tempStart();
        }

        void DVBS2Demod::setDemodParams(int modcod, bool shortframes, bool pilots, float sof_thresold, int max_ldpc_trials) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            d_modcod = modcod;
            d_shortframes = shortframes;
            d_pilots = pilots;
            d_sof_thresold = sof_thresold;
            d_max_ldpc_trials = max_ldpc_trials;

            float g1 = 0, g2 = 0;

            // Parse modcod number
            auto cfg = dvbs2::get_dvbs2_cfg(d_modcod, d_shortframes, d_pilots);

            frame_slot_count = cfg.frame_slot_count;
            s2_constellation = cfg.constellation;
            s2_constel_obj_type = cfg.constel_obj_type;
            s2_framesize = cfg.framesize;
            s2_coderate = cfg.coderate;
            g1 = cfg.g1;
            g2 = cfg.g2;

            // PL (SOF) Synchronization
            pl_sync.setParams(frame_slot_count, d_pilots);
            pl_sync.thresold = d_sof_thresold;

            // PLL
            s2_pll.pilots = d_pilots;
            s2_pll.constellation = std::make_shared<dsp::constellation_t>(s2_constel_obj_type, g1, g2);
            s2_pll.constellation->make_lut(256);
            s2_pll.frame_slot_count = frame_slot_count;
            s2_pll.pls_code = d_modcod << 2 | d_shortframes << 1 | d_pilots;
            s2_pll.update();

            // BB to soft syms
            s2_bb_to_soft.pilots = d_pilots;
            s2_bb_to_soft.constellation = std::make_shared<dsp::constellation_t>(s2_constel_obj_type, g1, g2);
            s2_bb_to_soft.constellation->make_lut(256);
            s2_bb_to_soft.frame_slot_count = frame_slot_count;
            s2_bb_to_soft.deinterleaver = std::make_shared<dvbs2::S2Deinterleaver>(s2_constellation, s2_framesize, s2_coderate);

            // Init the rest
            delete ldpc_decoder;
            delete bch_decoder;
            delete descramber;
            ldpc_decoder = new dvbs2::BBFrameLDPC(s2_framesize, s2_coderate);
            bch_decoder = new dvbs2::BBFrameBCH(s2_framesize, s2_coderate);
            descramber = new dvbs2::BBFrameDescrambler(s2_framesize, s2_coderate);
            base_type::tempStart();
        }

        void  DVBS2Demod::setSymbolrate(double symbolrate) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            // printf("SsamR: %f\n", samplerate);
            d_symbolrate = symbolrate;
            taps::free(rrcTaps);
            rrcTaps = taps::rootRaisedCosine<float>(d_rrc_taps, d_rrc_alpha, d_symbolrate, d_samplerate);
            recov.setOmega(d_samplerate / d_symbolrate);
            // fll.setSymbolrate(d_symbolrate);
            pll.setSymbolrate(d_symbolrate);
            base_type::tempStart();
        }
        void  DVBS2Demod::setSamplerate(double samplerate) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            // printf("SsamR: %f\n", samplerate);
            d_samplerate = samplerate;
            taps::free(rrcTaps);
            rrcTaps = taps::rootRaisedCosine<float>(d_rrc_taps, d_rrc_alpha, d_symbolrate, d_samplerate);
            recov.setOmega(d_samplerate / d_symbolrate);
            // fll.setSamplerate(d_samplerate);
            pll.setSamplerate(d_samplerate);
            base_type::tempStart();
        }

        int DVBS2Demod::process(int count, const complex_t* in, uint8_t* out) {
            int ret = count;
            int outcnt = 0;
            // std::fstream file;
            ret = agc.process(ret, (complex_t*) in, tmp);
            // ret = fll.process(ret, tmp, tmp);
            // file.open("test.bin", std::ios::app | std::ios::binary);
            // file.write(reinterpret_cast<const char*>(tmp), ret * sizeof(complex_t));
            // file.close();

            ret = pll.process(ret, tmp, tmp);
            ret = rrc.process(ret, tmp, tmp2);
            ret = recov.process(ret, tmp2, tmp);


            ret = pl_sync.process(ret, tmp, tmp2);

            pl_sync_best_match = pl_sync.best_match;
            for(int curfr = 0; curfr < ret/(pl_sync.raw_frame_size); curfr++) {
                int lret = s2_pll.process(ret, &tmp2[curfr*pl_sync.raw_frame_size], tmp); //INCLUDES FREQ CORRECTION INSIDE; MAY CONFLICT WITH FLL!
                lret = s2_bb_to_soft.process(lret, tmp, tmp_frame);

                // Push into constellation
                // constellation_s2.pushComplexPL(&tmp[0], 90);
                // constellation_s2.pushComplexSlots(&tmp[90], frame_slot_count * 90);
                d_handler(&tmp[0], (frame_slot_count + 1) * 90 + s2_pll.pilot_cnt * 36, d_ctx);


                pll.ext_advance(s2_pll.getFreq() * d_freqprop);

                detected_modcod = s2_bb_to_soft.detect_modcod;
                detected_shortframes = s2_bb_to_soft.detect_shortframes;
                detected_pilots = s2_bb_to_soft.detect_pilots;

                memcpy(&simd_packer[simd_packer_ptr], tmp_frame, d_shortframes ? 16200 : 64800);
                simd_packer_ptr += d_shortframes ? 16200 : 64800;
                if(simd_packer_ptr >= (d_shortframes ? 16200 : 64800) * dvbs2::simd_type::SIZE) {
                    simd_packer_ptr = 0;
                    memcpy(sym_buffer, simd_packer, (d_shortframes ? 16200 : 64800) * dvbs2::simd_type::SIZE);

                    ldpc_trials = ldpc_decoder->decode(sym_buffer, d_max_ldpc_trials);

                    if (ldpc_trials == -1)
                        ldpc_trials = d_max_ldpc_trials;

                    for (int ff = 0; ff < dvbs2::simd_type::SIZE; ff++) {
                        int8_t *buf = &sym_buffer[(d_shortframes ? 16200 : 64800) * ff];

                        // Repack
                        memset(repacker_buffer, 0, ldpc_decoder->dataSize());
                        for (int i = 0; i < ldpc_decoder->dataSize(); i++)
                            repacker_buffer[i / 8] = repacker_buffer[i / 8] << 1 | (buf[i] < 0);

                        bch_corrections = bch_decoder->decode(repacker_buffer);

                        descramber->work(repacker_buffer);

                        memcpy(&out[outcnt], repacker_buffer, bch_decoder->dataSize() / 8);
                        outcnt += bch_decoder->dataSize() / 8;
                    }
                }

            }
            return outcnt;
        }
    }
}
