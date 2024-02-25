#include "module_dvbs2_demod.h"

#include <fstream>

namespace dsp {
    namespace dvbs2 {
        void DVBS2Demod::init(stream<complex_t>* in, double symbolrate, double samplerate, float agc_rate, float rrc_alpha, int rrc_taps, float loop_bw, float fll_bw, double omegaGain, double muGain, void (*handler)(complex_t* data, int count, void* ctx), void* ctx, int modcod, bool shortframes, bool pilots, float sof_thresold, int max_ldpc_trials, double omegaRelLimit) {
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
            // rrcTaps = taps::rootRaisedCosine<float>(d_rrc_taps, d_rrc_alpha, d_symbolrate, d_symbolrate);
            rrc.init(NULL, rrcTaps);
            agc.init(NULL, 1.0, 10e6, d_agcr);
            freqShift.init(NULL);
            // pll.init(NULL, d_fll_bw, d_symbolrate, d_samplerate, d_rrc_taps, d_rrc_alpha, 0, -FL_M_PI/2.0f, FL_M_PI/2.0f);
            // recov.init(NULL, d_samplerate / d_symbolrate,  d_clock_gain_omega, d_clock_gain_mu, d_clock_omega_relative_limit, 1);
            recov.init(NULL, d_symbolrate / d_symbolrate,  d_clock_gain_omega, d_clock_gain_mu, d_clock_omega_relative_limit, 2);
            // lms.init(NULL, 65, 0.000001f);

            // PL (SOF) Synchronization
            pl_sync.init(NULL, frame_slot_count, d_pilots, &sof, &pls);
            pl_sync.thresold = d_sof_thresold;

            plhdr_demod.init(NULL, d_loop_bw, &sof, &pls);

            // PLL
            s2_pll.init(NULL, d_loop_bw, &sof, &pls, &scrambler);
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
            freqShift.out.free();
            recov.out.free();
            // lms.out.free();
            pl_sync.out.free();
            plhdr_demod.out.free();
            s2_pll.out.free();
            s2_bb_to_soft.out.free();

            base_type::init(in);
        }
        DVBS2Demod::~DVBS2Demod() {
            taps::free(rrcTaps);
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
            freqShift.curr_freq = 0;
            freqShift.curr_phase = 0;
            // pll.reset();
            recov.reset();
            // lms.reset();
            cr_samp = false;
            pl_sync.reset();
            plhdr_demod.reset();
            s2_pll.reset();
            // s2_bb_to_soft.reset();
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
            if(base_type::_in) {
                base_type::tempStop();
            }
            // printf("SsamR: %f\n", samplerate);
            d_symbolrate = symbolrate;
            taps::free(rrcTaps);
            rrcTaps = taps::rootRaisedCosine<float>(d_rrc_taps, d_rrc_alpha, d_symbolrate, d_samplerate);
            // rrcTaps = taps::rootRaisedCosine<float>(d_rrc_taps, d_rrc_alpha, d_symbolrate, d_symbolrate);
            rrc.setTaps(rrcTaps);
            // recov.setOmega(d_samplerate / d_symbolrate);
            recov.setOmega(d_symbolrate / d_symbolrate);
            cr_samp = false;
            freqShift.curr_freq = 0;
            // fll.setSymbolrate(d_symbolrate);
            // pll.setSymbolrate(d_symbolrate);
            if(base_type::_in) {
                base_type::tempStart();
            }
        }

        void DVBS2Demod::setSamplerate(double samplerate) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            if(base_type::_in) {
                base_type::tempStop();
            }
            // printf("SsamR: %f\n", samplerate);
            d_samplerate = samplerate;
            taps::free(rrcTaps);
            rrcTaps = taps::rootRaisedCosine<float>(d_rrc_taps, d_rrc_alpha, d_symbolrate, d_samplerate);
            // rrcTaps = taps::rootRaisedCosine<float>(d_rrc_taps, d_rrc_alpha, d_symbolrate, d_symbolrate);
            rrc.setTaps(rrcTaps);
            // recov.setOmega(d_samplerate / d_symbolrate);
            recov.setOmega(d_symbolrate / d_symbolrate);
            cr_samp = false;
            freqShift.curr_freq = 0;
            // fll.setSamplerate(d_samplerate);
            // pll.setSamplerate(d_samplerate);
            if(base_type::_in) {
                base_type::tempStart();
            }
        }

        int DVBS2Demod::process(int count, const complex_t* in, uint8_t* out) {
            int ret = count;
            int outcnt = 0;
            std::fstream file;
            ret = agc.process(ret, (complex_t*) in, tmp);
            ret = freqShift.process(ret, tmp, tmp);
            // ret = fll.process(ret, tmp, tmp);

            // ret = pll.process(ret, tmp, tmp);
            ret = recov.process(ret, tmp, tmp2);
            ret = rrc.process(ret, tmp2, tmp2);
            // file.open("test.bin", std::ios::app | std::ios::binary);
            // file.write(reinterpret_cast<const char*>(tmp2), ret * sizeof(complex_t));
            // file.close();

            int newret = 0;
            for(int i = 0; i < ret; i++) {
                if(cr_samp) {
                    tmp[i/2] = tmp2[i];
                    newret++;
                }
                cr_samp = !cr_samp;
            }
            ret = newret; //external decimator


            // ret = lms.process(ret, tmp, tmp);
            // d_handler(&tmp[0], ret, d_ctx);
/*
            int syncseq_len = 256;
            uint8_t sync_seq[] = {1, 1, 1, 0, 3, 0, 0, 3, 0, 3, 3, 0, 0, 2, 2, 0, 2, 1, 0, 3, 2, 1, 3, 0, 1, 2, 1, 1, 3, 1, 1, 1, 3, 3, 0, 1, 1, 3, 2, 1, 2, 3, 0, 0, 0, 2, 3, 0, 0, 1, 0, 2, 3, 1, 1, 1, 3, 1, 1, 2, 3, 1, 3, 2, 1, 0, 1, 3, 3, 1, 2, 0, 1, 1, 1, 0, 2, 3, 0, 1, 1, 2, 0, 1, 0, 0, 1, 2, 0, 1, 3, 1, 1, 3, 1, 1, 0, 0, 3, 1, 0, 1, 3, 2, 2, 1, 1, 1, 3, 3, 0, 3, 3, 3, 1, 2, 0, 3, 3, 1, 1, 3, 2, 3, 3, 0, 3, 0, 1, 2, 3, 1, 2, 3, 0, 1, 1, 0, 3, 3, 0, 3, 2, 2, 0, 1, 3, 1, 2, 3, 2, 2, 0, 0, 1, 0, 1, 3, 0, 1, 1, 1, 2, 2, 1, 3, 3, 3, 3, 3, 2, 2, 0, 2, 3, 0, 0, 0, 0, 2, 2, 2, 0, 0, 3, 2, 1, 3, 0, 3, 3, 3, 0, 1, 3, 2, 0, 3, 0, 2, 0, 1, 1, 1, 0, 3, 3, 2, 0, 3, 3, 0, 0, 1, 3, 1, 0, 0, 2, 1, 1, 3, 1, 0, 3, 3, 1, 0, 3, 0, 2, 2, 0, 1, 1, 3, 0, 1, 3, 1, 1, 1, 2, 3, 3, 2, 1, 2, 0, 3, 0, 3, 3, 3, 1, 3};
            for(int i = 0; i < ret; i++) {
                for(int k = 1; k < syncseq_len; k++) {
                    debug_buff[k-1] = debug_buff[k];
                }
                int x = ((tmp[i].re < 0) << 1) | (tmp[i].im < 0);
                debug_buff[syncseq_len-1] = x;
                if(debug_sync) {
                    if(debug_ctr > 0) {
                        debug_ctr--;
                        continue;
                    }
                    int errs = 0;
                    for(int k = 0; k < syncseq_len; k++) {
                        if(debug_buff[k] != sync_seq[k]) {
                            errs++;
                        }
                    }
                    if(errs != 0) {
                        debug_sync_err += errs;
                        debug_sync_g_err += errs;
                        printf("%d errors(%d/%d, BER %f)\n", errs, debug_sync_g_err, debug_sync_g_b, (((float)debug_sync_g_err)/((float)debug_sync_g_b)));
                        debug_rst_ctr++;
                    } else {
                        debug_rst_ctr = 0;
                    }
                    if(debug_rst_ctr > 10) {
                        printf("RESYNC\n");
                        debug_sync = false;
                        debug_sync_err = 0;
                        debug_sync_b = 0;
                        debug_ctr = 0;
                        debug_rst_ctr = 0;
                        continue;
                    }
                    debug_sync_b += syncseq_len;
                    debug_sync_g_b += syncseq_len;
                    debug_ctr += syncseq_len-1;
                } else {
                    if(debug_sync_err > 0) {
                        debug_sync_err--;
                        continue;
                    }
                    bool ok = true;
                    for(int k = 0; k < syncseq_len; k++) {
                        if(debug_buff[k] != sync_seq[k]) {
                            ok = false;
                            break;
                        }
                    }
                    if(ok) {
                        debug_ctr++;
                        debug_sync_err += syncseq_len-1;
                        printf("ok at %d, %d\n", i, debug_ctr);
                    } else {
                        debug_ctr = 0;
                    }
                    if(debug_ctr >= 1) {
                        printf("SYNC ACQUIRED! start counting\n");
                        debug_sync = true;
                        debug_sync_b += syncseq_len*1;
                        debug_sync_g_b += syncseq_len*1;
                        debug_sync_err = 0;
                        debug_ctr = syncseq_len-1;
                    }
                }
            }*/


            ret = pl_sync.process(ret, tmp, tmp2);

            pl_sync_best_match = pl_sync.best_match;
            for(int curfr = 0; curfr < ret/(pl_sync.raw_frame_size); curfr++) {
                float est_coarse_fr_err = dvbs2_pilot_coarse_fed(&tmp2[curfr*pl_sync.raw_frame_size], pl_sync.raw_frame_size, d_pilots, s2_pll.pls_code, sof, pls, &scrambler);
                // printf("ERR=%f   CUR_FR=%f \n", est_coarse_fr_err, freqShift.curr_freq);
                if(std::abs(est_coarse_fr_err) < 0.02) {
                    freqShift.curr_freq = freqShift.curr_freq + est_coarse_fr_err * (d_fll_bw/100.0f);
                } else {
                    freqShift.curr_freq = freqShift.curr_freq + est_coarse_fr_err * d_fll_bw;
                }
                if(freqShift.curr_freq > 0.3f*FL_M_PI) {
                    freqShift.curr_freq = 0.3f*FL_M_PI;
                }
                if(freqShift.curr_freq < -0.3f*FL_M_PI) {
                    freqShift.curr_freq = -0.3f*FL_M_PI;
                }
                int lret = s2_pll.process(pl_sync.raw_frame_size, &tmp2[curfr*pl_sync.raw_frame_size], tmp);
                lret = plhdr_demod.process(pl_sync.raw_frame_size, &tmp2[curfr*pl_sync.raw_frame_size], tmp);
                lret = s2_bb_to_soft.process(lret, tmp, tmp_frame);

                // Push into constellation
                d_handler(&tmp[0], (frame_slot_count + 1) * 90 + s2_pll.pilot_cnt * 36, d_ctx);

                detected_modcod = plhdr_demod.detect_modcod;
                detected_shortframes = plhdr_demod.detect_shortframes;
                detected_pilots = plhdr_demod.detect_pilots;

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
