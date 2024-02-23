#include "dvbs2_pl_sync.h"

#include <utils/flog.h>

namespace dsp {
    namespace dvbs2 {
        S2PLSyncBlock::~S2PLSyncBlock() {
            delete[] correlation_buffer;
        }

        void S2PLSyncBlock::init(stream<complex_t>* input, int slot_num, bool pilots, s2_sof* esof, s2_plscodes* epls) {
            sof = esof;
            pls = epls;
            slot_number = slot_num;

            raw_frame_size = (slot_number + 1) * 90; // PL (90 Symbols) + slots of 90 symbols

            if (pilots) {
                int raw_size = (raw_frame_size - 90) / 90;
                int pilot_cnt = 1;
                raw_size -= 16; // First pilot is 16 slots after the SOF

                while (raw_size > 16) {
                    raw_size -= 16; // The rest is 32 symbols further
                    pilot_cnt++;
                }

                raw_frame_size += pilot_cnt * 36;

                flog::info("Pilots size (PLSYNC) : {0}", raw_frame_size);
            }

            correlation_buffer = new complex_t[raw_frame_size];
            in_buffer_lim = raw_frame_size;

            base_type::init(input);
        }

        void S2PLSyncBlock::reset() {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            in_buffer_lim = raw_frame_size;
            in_buffer_ptr = 0;
            in_buffer_state = 0;
            base_type::tempStart();
        }

        void S2PLSyncBlock::setParams(int slot_num, bool pilots) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            slot_number = slot_num;

            raw_frame_size = (slot_number + 1) * 90; // PL (90 Symbols) + slots of 90 symbols

            if (pilots) {
                int raw_size = (raw_frame_size - 90) / 90;
                int pilot_cnt = 1;
                raw_size -= 16; // First pilot is 16 slots after the SOF

                while (raw_size > 16) {
                    raw_size -= 16; // The rest is 32 symbols further
                    pilot_cnt++;
                }

                raw_frame_size += pilot_cnt * 36;

                flog::info("Pilots size (PLSYNC) : {0}", raw_frame_size);
            }
            delete[] correlation_buffer;
            correlation_buffer = new complex_t[raw_frame_size];
            in_buffer_lim = raw_frame_size;
            in_buffer_ptr = 0;
            in_buffer_state = 0;
            base_type::tempStart();
        }

        int S2PLSyncBlock::process(int count, complex_t* in, complex_t* out) {
            int outcnt = 0;
            for(int i = 0; i < count; i++) {
                in_buffer[in_buffer_ptr] = in[i];
                in_buffer_ptr++;
                if(in_buffer_ptr >= in_buffer_lim) {
                    outcnt += internal_process(&out[outcnt]);
                    in_buffer_ptr = 0;
                }
            }
            return outcnt;
        }

        int S2PLSyncBlock::internal_process(complex_t* out) {
            if(in_buffer_state == 0) {
                memcpy(correlation_buffer, in_buffer, raw_frame_size*sizeof(complex_t));
                // Correlate the PLHeader
                best_pos = 0;
                best_match = 0;
                complex_t best_match_raw = {0, 0};

                complex_t plheader_symbols[sof->LENGTH + pls->LENGTH];

                for (int ss = 0; ss < raw_frame_size - sof->LENGTH - pls->LENGTH; ss++) {

                    plheader_symbols[0] = {0, 0};
                    volk_32fc_conjugate_32fc((lv_32fc_t *)&plheader_symbols[1], (lv_32fc_t *)&correlation_buffer[ss], sof->LENGTH + pls->LENGTH - 1);
                    volk_32fc_x2_multiply_32fc((lv_32fc_t *)plheader_symbols, (lv_32fc_t *)plheader_symbols, (lv_32fc_t *)&correlation_buffer[ss], sof->LENGTH + pls->LENGTH);

                    double difference = 0;

                    complex_t csof = correlate_sof_diff(plheader_symbols);
                    complex_t cplsc = correlate_plscode_diff(&plheader_symbols[sof->LENGTH]);
                    complex_t c0 = csof + cplsc; // Best when b7==0 (pilots off)
                    complex_t c1 = csof - cplsc; // Best when b7==1 (pilots on)
                    complex_t c = c0.amplitude() > c1.amplitude() ? c0 : c1;
                    complex_t d = c * (1.0f / (26 - 1 + 64 / 2));

                    difference = d.amplitude();

                    if (difference > best_match && d.im > 0) {
                        best_match = difference;
                        best_pos = ss;
                        best_match_raw = d;

                        current_position = best_pos;

                        if (difference > thresold) {
                            goto skip_slow_corr;
                        }
                    }
                }
skip_slow_corr:
                if (best_pos != 0 && best_pos < raw_frame_size) { // Safety
                    in_buffer_lim = best_pos;
                    in_buffer_state = 1; //gather another pos symbols
                    return 0;
                }
            } else {
                if (best_pos != 0 && best_pos < raw_frame_size) { // Safety
                    int pos = best_pos;
                    memmove(&correlation_buffer[0], &correlation_buffer[pos], (raw_frame_size - pos) * sizeof(complex_t));
                    // ring_buffer.read(&correlation_buffer[raw_frame_size - pos], pos);
                    memcpy(&correlation_buffer[raw_frame_size - pos], in_buffer, pos*sizeof(complex_t));
                    best_pos = 0;
                }
                in_buffer_lim = raw_frame_size;
                in_buffer_state = 0;
            }

            memcpy(out, correlation_buffer, raw_frame_size * sizeof(complex_t));
            return raw_frame_size;
        }

        complex_t S2PLSyncBlock::correlate_sof_diff(complex_t *diffs) {
            complex_t c = {0, 0};
            const uint32_t dsof = sof->VALUE ^ (sof->VALUE >> 1);
            for (int i = 0; i < sof->LENGTH; ++i) {
                // Constant  odd bit => +PI/4
                // Constant even bit => -PI/4
                // Toggled   odd bit => -PI/4
                // Toggled  even bit => +PI/4
                if (((dsof >> (sof->LENGTH - 1 - i)) ^ i) & 1)
                    c += diffs[i];
                else
                    c -= diffs[i];
            }
            return c;
        }

        complex_t S2PLSyncBlock::correlate_plscode_diff(complex_t *diffs) {
            complex_t c = {0, 0};
            uint64_t dscr = pls->SCRAMBLING ^ (pls->SCRAMBLING >> 1);
            for (int i = 1; i < pls->LENGTH; i += 2) {
                if ((dscr >> (pls->LENGTH - 1 - i)) & 1)
                    c -= diffs[i];
                else
                    c += diffs[i];
            }
            return c;
        }
    }
}
