/* -*- c++ -*- */
/*
 * Copyright 2018 Ron Economos.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

/*
 * This file has been adapted from https://github.com/drmpeg/gr-dvbs2rx.
 * A lot has been modified, especially that this now deals with bytes
 */

#include <cstdint>
#include "common/codings/dvb-s2/dvbs2.h"
#include <dsp/stream.h>
#include <cstdio>

#define TS_SIZE 188
#define TS_SYNC_BYTE 0x47
#define TS_ERROR_INDICATOR 0x80

namespace dsp {
    namespace dvbs2 {
        struct BBHeader {
            uint8_t ts_gs = 0;
            bool sis_mis = false;
            bool ccm_acm = false;
            bool issyi = false;
            bool npd = false;
            uint8_t ro = 0;
            uint8_t isi = 0;
            uint16_t upl = 0;
            uint16_t dfl = 0;
            uint8_t sync = 0;
            uint16_t syncd = 0;

            BBHeader() {}

            BBHeader(uint8_t *bbf) {
                ts_gs = bbf[0] >> 6;
                sis_mis = (bbf[0] >> 5) & 1;
                ccm_acm = (bbf[0] >> 4) & 1;
                issyi = (bbf[0] >> 3) & 1;
                npd = (bbf[0] >> 2) & 1;
                ro = bbf[0] & 0b11;
                isi = sis_mis == 0 ? bbf[1] : 0;
                upl = bbf[2] << 8 | bbf[3];
                dfl = bbf[4] << 8 | bbf[5];
                sync = bbf[6];
                syncd = bbf[7] << 8 | bbf[8];
            }
        };

        class BBFrameTSParser {
        public:
            void setFrameSize(int bbframe_size);

            int work(uint8_t *bbframe, int cnt, uint8_t *tsframes, int buffer_outsize); // TSFrame should be at least as big as bbframe to be safe

            BBHeader last_header;
            bool last_gse_crc_err = 0;
            int last_bb_cnt = 0;
            int last_bb_proc = 0;
            int last_ts_errs = 0;
        private:
            unsigned int kbch;
            unsigned int max_dfl;
            unsigned int df_remaining;
            unsigned int count;
            unsigned int synched;
            unsigned char crc;
            unsigned int distance;
            unsigned int spanning;
            unsigned int index;
            unsigned char packet[188];
            unsigned char packet_reassembly[TS_SIZE];

            //allows up to 3 simultaneous defragmentations
            uint8_t gse_reassembly_buff[3][65536];
            bool gse_reassembly[3] = {0, 0, 0};
            int gse_reassembly_ctr[3] = {0, 0, 0};
            int gse_reassembly_id[3] = {0, 0, 0};
            uint16_t gse_reassembly_prototype[3];
            uint8_t gse_reassembly_mac[3][6];
            int32_t gse_reassembly_crcpart[3];

            void build_crc8_table(void);
            unsigned int check_crc8(const unsigned char *, int);
            unsigned char crc_tab[256];

            //taken from gr-dvbgse
            void crc32_init(void);
            int32_t crc32_checksum(uint8_t* buf, int size, int32_t crc);
            unsigned int crc32_table[256];
        };
    }
}
