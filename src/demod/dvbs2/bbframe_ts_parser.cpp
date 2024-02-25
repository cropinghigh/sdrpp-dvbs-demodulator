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
 * This file has been adapted from https://github.com/drmpeg/gr-dvbs2rx
 */

#include "bbframe_ts_parser.h"
#include <cstring>
//#include "logger.h"

namespace dsp {
    namespace dvbs2 {
        void BBFrameTSParser::setFrameSize(int bbframe_size) {
            kbch = bbframe_size;

            max_dfl = kbch - 80;
            build_crc8_table();
            crc32_init();

            count = 0;
            index = 0;
            synched = false;
            spanning = false;
        }

        #define CRC_POLY 0xAB
        // Reversed
        #define CRC_POLYR 0xD5

        void BBFrameTSParser::build_crc8_table(void) {
            int r, crc;

            for (int i = 0; i < 256; i++) {
                r = i;
                crc = 0;
                for (int j = 7; j >= 0; j--) {
                    if ((r & (1 << j) ? 1 : 0) ^ ((crc & 0x80) ? 1 : 0)) {
                        crc = (crc << 1) ^ CRC_POLYR;
                    } else {
                        crc <<= 1;
                    }
                }
                crc_tab[i] = crc;
            }
        }

        /*
        * MSB is sent first
        *
        * The polynomial has been reversed
        */
        unsigned int BBFrameTSParser::check_crc8(const uint8_t *in, int length) {
            int crc = 0;
            int b;

            for (int n = 0; n < length; n++) {
                b = ((in[n / 8] >> (7 - (n % 8))) & 1) ^ (crc & 0x01);
                crc >>= 1;
                if (b) {
                    crc ^= CRC_POLY;
                }
            }

            return (crc);
        }

        void BBFrameTSParser::crc32_init(void) {
            unsigned int i, j, k;

            for (i = 0; i < 256; i++) {
                k = 0;
                for (j = (i << 24) | 0x800000; j != 0x80000000; j <<= 1) {
                    k = (k << 1) ^ (((k ^ j) & 0x80000000) ? 0x04c11db7 : 0);
                }
                crc32_table[i] = k;
            }
        }

        int32_t BBFrameTSParser::crc32_checksum(uint8_t* buf, int size, int32_t crc) {
            for (int i = 0; i < size; i++) {
                crc = (crc << 8) ^ crc32_table[((crc >> 24) ^ buf[i]) & 0xff];
            }
            return (crc);
        }

        int BBFrameTSParser::work(uint8_t *bbframe, int cnt, uint8_t *tsframes, int buffer_outsize) {
            int in_p = 0;
            int out_p = 0;
            int tei_p = 0;
            uint8_t tmp;

            int errors = 0;

            int produced = 0;

            int bbproc = 0;

            for (int currentBBIndex = 0; currentBBIndex < cnt; currentBBIndex++) {
                // printf("NEWBB %d\n", currentBBIndex);
                uint8_t *bbf = &bbframe[(kbch / 8) * currentBBIndex]; // Current bbframe
                in_p = 0;

                // Check CRC
                unsigned int crc_check = check_crc8(bbf, 80);

                if (crc_check == 0)
                    crc_check = true;
                else
                    crc_check = false;

                if (crc_check != true) {
                    synched = false;
                    // logger->info("BBHeader CRC Fail");
                    // printf("BBH CRC FAIL\n");
                    continue;
                }

                // Parse header
                BBHeader header(bbf);

                // Validate header
                if (header.dfl > max_dfl || header.syncd >= header.dfl-8) {
                    synched = false;
                    // logger->info("DFL Too long!");
                    continue;
                }

                if (header.dfl % 8 != 0) {
                    synched = false;
                    // printf("DFL%%8\n");
                    // logger->info("DFL Not a multiple of 8!");
                    continue;
                }

                df_remaining = header.dfl / 8;

                // Skip header to datafield
                bbf += 10;

                if (!synched) {
                    // logger->info("Resynchronizing...");
                    // printf("RESYNC\n");
                    bbf += header.syncd / 8 + 1;
                    df_remaining -= header.syncd / 8 + 1;

                    count = 0;
                    synched = true;
                    index = 0;
                    spanning = false;
                    distance = header.syncd / 8;
                }

                last_header = header;
                // printf("TS %d\n", header.ts_gs);
                bbproc++;
                if(header.ts_gs == 0b11) {
                    //MPEGTS
                    while (df_remaining >= TS_SIZE && (buffer_outsize-out_p > TS_SIZE)) {
                        // printf("IT DFR=%d OUTP=%d/%d\n", df_remaining, out_p, buffer_outsize);
                        uint8_t* curptr = 0;
                        if(count > 0) {
                            //something from previous bbframe 
                            int remaining = TS_SIZE - count;
                            memcpy(&packet_reassembly[count], bbf, remaining);
                            bbf += remaining;
                            df_remaining -= remaining;
                            curptr = packet_reassembly;
                            count = 0;
                        } else {
                            curptr = bbf;
                            bbf += TS_SIZE;
                            df_remaining -= TS_SIZE;
                        }
                        //dont check crc, this makes things much much worse
                        // uint pack_crc = check_crc8(curptr, TS_SIZE-1);
                        // uint rx_crc = curptr[TS_SIZE-1];
                        // bool crc_valid = (pack_crc == rx_crc);
                        tsframes[out_p] = TS_SYNC_BYTE;
                        memcpy(&tsframes[out_p+1], curptr, TS_SIZE-1);
                        out_p += TS_SIZE;
                    }
                    if(df_remaining > 0) {
                        count = df_remaining;
                        memcpy(packet_reassembly, bbf, df_remaining);
                        bbf += df_remaining;
                    }
                    if(buffer_outsize - out_p <= TS_SIZE) {
                        printf("BUFF OVF!\n");
                        break;
                    }
                } else if(header.ts_gs == 0b00) {
                    //GSE?
                    //TODO
                } else if(header.ts_gs == 0b01) {
                    int curr_gse_ptr = 0;
                    while(curr_gse_ptr < header.dfl/8) {
                        if(!header.issyi && !header.npd && header.upl == 0) {
                            //GSE
                            uint8_t gse_h1 = bbf[curr_gse_ptr+0];
                            uint8_t gse_h2 = bbf[curr_gse_ptr+1];
                            uint8_t gse_lt = (gse_h1 & 0b00110000) >> 2;
                            if(!((gse_h1 & 0b10000000) >> 7) && !((gse_h1 & 0b01000000) >> 6) && gse_lt == 0) {
                                break; //leaving the bbframe
                            }
                            uint16_t gse_len = 0;
                            gse_len |= ((gse_h1 & 0b00001000) >> 3) << 11;
                            gse_len |= ((gse_h1 & 0b00000100) >> 2) << 10;
                            gse_len |= ((gse_h1 & 0b00000010) >> 1) << 9;
                            gse_len |= ((gse_h1 & 0b00000001) >> 0) << 8;
                            gse_len |= (gse_h2);
                            if(((gse_h1 & 0b10000000) >> 7) && ((gse_h1 & 0b01000000) >> 6) ) {
                                //PDU not fragmented
                                //no fragid
                                //no total_len
                                uint16_t gse_prototype = (bbf[curr_gse_ptr+3] | (bbf[curr_gse_ptr+2] << 8));
                                gse_len -= 2;
                                uint8_t mac[6] = {0,0,0,0,0,0};
                                int datastart = 4;
                                if(gse_lt == 0b00) {
                                    //6-byte MAC
                                    mac[0] = bbf[curr_gse_ptr+4+5];
                                    mac[1] = bbf[curr_gse_ptr+4+4];
                                    mac[2] = bbf[curr_gse_ptr+4+3];
                                    mac[3] = bbf[curr_gse_ptr+4+2];
                                    mac[4] = bbf[curr_gse_ptr+4+1];
                                    mac[5] = bbf[curr_gse_ptr+4+0];
                                    datastart += 6;
                                    gse_len -= 6;
                                } else if(gse_lt == 0b10) {
                                    //3-byte MAC
                                    mac[3] = bbf[curr_gse_ptr+4+2];
                                    mac[4] = bbf[curr_gse_ptr+4+1];
                                    mac[5] = bbf[curr_gse_ptr+4+0];
                                    datastart += 3;
                                    gse_len -= 3;
                                } else {
                                    //broadcast
                                    datastart += 0;
                                    gse_len -= 0;
                                }
                                //FORM AN GRE PACKET TO ENCAPSULATE RECEIVED PACKET IN UDP
                                tsframes[out_p++] = 0b00000000; //no checksum, no key, no seqnum
                                tsframes[out_p++] = 0b00000000; //reserved;ver=0
                                if(gse_prototype == 0x0800) {
                                    tsframes[out_p++] = 0x08;
                                    tsframes[out_p++] = 0x00; //ipv4
                                } else if(gse_prototype == 0x86DD) {
                                    tsframes[out_p++] = 0x86;
                                    tsframes[out_p++] = 0xDD; //ipv6
                                }
                                memcpy(&tsframes[out_p], &bbf[curr_gse_ptr+datastart], gse_len);
                                out_p += gse_len;
                                curr_gse_ptr += datastart+gse_len;
                            } else {
                                if(((gse_h1 & 0b10000000) >> 7)) {
                                    //START
                                    uint8_t fragid = bbf[curr_gse_ptr+2];
                                    uint16_t total_len = (bbf[curr_gse_ptr+4] | (bbf[curr_gse_ptr+3] << 8));
                                    uint16_t gse_prototype = (bbf[curr_gse_ptr+6] | (bbf[curr_gse_ptr+5] << 8));
                                    gse_len -= 5;
                                    uint8_t mac[6] = {0,0,0,0,0,0};
                                    int datastart = 7;
                                    if(gse_lt == 0b00) {
                                        //6-byte MAC
                                        mac[0] = bbf[curr_gse_ptr+7+0];
                                        mac[1] = bbf[curr_gse_ptr+7+1];
                                        mac[2] = bbf[curr_gse_ptr+7+2];
                                        mac[3] = bbf[curr_gse_ptr+7+3];
                                        mac[4] = bbf[curr_gse_ptr+7+4];
                                        mac[5] = bbf[curr_gse_ptr+7+5];
                                        datastart += 6;
                                        gse_len -= 6;
                                    } else if(gse_lt == 0b10) {
                                        //3-byte MAC
                                        mac[0] = bbf[curr_gse_ptr+7+0];
                                        mac[1] = bbf[curr_gse_ptr+7+1];
                                        mac[2] = bbf[curr_gse_ptr+7+2];
                                        datastart += 3;
                                        gse_len -= 3;
                                    } else {
                                        //broadcast
                                        datastart += 0;
                                        gse_len -= 0;
                                    }
                                    for(int rid = 0; rid < 3; rid++) {
                                        if(!gse_reassembly[rid] || fragid == gse_reassembly_id[rid]) {
                                            gse_reassembly[rid] = true;
                                            gse_reassembly_id[rid] = fragid;
                                            gse_reassembly_prototype[rid] = gse_prototype;
                                            for(int i = 0; i < 6; i++) {
                                                gse_reassembly_mac[rid][i] = mac[i];
                                            }
                                            memcpy(&gse_reassembly_buff[rid][0], &bbf[curr_gse_ptr+datastart], gse_len);
                                            gse_reassembly_ctr[rid] = gse_len;
                                            gse_reassembly_crcpart[rid] = 0xffffffff;
                                            gse_reassembly_crcpart[rid] = crc32_checksum(&bbf[curr_gse_ptr+3], 2, gse_reassembly_crcpart[rid]);
                                            gse_reassembly_crcpart[rid] = crc32_checksum(&bbf[curr_gse_ptr+5], 2, gse_reassembly_crcpart[rid]);
                                            if(gse_lt == 0b00) {
                                                gse_reassembly_crcpart[rid] = crc32_checksum(mac, 6, gse_reassembly_crcpart[rid]);
                                            } else if(gse_lt == 0b10) {
                                                gse_reassembly_crcpart[rid] = crc32_checksum(mac, 3, gse_reassembly_crcpart[rid]);
                                            }
                                            gse_reassembly_crcpart[rid] = crc32_checksum(&bbf[curr_gse_ptr+datastart], gse_len, gse_reassembly_crcpart[rid]);
                                            break;
                                        }
                                    }
                                    curr_gse_ptr += datastart+gse_len;
                                } else if(((gse_h1 & 0b01000000) >> 6)) {
                                    //END
                                    uint8_t fragid = bbf[curr_gse_ptr+2];
                                    gse_len -= 1;
                                    int datastart = 3;
                                    for(int rid = 0; rid < 3; rid++) {
                                        if(gse_reassembly[rid] && gse_reassembly_id[rid] == fragid) {
                                            gse_reassembly[rid] = false;
                                            memcpy(&gse_reassembly_buff[rid][gse_reassembly_ctr[rid]], &bbf[curr_gse_ptr+datastart], gse_len);
                                            gse_reassembly_ctr[rid] += gse_len - sizeof(uint32_t);
                                            gse_reassembly_crcpart[rid] = crc32_checksum(&bbf[curr_gse_ptr+datastart], gse_len-sizeof(int32_t), gse_reassembly_crcpart[rid]);
                                            int32_t crc32 = 0;
                                            for(int cb = 1; cb <= sizeof(int32_t); cb++ ) {
                                                ((uint8_t*)&crc32)[cb-1] = bbf[curr_gse_ptr+datastart+gse_len-cb];
                                            }
                                            if(gse_reassembly_crcpart[rid] != crc32) {
                                                // printf("CRC ERROR: 0x%x != 0x%x\n", gse_reassembly_crcpart[0], crc32);
                                                last_gse_crc_err = 1;
                                            } else {
                                                //FORM AN GRE PACKET TO ENCAPSULATE RECEIVED PACKET IN UDP
                                                last_gse_crc_err = 0;
                                                tsframes[out_p++] = 0b00000000; //no checksum, no key, no seqnum
                                                tsframes[out_p++] = 0b00000000; //reserved;ver=0
                                                if(gse_reassembly_prototype[rid] == 0x0800) {
                                                    tsframes[out_p++] = 0x08;
                                                    tsframes[out_p++] = 0x00; //ipv4
                                                } else if(gse_reassembly_prototype[rid] == 0x86DD) {
                                                    tsframes[out_p++] = 0x86;
                                                    tsframes[out_p++] = 0xDD; //ipv6
                                                }

                                                memcpy(&tsframes[out_p], &gse_reassembly_buff[rid][0], gse_reassembly_ctr[rid]);
                                                out_p += gse_reassembly_ctr[rid];
                                            }
                                            break;
                                        }
                                    }
                                    curr_gse_ptr += datastart+gse_len;
                                } else {
                                    //PDU
                                    uint8_t fragid = bbf[curr_gse_ptr+2];
                                    gse_len -= 1;
                                    int datastart = 3;
                                    for(int rid = 0; rid < 3; rid++) {
                                        if(gse_reassembly[rid] && gse_reassembly_id[rid] == fragid) {
                                            memcpy(&gse_reassembly_buff[rid][gse_reassembly_ctr[rid]], &bbf[curr_gse_ptr+datastart], gse_len);
                                            gse_reassembly_ctr[rid] += gse_len;
                                            gse_reassembly_crcpart[rid] = crc32_checksum(&bbf[curr_gse_ptr+datastart], gse_len, gse_reassembly_crcpart[rid]);
                                            break;
                                        }
                                    }
                                    curr_gse_ptr += datastart+gse_len;
                                }
                            }
                        } else {
                            curr_gse_ptr = header.dfl/8; //skipping unknown packet
                        }
                    }
                }
            }
            last_bb_cnt = cnt;
            last_bb_proc = bbproc;
            last_ts_errs = errors;
            return out_p;
        }
    }
}
