#include "dvbs_ts_deframer.h"
#include <cstdint>
#include <cstring>
#include <cstdio>

namespace deframing
{
    DVBS_TS_Deframer::DVBS_TS_Deframer()
    {
        frame_buffer = new uint8_t[TS_SIZE];
        full_frame_shifter = new uint8_t[TS_SIZE];
    }

    DVBS_TS_Deframer::~DVBS_TS_Deframer()
    {
        delete[] frame_buffer;
        delete[] full_frame_shifter;
    }

    int DVBS_TS_Deframer::getState()
    {
        return 0; // d_state;
    }

    inline uint8_t pack_8(uint8_t *bits)
    {
        return bits[0] << 7 |
               bits[1] << 6 |
               bits[2] << 5 |
               bits[3] << 4 |
               bits[4] << 3 |
               bits[5] << 2 |
               bits[6] << 1 |
               bits[7] << 0;
    }

    int DVBS_TS_Deframer::work(uint8_t *input, int size, uint8_t *output)
    {
        int frame_count = 0;

        for (int ibit = 0; ibit < size; ibit++) // We work bit-per-bit
        {
            // Use a large shifter, we can't use a normal one!
            memmove(&full_frame_shifter[0], &full_frame_shifter[1], TS_SIZE - 1);
            full_frame_shifter[TS_SIZE - 1] = input[ibit];
            if(skip_bits > 0) {
                skip_bits--;
                continue;
            }

            int t_errors_nor = 0;
            int t_errors_inv = 0;
            

            // 8 Sync bytes. First one should be inverted
            for (int i = 0; i < 8; i++)
            {
                sync_bytes[i] = pack_8(&full_frame_shifter[204*8 * i]);
                t_errors_nor += compare_8(i == 0 ? 0xB8 : 0x47, sync_bytes[i]);
                t_errors_inv += compare_8(i == 0 ? 0x47 : 0xB8, sync_bytes[i]);
            }

            // logger->critical("%d %d", errors_nor, errors_inv);

            if (t_errors_nor <= 8)
            {
                // printf("NOR ERR: %d\n", errors_nor);
                uint8_t *outfrm = &output[frame_count * 204 * 8];
                memset(outfrm, 0, 204 * 8);
                for (int i = 0; i < 204 * 8 * 8; i++)
                    outfrm[i / 8] = outfrm[i / 8] << 1 | full_frame_shifter[i];
                frame_count++;
                errors_nor = t_errors_nor;
                errors_inv = 0;
                // skip_bits+=204*8*8;
            }
            if (t_errors_inv <= 8)
            {
                // printf("INV ERR: %d\n", errors_inv);
                uint8_t *outfrm = &output[frame_count * 204 * 8];
                memset(outfrm, 0, 204 * 8);
                for (int i = 0; i < 204 * 8 * 8; i++)
                    outfrm[i / 8] = outfrm[i / 8] << 1 | !full_frame_shifter[i];
                frame_count++;
                errors_nor = 0;
                errors_inv = t_errors_inv;
                // skip_bits+=204*8*8;
            }
        }

        return frame_count;
    }

    void DVBS_TS_Deframer::write_bit(uint8_t b)
    {
        frame_buffer[bit_of_frame / 8] = frame_buffer[bit_of_frame / 8] << 1 | b;
        bit_of_frame++;
    }

    void DVBS_TS_Deframer::reset_frame()
    {
        memset(frame_buffer, 0, TS_SIZE / 8);
        frame_buffer[0] = TS_ASM;
        // bit_of_frame = TS_ASM_SIZE;
    }
}
