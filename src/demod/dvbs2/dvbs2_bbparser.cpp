#include "dvbs2_bbparser.h"

#include <utils/flog.h>

namespace dsp {
    namespace dvbs2 {
        int BBParser::process_one_bbframe(int count, uint8_t* in, uint8_t* out) {
            if(count < 10) {
                return 0;
            }
            matype = in[0];
            upl = in[2];
            dfl = in[4];
            sync = in[6];
            syncd = in[7];
            crc8 = in[9];
            printf("%d %d %d %d %d %d\n", matype, upl, dfl, sync, syncd, crc8);
            int readlen = std::min(dfl*8, count);
            memcpy(out, &in[10], readlen);
            return readlen;
        }
    }
}
