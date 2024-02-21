#pragma once

#include <cstdint>
#include <cstring>
#include <math.h>

namespace dsp {
    namespace dvbs2 {
        class BBParser {
        public:
            int process_one_bbframe(int count, uint8_t* in, uint8_t* out);

            uint16_t matype = 0;
            uint16_t upl = 0;
            uint16_t dfl = 0;
            uint8_t sync = 0;
            uint16_t syncd = 0;
            uint8_t crc8 = 0;
        private:
        };
    }
}
