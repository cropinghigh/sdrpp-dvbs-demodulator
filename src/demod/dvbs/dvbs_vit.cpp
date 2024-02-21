#include "dvbs_vit.h"
#include "dvbs_defines.h"

namespace dsp {
    namespace dvbs {
        int DVBSVitBlock::process(int count, int8_t* in, uint8_t* out) {
            int oidx = 0;
            for(int i = 0; i < count; i+=VIT_BUF_SIZE) {
                int size = viterbi->work(&in[i], VIT_BUF_SIZE, &out[oidx]);
                oidx += size;
            }
            return oidx;
        }
    }
}
