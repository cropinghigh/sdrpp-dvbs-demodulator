#include "dvbs_defra.h"

namespace dsp {
    namespace dvbs {
        int DVBSDefra::process(int count, uint8_t* in, uint8_t* out) {
            int frm_cnt = ts_deframer->work(in, count, out);

            return frm_cnt * 204 * 8;
        }
    }
}
