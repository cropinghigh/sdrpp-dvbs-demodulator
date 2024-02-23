#pragma once

#include "dvbs2/dvbs2.h"
#include "common/dsp/demod/constellation.h"

namespace dsp {
    namespace dvbs2
    {
        struct dvb_cgf_holder
        {
            int frame_slot_count;
            dvbs2::dvbs2_constellation_t constellation;
            dsp::constellation_type_t constel_obj_type;
            dvbs2::dvbs2_framesize_t framesize;
            dvbs2::dvbs2_code_rate_t coderate;
            bool pilots;
            float g1, g2;
        };

        dvb_cgf_holder get_dvbs2_cfg(int modcod, bool shortframes, bool pilots);
        int get_dvbs2_modcod(dvb_cgf_holder dvbs2_cfg);
    }
}
