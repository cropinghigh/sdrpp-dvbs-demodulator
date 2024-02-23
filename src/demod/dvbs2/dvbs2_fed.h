#pragma once

#include "s2_defs.h"

namespace dsp {
    namespace dvbs2 {
        inline float dvbs2_pilot_coarse_fed(complex_t* frame, int raw_frame_size, bool pilots, int pls_code, dvbs2::s2_sof fed_sof, dvbs2::s2_plscodes fed_pls, S2Scrambling* scrambler) {
            float err = 0;
            float symcnt = 90-2;
            float sym_err;
            //using formula from ETSI implementation guidelines
            for(int i = 0; i < fed_sof.LENGTH-2; i++) {
                //SOF
                sym_err = (frame[i+2] * fed_sof.symbols[i+2].conj() * frame[i].conj() * fed_sof.symbols[i]).im;
                err += sym_err;
            }
            sym_err = (frame[24+2] * fed_pls.symbols[pls_code][24 - fed_sof.LENGTH + 2].conj() * frame[24].conj() * fed_sof.symbols[24]).im;
            err += sym_err;
            sym_err = (frame[25+2] * fed_pls.symbols[pls_code][25 - fed_sof.LENGTH + 2].conj() * frame[25].conj() * fed_sof.symbols[25]).im;
            err += sym_err;
            for(int i = fed_sof.LENGTH; i < 90-2; i++) {
                //PLSCODE
                sym_err = (frame[i + 2] * fed_pls.symbols[pls_code][i - fed_sof.LENGTH + 2].conj() * frame[i].conj() * fed_pls.symbols[pls_code][i - fed_sof.LENGTH]).im;
                err += sym_err;
            }
            if(pilots) {
                complex_t descr_t1;
                complex_t descr_t2;
                for(int pltblock = 0; pltblock < ((raw_frame_size/90 - 1)/16)-1; pltblock++) {
                    int startsym = 90*17 + pltblock*(90*16 + 37);
                    complex_t* pltstart = &frame[startsym];
                    scrambler->setPos(startsym - 90);
                    for(int i = 0; i < 36; i++) {
                        complex_t descr = scrambler->descramble(pltstart[i]);
                        // complex_t descr = pltstart[i];
                        if(i >= 2) {
                            sym_err = (descr * (complex_t {0.707f, 0.707f}).conj() * descr_t2.conj() * (complex_t {0.707f, 0.707f})).im;
                            err += sym_err;
                        }
                        descr_t2 = descr_t1;
                        descr_t1 = descr;
                        // err += sym_err;
                    }
                    symcnt += 36-2;
                }
            }
            return err / symcnt;
        }
    }
}
