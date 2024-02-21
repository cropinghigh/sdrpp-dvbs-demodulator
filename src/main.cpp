#include <imgui.h>
#include <config.h>
#include <core.h>
#include <gui/style.h>
#include <gui/gui.h>
#include <signal_path/signal_path.h>
#include <module.h>
#include <unistd.h>
#include <fstream>
#include "dvbs/module_dvbs_demod.h"
#include "dvbs2/module_dvbs2_demod.h"
#include "dvbs2/bbframe_ts_parser.h"

#include <gui/widgets/constellation_diagram.h>
#include <gui/widgets/file_select.h>
#include <gui/widgets/volume_meter.h>

#include "gui_widgets.h"

#include <utils/net.h>
#include <utils/flog.h>

#define CONCAT(a, b)    ((std::string(a) + b).c_str())

SDRPP_MOD_INFO {
    /* Name:            */ "dvbs_demodulator",
    /* Description:     */ "DVB-S&DVB-S2 demodulator for SDR++(based on satdump demod code)",
    /* Author:          */ "cropinghigh",
    /* Version:         */ 0, 0, 5,
    /* Max instances    */ -1
};

ConfigManager config;

const char* s2ConstellationsTxt[] = {
    "QPSK",
    "8PSK",
    "16APSK",
    "32APSK",
};
const char* s2CoderatesTxt[] = {
    "1/4",
    "1/3",
    "2/5",
    "1/2",
    "3/5",
    "2/3",
    "3/4",
    "4/5",
    "5/6",
    "7/8",
    "8/9",
    "9/10",
};
const char* s2FramesizesTxt[] = {
    "NORM",
    "SHORT",
};
const char* s2PilotsTxt[] = {
    "-",
    "+",
};

#define DVBS2_DEMOD_SOF_THRES 0.6f
#define DVBS2_DEMOD_LDPC_RETRIES 25
#define CLOCK_RECOVERY_BW 0.002f
#define CLOCK_RECOVERY_DAMPN_F 0.707f
#define CLOCK_RECOVERY_REL_LIM 0.02f
#define RRC_TAP_COUNT 65
#define RRC_ALPHA 0.35f
#define AGC_RATE 0.0004f
#define COSTAS_LOOP_BANDWIDTH 0.03f
#define FLL_LOOP_BANDWIDTH 0.0001f
#define FREQ_PROPAGATION_S2 0.002f

class DVBSDemodulatorModule : public ModuleManager::Instance {
public:
    DVBSDemodulatorModule(std::string name) {
        this->name = name;

        for (int i = 0; i < 4; i++) {
            s2ConstellationsListTxt += s2ConstellationsTxt[i];
            s2ConstellationsListTxt += '\0';
        }
        for (int i = 0; i < 12; i++) {
            s2CoderatesListTxt += s2CoderatesTxt[i];
            s2CoderatesListTxt += '\0';
        }
        for (int i = 0; i < 2; i++) {
            s2FramesizesListTxt += s2FramesizesTxt[i];
            s2FramesizesListTxt += '\0';
        }
        for (int i = 0; i < 2; i++) {
            s2PilotsListTxt += s2PilotsTxt[i];
            s2PilotsListTxt += '\0';
        }

        // Load config
        config.acquire();
        if (!config.conf.contains(name) || !config.conf[name].contains("hostname")) {
            config.conf[name]["hostname"] = "localhost";
            config.conf[name]["port"] = 8355;
            config.conf[name]["sending"] = false;
            config.conf[name]["dvbs_version"] = dvbs_ver_selected;
            config.conf[name]["dvbs_symrate"] = dvbs_sym_rate;
            config.conf[name]["dvbs2_symrate"] = dvbs2_sym_rate;
            config.conf[name]["dvbs2_constellation"] = dsp::dvbs2::MOD_QPSK;
            config.conf[name]["dvbs2_coderate"] = dsp::dvbs2::C1_2;
            config.conf[name]["dvbs2_framesize"] = dsp::dvbs2::FECFRAME_SHORT;
            config.conf[name]["dvbs2_pilots"] = false;
            config.conf[name]["dvbs2_automodcod"] = false;
            config.conf[name]["dvbs_bandwidth"] = dvbs_bw;
            config.conf[name]["dvbs2_bandwidth"] = dvbs2_bw;
        }
        strcpy(hostname, std::string(config.conf[name]["hostname"]).c_str());
        port = config.conf[name]["port"];
        bool startNow = config.conf[name]["sending"];
        dvbs_sym_rate_disp = dvbs_sym_rate = config.conf[name]["dvbs_symrate"];
        dvbs2_sym_rate_disp = dvbs2_sym_rate =  config.conf[name]["dvbs2_symrate"];
        dvbs2_cfg.constellation = config.conf[name]["dvbs2_constellation"];
        dvbs2_cfg.coderate = config.conf[name]["dvbs2_coderate"];
        dvbs2_cfg.framesize = config.conf[name]["dvbs2_framesize"];
        dvbs2_cfg.pilots = true; //init later
        auto_modcod = config.conf[name]["dvbs2_automodcod"];
        dvbs_bw = config.conf[name]["dvbs_bandwidth"];
        dvbs2_bw = config.conf[name]["dvbs2_bandwidth"];
        config.release(true);

        vfo = sigpath::vfoManager.createVFO(name, ImGui::WaterfallVFO::REF_CENTER, 0, dvbs_bw, dvbs_sym_rate*2.0f, 1000.0f, dvbs_sym_rate*2.5f, false);
        onUserChangedBandwidthHandler.handler = vfoUserChangedBandwidthHandler;
        onUserChangedBandwidthHandler.ctx = this;
        vfo->wtfVFO->onUserChangedBandwidth.bindHandler(&onUserChangedBandwidthHandler);
        //Clock recov coeffs
        float recov_bandwidth = CLOCK_RECOVERY_BW;
        float recov_dampningFactor = CLOCK_RECOVERY_DAMPN_F;
        float recov_denominator = (1.0f + 2.0*recov_dampningFactor*recov_bandwidth + recov_bandwidth*recov_bandwidth);
        float recov_mu = (4.0f * recov_dampningFactor * recov_bandwidth) / recov_denominator;
        float recov_omega = (4.0f * recov_bandwidth * recov_bandwidth) / recov_denominator;
        dvbsDemod.init(vfo->output, dvbs_sym_rate, dvbs_sym_rate*2, AGC_RATE, RRC_ALPHA, RRC_TAP_COUNT, COSTAS_LOOP_BANDWIDTH, FLL_LOOP_BANDWIDTH, recov_omega, recov_mu, _constDiagHandler, this, CLOCK_RECOVERY_REL_LIM);
        dvbs2Demod.init(vfo->output, dvbs2_sym_rate, dvbs2_sym_rate*2, AGC_RATE, RRC_ALPHA, RRC_TAP_COUNT, COSTAS_LOOP_BANDWIDTH, FLL_LOOP_BANDWIDTH, recov_omega, recov_mu, _constDiagHandler, this, dsp::dvbs2::get_dvbs2_modcod(dvbs2_cfg), (dvbs2_cfg.framesize == dsp::dvbs2::FECFRAME_SHORT), (dvbs2_cfg.pilots), DVBS2_DEMOD_SOF_THRES, DVBS2_DEMOD_LDPC_RETRIES, FREQ_PROPAGATION_S2, CLOCK_RECOVERY_REL_LIM);
        dvbs2_cfg.pilots = config.conf[name]["dvbs2_pilots"];
        updateS2Demod();
        demodSink.init(&dvbsDemod.out, _demodSinkHandler, this);

        setMode();

        if(startNow) {
            startNetwork();
        }

        gui::menu.registerEntry(name, menuHandler, this, this);
    }

    ~DVBSDemodulatorModule() {
        if(isEnabled()) {
            disable();
        }
        gui::menu.removeEntry(name);
        sigpath::sinkManager.unregisterStream(name);
    }

    void postInit() {}

    void enable() {
        vfo = sigpath::vfoManager.createVFO(name, ImGui::WaterfallVFO::REF_CENTER, 0, dvbs_sym_rate*2.0f, dvbs_sym_rate*2.0f, 1000.0f, dvbs_sym_rate*2.5f, false);
        vfo->wtfVFO->onUserChangedBandwidth.bindHandler(&onUserChangedBandwidthHandler);
        setMode();
        enabled = true;
    }

    void disable() {
        demodSink.stop();
        dvbsDemod.stop();
        dvbs2Demod.stop();
        sigpath::vfoManager.deleteVFO(vfo);
        enabled = false;
    }

    bool isEnabled() {
        return enabled;
    }

private:

    void startNetwork() {
        stopNetwork();
        try {
            conn = net::openudp(hostname, port);
        } catch (std::runtime_error& e) {
            flog::error("Network error: %s\n", e.what());
        }
    }

    void stopNetwork() {
        if (conn) { conn->close(); }
    }

    void setMode() {
        demodSink.stop();
        dvbsDemod.stop();
        dvbs2Demod.stop();
        if(dvbs_ver_selected == 0) {
            dvbsDemod.setInput(vfo->output);
            demodSink.setInput(&dvbsDemod.out);
            dvbsDemod.start();
            demodSink.start();
            setSymRate();
        } else {
            dvbs2Demod.setInput(vfo->output);
            demodSink.setInput(&dvbs2Demod.out);
            dvbs2Demod.start();
            demodSink.start();
            setSymRate();
        }
    }

    void setSymRate() {
        if(dvbs_ver_selected == 0) {
            if(dvbs_bw >= dvbs_sym_rate*2.0f) {
                dvbs_bw = dvbs_sym_rate*2.0f;
            }
            if(dvbs_bw <= dvbs_sym_rate/2.0f) {
                dvbs_bw = dvbs_sym_rate/2.0f;
            }
            vfo->setSampleRate(dvbs_sym_rate*2.0f, dvbs_bw);
            vfo->setBandwidthLimits(dvbs_sym_rate/2.0f, dvbs_sym_rate*2.5f, false);
            dvbsDemod.setSamplerate(dvbs_sym_rate*2);
            dvbsDemod.setSymbolrate(dvbs_sym_rate);
            dvbsDemod.reset();
        } else {
            if(dvbs2_bw >= dvbs2_sym_rate*2.0f) {
                dvbs2_bw = dvbs2_sym_rate*2.0f;
            }
            if(dvbs2_bw <= dvbs2_sym_rate/2.0f) {
                dvbs2_bw = dvbs2_sym_rate/2.0f;
            }
            vfo->setSampleRate(dvbs2_sym_rate*2.0f, dvbs2_bw);
            vfo->setBandwidthLimits(dvbs2_sym_rate/2.0f, dvbs2_sym_rate*2.5f, false);
            dvbs2Demod.setSamplerate(dvbs2_sym_rate*2);
            dvbs2Demod.setSymbolrate(dvbs2_sym_rate);
            dvbs2Demod.reset();
        }
    }

    void updateS2Demod() {
        dvbs2_cfg = dsp::dvbs2::get_dvbs2_cfg(dsp::dvbs2::get_dvbs2_modcod(dvbs2_cfg), (dvbs2_cfg.framesize == dsp::dvbs2::FECFRAME_SHORT), (dvbs2_cfg.pilots));
        dvbs2Demod.setDemodParams(dsp::dvbs2::get_dvbs2_modcod(dvbs2_cfg), (dvbs2_cfg.framesize == dsp::dvbs2::FECFRAME_SHORT), (dvbs2_cfg.pilots), DVBS2_DEMOD_SOF_THRES, DVBS2_DEMOD_LDPC_RETRIES);
        dvbs2bbparser.setFrameSize(dvbs2Demod.getKBCH());
    }

    static void menuHandler(void* ctx) {
        DVBSDemodulatorModule* _this = (DVBSDemodulatorModule*)ctx;
        float menuWidth = ImGui::GetContentRegionAvail().x;

        if(!_this->enabled) {
            style::beginDisabled();
        }

        ImGui::BeginGroup();
        ImGui::Columns(2, CONCAT("DVBSModeColumns##_", _this->name), false);
        if (ImGui::RadioButton(CONCAT("DVBS-QPSK##_", _this->name), _this->dvbs_ver_selected == 0) && _this->dvbs_ver_selected != 0) {
            _this->dvbs_ver_selected = 0; //dvb-s
            _this->setMode();
            config.acquire();
            config.conf[_this->name]["dvbs_version"] = _this->dvbs_ver_selected;
            config.release(true);
        }
        ImGui::NextColumn();
        if (ImGui::RadioButton(CONCAT("DVBS2##_", _this->name), _this->dvbs_ver_selected == 1) && _this->dvbs_ver_selected != 1) {
            _this->dvbs_ver_selected = 1; //dvb-s2
            _this->setMode();
            config.acquire();
            config.conf[_this->name]["dvbs_version"] = _this->dvbs_ver_selected;
            config.release(true);
        }
        ImGui::Columns(1, CONCAT("EndDVBSModeColumns##_", _this->name), false);
        ImGui::EndGroup();

        bool netActive = (_this->conn && _this->conn->isOpen());
        if(netActive) { style::beginDisabled(); }
        if (ImGui::InputText(CONCAT("UDP ##_dvbsdemod_host_", _this->name), _this->hostname, 1023)) {
            config.acquire();
            config.conf[_this->name]["hostname"] = _this->hostname;
            config.release(true);
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
        if (ImGui::InputInt(CONCAT("##_dvbsdemod_port_", _this->name), &(_this->port), 0, 0)) {
            config.acquire();
            config.conf[_this->name]["port"] = _this->port;
            config.release(true);
        }
        if(netActive) { style::endDisabled(); }

        if (netActive && ImGui::Button(CONCAT("Net stop##_dvbsdemod_net_stop_", _this->name), ImVec2(menuWidth, 0))) {
            _this->stopNetwork();
            config.acquire();
            config.conf[_this->name]["sending"] = false;
            config.release(true);
        } else if (!netActive && ImGui::Button(CONCAT("Net start##_dvbsdemod_net_stop_", _this->name), ImVec2(menuWidth, 0))) {
            _this->startNetwork();
            config.acquire();
            config.conf[_this->name]["sending"] = true;
            config.release(true);
        }

        ImGui::TextUnformatted("Net status:");
        ImGui::SameLine();
        if (netActive) {
            if(_this->dvbs_ver_selected == 0) {
                if(_this->dvbsDemod.stats_viterbi_lock) {
                    ImGui::TextColored(ImVec4(0.0, 1.0, 0.0, 1.0), "Sending");
                } else {
                    ImGui::TextColored(ImVec4(1.0, 1.0, 0.0, 1.0), "Ready");
                }
            } else {
                if(_this->dvbs2bbparser.last_bb_proc > _this->dvbs2bbparser.last_bb_cnt/2) {
                    ImGui::TextColored(ImVec4(0.0, 1.0, 0.0, 1.0), "Sending");
                } else {
                    ImGui::TextColored(ImVec4(1.0, 1.0, 0.0, 1.0), "Ready");
                }
            }
        } else {
            ImGui::TextUnformatted("Idle");
        }
        if(_this->dvbs_ver_selected == 0) {
            ImGui::Text("Symbol rate: ");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
            if (ImGui::InputInt(CONCAT("##_dvbsdemod_rate_", _this->name), &(_this->dvbs_sym_rate_disp), 1, 10)) {
            }
            if (ImGui::Button(CONCAT("Apply##_dvbsdemod_rate_a_", _this->name))) {
                _this->dvbs_sym_rate = _this->dvbs_sym_rate_disp;
                if(_this->dvbs_sym_rate > 0) {
                    _this->setSymRate();
                }
                config.acquire();
                config.conf[_this->name]["dvbs_symrate"] = _this->dvbs_sym_rate;
                config.release(true);
            }
            _this->dvbs_viterbi_err_avg[_this->dvbs_viterbi_err_avg_ptr] = _this->dvbsDemod.stats_viterbi_ber;
            _this->dvbs_viterbi_err_avg_ptr++;
            if(_this->dvbs_viterbi_err_avg_ptr >= 30) _this->dvbs_viterbi_err_avg_ptr = 0;
            float avg_viterbi_err = 0;
            for(int i = 0; i < 30; i++) {
                avg_viterbi_err += _this->dvbs_viterbi_err_avg[i];
            }
            avg_viterbi_err /= 0.3f;
            avg_viterbi_err = 100.0f - avg_viterbi_err;
            // ImGui::Text("Viterbi BER: ");ImGui::SameLine();ImGui::TextColored(ImVec4((avg_viterbi_err > 15.0f ? 1.0 : 0.0), (avg_viterbi_err > 15.0f ? 0.0 : 1.0), 0.0, 1.0), "%4.2f %%", avg_viterbi_err);
            ImGui::Text("Viterbi sig lvl: ");
            ImGui::SameLine();
            ImGui::SigQualityMeter(avg_viterbi_err, 60.0f, 100.0f);
            // ImGui::BoxIndicator(GImGui->FontSize*2, _this->dvbsDemod.stats_viterbi_lock ? IM_COL32(5, 230, 5, 255) : IM_COL32(230, 5, 5, 255));
            // ImGui::SameLine();
            // ImGui::Text("Viterbi locked: ");ImGui::SameLine();ImGui::TextColored(ImVec4((_this->dvbsDemod.stats_viterbi_lock ? 0.0 : 1.0), (_this->dvbsDemod.stats_viterbi_lock ? 1.0 : 0.0), 0.0, 1.0), "%s", _this->dvbsDemod.stats_viterbi_lock ? "Y" : "N");
            if(!_this->dvbsDemod.stats_viterbi_lock) {
                style::beginDisabled();
            }
            ImGui::Text("Viterbi detected rate: %s", _this->dvbsDemod.stats_viterbi_rate.c_str());
            ImGui::Text("Deframer errors: %d", _this->dvbsDemod.stats_deframer_err);
            ImGui::Text("Reed-solomon avg errors: %f", _this->dvbsDemod.stats_rs_avg);
            if(!_this->dvbsDemod.stats_viterbi_lock) {
                style::endDisabled();
            }
        } else {
            ImGui::Text("Symbol rate: ");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
            if (ImGui::InputInt(CONCAT("##_dvbs2demod_rate_", _this->name), &(_this->dvbs2_sym_rate_disp), 1, 10)) {
                
            }
            if (ImGui::Button(CONCAT("Apply##_dvbs2demod_rate_a_", _this->name))) {
                _this->dvbs2_sym_rate = _this->dvbs2_sym_rate_disp;
                if(_this->dvbs2_sym_rate > 0) {
                    _this->setSymRate();
                }
                config.acquire();
                config.conf[_this->name]["dvbs2_symrate"] = _this->dvbs2_sym_rate;
                config.release(true);
            }
            dsp::dvbs2::dvb_cgf_holder modcod_det;
            modcod_det.constellation = dsp::dvbs2::MOD_QPSK;
            modcod_det.coderate = dsp::dvbs2::C1_4;
            modcod_det.pilots = 0;
            modcod_det.framesize = dsp::dvbs2::FECFRAME_NORMAL;
            if(_this->dvbs2Demod.detected_modcod > 0 && _this->dvbs2Demod.detected_modcod < 29) {
                modcod_det = dsp::dvbs2::get_dvbs2_cfg(_this->dvbs2Demod.detected_modcod, _this->dvbs2Demod.detected_shortframes, _this->dvbs2Demod.detected_pilots);
            }
            _this->dvbs2_modcod_checkbuff[_this->dvbs2_modcod_checkbuff_ptr] = _this->dvbs2Demod.detected_modcod;
            _this->dvbs2_modcod_checkbuff_ptr++;
            if(_this->dvbs2_modcod_checkbuff_ptr >= 50) {
                _this->dvbs2_modcod_checkbuff_ptr = 0;
            }
            bool modcod_consistent = true;
            int modcod_check = _this->dvbs2_modcod_checkbuff[0];
            for(int i = 0; i < 50; i++) {
                if(_this->dvbs2_modcod_checkbuff[i] != modcod_check || _this->dvbs2_modcod_checkbuff[i] <= 0 || _this->dvbs2_modcod_checkbuff[i] >= 29) {
                    modcod_consistent = false;
                    break;
                }
            }
            if(_this->auto_modcod && modcod_consistent) {
                if(dsp::dvbs2::get_dvbs2_modcod(_this->dvbs2_cfg) != _this->dvbs2Demod.detected_modcod ||
                _this->dvbs2_cfg.pilots != _this->dvbs2Demod.detected_pilots) {
                    _this->dvbs2_cfg = dsp::dvbs2::get_dvbs2_cfg(dsp::dvbs2::get_dvbs2_modcod(modcod_det), (_this->dvbs2_cfg.framesize == dsp::dvbs2::FECFRAME_SHORT), (modcod_det.pilots));
                    _this->updateS2Demod();
                    config.acquire();
                    config.conf[_this->name]["dvbs2_constellation"] = _this->dvbs2_cfg.constellation;
                    config.conf[_this->name]["dvbs2_coderate"] = _this->dvbs2_cfg.coderate;
                    config.conf[_this->name]["dvbs2_framesize"] = _this->dvbs2_cfg.framesize;
                    config.conf[_this->name]["dvbs2_pilots"] = _this->dvbs2_cfg.pilots;
                    config.release(true);
                }
            }
            ImGui::Columns(2, CONCAT("DVBS2CFGCols##_", _this->name), false);
            ImGui::Text("Demod params");ImGui::NextColumn();ImGui::Text("Modcod detected");ImGui::NextColumn();
            ImGui::Text("Constellation:");ImGui::SameLine();
            if(ImGui::Combo(CONCAT("##_dvbs2_constellation_sel_", _this->name), (int*)&_this->dvbs2_cfg.constellation, _this->s2ConstellationsListTxt.c_str())) {
                _this->updateS2Demod();
                config.acquire();
                config.conf[_this->name]["dvbs2_constellation"] = _this->dvbs2_cfg.constellation;
                config.release(true);
            }
            ImGui::NextColumn();ImGui::TextColored((modcod_consistent ? ImVec4(0.0, 1.0, 0.0, 1.0) : ImVec4(1.0, 0.0, 0.0, 1.0)), "%s", s2ConstellationsTxt[modcod_det.constellation]);ImGui::NextColumn();
            ImGui::Text("Coderate:");ImGui::SameLine();
            if(ImGui::Combo(CONCAT("##_dvbs2_coderate_sel_", _this->name), (int*)&_this->dvbs2_cfg.coderate, _this->s2CoderatesListTxt.c_str())) {
                _this->updateS2Demod();
                config.acquire();
                config.conf[_this->name]["dvbs2_coderate"] = _this->dvbs2_cfg.coderate;
                config.release(true);
            }
            ImGui::NextColumn();ImGui::TextColored((modcod_consistent ? ImVec4(0.0, 1.0, 0.0, 1.0) : ImVec4(1.0, 0.0, 0.0, 1.0)),"%s", s2CoderatesTxt[modcod_det.coderate]);ImGui::NextColumn();
            ImGui::Text("Pilots:");ImGui::SameLine();
            if(ImGui::Checkbox(CONCAT("##_dvbs2_pilots_sel_", _this->name), &_this->dvbs2_cfg.pilots)) {
                _this->updateS2Demod();
                config.acquire();
                config.conf[_this->name]["dvbs2_pilots"] = _this->dvbs2_cfg.pilots;
                config.release(true);
            }
            ImGui::NextColumn();ImGui::TextColored((modcod_consistent ? ImVec4(0.0, 1.0, 0.0, 1.0) : ImVec4(1.0, 0.0, 0.0, 1.0)), "%s", s2PilotsTxt[modcod_det.pilots]);ImGui::NextColumn();
            ImGui::Text("Frames:");ImGui::SameLine();
            if(ImGui::Combo(CONCAT("##_dvbs2_frames_sel_", _this->name), (int*)&_this->dvbs2_cfg.framesize, _this->s2FramesizesListTxt.c_str())) {
                _this->updateS2Demod();
                config.acquire();
                config.conf[_this->name]["dvbs2_framesize"] = _this->dvbs2_cfg.framesize;
                config.release(true);
            }
            ImGui::NextColumn();ImGui::TextColored((modcod_consistent ? ImVec4(0.0, 1.0, 0.0, 1.0) : ImVec4(1.0, 0.0, 0.0, 1.0)), "%s", s2FramesizesTxt[modcod_det.framesize]);ImGui::NextColumn();
            ImGui::Columns(1, CONCAT("EndDVBS2CFGCols##_", _this->name), false);
            ImGui::Text("AUTO modcod:");ImGui::SameLine();
            if(ImGui::Checkbox(CONCAT("##_dvbs2_automodcod_", _this->name), &_this->auto_modcod)) {
                config.acquire();
                config.conf[_this->name]["dvbs2_automodcod"] = _this->auto_modcod;
                config.release(true);
            }

            _this->dvbs2_pl_best_avg[_this->dvbs2_pl_best_avg_ptr] = _this->dvbs2Demod.pl_sync_best_match;
            _this->dvbs2_pl_best_avg_ptr++;
            if(_this->dvbs2_pl_best_avg_ptr >= 30) _this->dvbs2_pl_best_avg_ptr = 0;
            float avg_bestmatch = 0;
            for(int i = 0; i < 30; i++) {
                avg_bestmatch += _this->dvbs2_pl_best_avg[i];
            }
            avg_bestmatch /= 0.3f;
            ImGui::Text("PLSync best match: %d", int(avg_bestmatch));
            ImGui::SameLine();
            ImGui::SigQualityMeter(avg_bestmatch, 20.0f, 100.0f);
            // ImGui::BoxIndicator(GImGui->FontSize*2, (avg_bestmatch >= DVBS2_DEMOD_SOF_THRES*100.0f) ? IM_COL32(5, 230, 5, 255) : IM_COL32(230, 5, 5, 255));
            if(!(avg_bestmatch >= DVBS2_DEMOD_SOF_THRES*100.0f)) {
                style::beginDisabled();
            }
            ImGui::Text("LDPC trials: %d / %d", int(_this->dvbs2Demod.ldpc_trials), DVBS2_DEMOD_LDPC_RETRIES);

            _this->dvbs2_bch_corr_avg[_this->dvbs2_bch_corr_avg_ptr] = _this->dvbs2Demod.bch_corrections;
            _this->dvbs2_bch_corr_avg_ptr++;
            if(_this->dvbs2_bch_corr_avg_ptr >= 30) _this->dvbs2_bch_corr_avg_ptr = 0;
            _this->dvbs2_avg_bchcorr = 0;
            for(int i = 0; i < 30; i++) {
                _this->dvbs2_avg_bchcorr += _this->dvbs2_bch_corr_avg[i];
            }
            _this->dvbs2_avg_bchcorr /= 0.3f;
            _this->dvbs2_avg_bchcorr = 100.0f - _this->dvbs2_avg_bchcorr*0.1;
            if(_this->dvbs2bbparser.last_bb_proc < _this->dvbs2bbparser.last_bb_cnt/2) {
                ImGui::Text("BCH sig quality: 0");
                ImGui::SameLine();
                ImGui::SigQualityMeter(0, 20.0f, 100.0f);
                // ImGui::BoxIndicator(GImGui->FontSize*2, IM_COL32(230, 5, 5, 255));
                ImGui::Text("BBF procesed: %d/%d", _this->dvbs2bbparser.last_bb_proc, _this->dvbs2bbparser.last_bb_cnt);
                if(_this->dvbs2bbparser.last_bb_cnt != 0) {
                    ImGui::SameLine();
                    ImGui::SigQualityMeter(100.0f*((float)_this->dvbs2bbparser.last_bb_proc)/((float)_this->dvbs2bbparser.last_bb_cnt), 0.0f, 100.0f);
                }
            } else {
                ImGui::Text("BCH sig quality: %d", int(_this->dvbs2_avg_bchcorr));
                ImGui::SameLine();
                ImGui::SigQualityMeter(_this->dvbs2_avg_bchcorr, 20.0f, 100.0f);
                // ImGui::BoxIndicator(GImGui->FontSize*2, IM_COL32(5, 230, 5, 255));
                ImGui::Text("BBF processed: %d/%d", _this->dvbs2bbparser.last_bb_proc, _this->dvbs2bbparser.last_bb_cnt);
                if(_this->dvbs2bbparser.last_bb_cnt != 0) {
                    ImGui::SameLine();
                    ImGui::SigQualityMeter(100.0f*((float)_this->dvbs2bbparser.last_bb_proc)/((float)_this->dvbs2bbparser.last_bb_cnt), 0.0f, 100.0f);
                }
                ImGui::Text("BBFrame content: %s", (_this->dvbs2bbparser.last_header.ts_gs == 0b11 ? "MPEGTS" : (_this->dvbs2bbparser.last_header.ts_gs == 0b01 ? "GSE" : "UNK")));
                ImGui::Text("Input stream: %s", (_this->dvbs2bbparser.last_header.sis_mis ? "Single" : "Multiple"));
                ImGui::Text("Cod&Mod: %s", (_this->dvbs2bbparser.last_header.ccm_acm ? "CCM" : "ACM"));
                ImGui::Text("ISSY: %s", (_this->dvbs2bbparser.last_header.issyi ? "Y" : "N"));
                ImGui::Text("NPD: %s", (_this->dvbs2bbparser.last_header.npd ? "Y" : "N"));
                ImGui::Text("RO: %s", (_this->dvbs2bbparser.last_header.ro == 0b00 ? "0.35" : (_this->dvbs2bbparser.last_header.ro == 0b01 ? "0.25" : "0.20")));
            }
            if(!(avg_bestmatch >= DVBS2_DEMOD_SOF_THRES*100.0f)) {
                style::endDisabled();
            }
        }
        ImGui::Text("Signal constellation: ");
        ImGui::SetNextItemWidth(menuWidth);
        _this->constDiag.draw();

        if(!_this->enabled) {
            style::endDisabled();
        }
    }

    static void _constDiagHandler(dsp::complex_t* data, int count, void* ctx) {
        DVBSDemodulatorModule* _this = (DVBSDemodulatorModule*)ctx;
        int curidx = 0;
        while(count > 0) {
            int copyCnt = std::min(count, (1024-_this->constDiagBuffPtr));
            memcpy(&_this->constDiagBuff[_this->constDiagBuffPtr], &data[curidx], copyCnt * sizeof(dsp::complex_t));
            curidx += copyCnt;
            _this->constDiagBuffPtr += copyCnt;
            count -= copyCnt;
            if(_this->constDiagBuffPtr >= 1024) {
                dsp::complex_t* cdBuff = _this->constDiag.acquireBuffer();
                memcpy(cdBuff, _this->constDiagBuff, 1024 * sizeof(dsp::complex_t));
                _this->constDiag.releaseBuffer();
                _this->constDiagBuffPtr = 0;
            }
        }
    }

    static void _demodSinkHandler(uint8_t* data, int count, void* ctx) {
        DVBSDemodulatorModule* _this = (DVBSDemodulatorModule*)ctx;
        if(_this->dvbs_ver_selected == 0) {
            if(_this->conn && _this->conn->isOpen())
                _this->conn->send(data, count);
        } else {
            // for(int i = 0; i < count; i += _this->dvbs2Demod.getKBCH()/8) {
            //     int cnt = _this->dvbs2bbparser.work(&data[i], 1, _this->packetbuff, 65536);
            //     if(_this->dvbs2bbparser.last_header.ts_gs == 0b11) {
            //         //MPEGTS
            //         // if(cnt > 0 && (cnt%188)==0) {
            //         //     for(int k = 0; k < cnt/188; k++) {
            //         //         if(_this->conn && _this->conn->isOpen())
            //         //             _this->conn->send(&_this->packetbuff[188*k], 188);
            //         //     }
            //         // }
            //         if(cnt > 0) {
            //             if(_this->conn && _this->conn->isOpen()) 
            //                 _this->conn->send(_this->packetbuff, cnt);
            //         }
            //     } else {
            //         //GSE/OTHER
            //         if(cnt > 0) {
            //             if(_this->conn && _this->conn->isOpen()) 
            //                 _this->conn->send(_this->packetbuff, cnt);
            //         }
            //     }
            // }
            int cnt = _this->dvbs2bbparser.work(data, count/(_this->dvbs2Demod.getKBCH()/8), _this->packetbuff, 65536*10);
            if(_this->dvbs2bbparser.last_header.ts_gs == 0b11) {
                //MPEGTS
                if(cnt > 0 && (cnt%188)==0) {
                    for(int k = 0; k < cnt/1880; k++) {
                        if(_this->conn && _this->conn->isOpen())
                            _this->conn->send(&_this->packetbuff[1880*k], 1880);
                    }
                    int rem = cnt%1880;
                    if(rem != 0 && _this->conn && _this->conn->isOpen())
                            _this->conn->send(&_this->packetbuff[cnt-rem], rem);
                }
                // if(cnt > 0) {
                    // if(_this->conn && _this->conn->isOpen()) 
                        // _this->conn->send(_this->packetbuff, cnt);
                // }
            } else {
                //GSE/OTHER
                if(cnt > 0) {
                    if(_this->conn && _this->conn->isOpen()) 
                        _this->conn->send(_this->packetbuff, cnt);
                }
            }
        }
    }

    static void vfoUserChangedBandwidthHandler(double newBw, void* ctx) {
        DVBSDemodulatorModule* _this = (DVBSDemodulatorModule*)ctx;
        _this->vfo->setBandwidth(newBw);
        if(_this->dvbs_ver_selected == 0) {
            _this->dvbs_bw = newBw;
            config.acquire();
            config.conf[_this->name]["dvbs_bandwidth"] = _this->dvbs_bw;
            config.release(true);
        } else {
            _this->dvbs2_bw = newBw;
            config.acquire();
            config.conf[_this->name]["dvbs2_bandwidth"] = _this->dvbs2_bw;
            config.release(true);
        }
    }

    std::string name;
    bool enabled = true;

    int dvbs_ver_selected = 0;

    VFOManager::VFO* vfo;
    float dvbs_bw = 250000*2.0f;
    float dvbs2_bw = 250000*2.0f;
    int constDiagBuffPtr = 0;
    dsp::complex_t constDiagBuff[1024];
    ImGui::ConstellationDiagram constDiag;

    int dvbs_sym_rate = 250000;
    int dvbs_sym_rate_disp = 250000;
    dsp::dvbs::DVBSDemod dvbsDemod;
    float dvbs_viterbi_err_avg[30];
    int dvbs_viterbi_err_avg_ptr = 0;

    int dvbs2_sym_rate = 250000;
    int dvbs2_sym_rate_disp = 250000;
    dsp::dvbs2::dvb_cgf_holder dvbs2_cfg;
    dsp::dvbs2::DVBS2Demod dvbs2Demod;
    float dvbs2_pl_best_avg[30];
    int dvbs2_pl_best_avg_ptr = 0;
    float dvbs2_bch_corr_avg[30];
    int dvbs2_bch_corr_avg_ptr = 0;
    float dvbs2_avg_bchcorr = 0;
    int dvbs2_modcod_checkbuff[50];
    int dvbs2_modcod_checkbuff_ptr = 0;
    bool auto_modcod = false;
    dsp::dvbs2::BBFrameTSParser dvbs2bbparser;
    uint8_t packetbuff[65536*10];

    dsp::sink::Handler<uint8_t> demodSink;

    EventHandler<double> onUserChangedBandwidthHandler;

    std::string s2ConstellationsListTxt;
    std::string s2CoderatesListTxt;
    std::string s2FramesizesListTxt;
    std::string s2PilotsListTxt;

    char hostname[1024] = "127.0.0.1";
    int port = 4754;

    std::shared_ptr<net::Socket> conn;

};

MOD_EXPORT void _INIT_() {
    std::string root = (std::string)core::args["root"];
    json def = json({});
    config.setPath(root + "/dvbs_demodulator_config.json");
    config.load(def);
    config.enableAutoSave();
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new DVBSDemodulatorModule(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    delete (DVBSDemodulatorModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
