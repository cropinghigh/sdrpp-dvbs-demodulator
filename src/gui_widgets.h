#pragma once
#include <imgui/imgui.h>

#ifndef IMGUI_DEFINE_MATH_OPERATORS
#define IMGUI_DEFINE_MATH_OPERATORS
#endif
#include <imgui/imgui_internal.h>

namespace ImGui {
    ImVec2 operator+(ImVec2 a, ImVec2 b) {
        return ImVec2(a.x + b.x, a.y + b.y);
    }

    #define CONSTDIAG_SIZE (uint32_t)70000
    class DVBS_ConstellationDiagram {
    public:
        DVBS_ConstellationDiagram() {
            memset(buffer, 0, 1024 * sizeof(dsp::complex_t));
        }

        void draw(const ImVec2& size_arg = ImVec2(0, 0)) {
             std::lock_guard<std::mutex> lck(bufferMtx);
            ImGuiWindow* window = GetCurrentWindow();
            ImGuiStyle& style = GetStyle();
            ImVec2 min = window->DC.CursorPos;
            ImVec2 size = CalcItemSize(size_arg, CalcItemWidth(), CalcItemWidth());
            ImRect bb(min, ImVec2(min.x + size.x, min.y + size.y));

            ItemSize(size, style.FramePadding.y);
            if (!ItemAdd(bb, 0)) {
                return;
            }

            window->DrawList->AddRectFilled(min, ImVec2(min.x + size.x, min.y + size.y), IM_COL32(0, 0, 0, 255));
            ImU32 col = ImGui::GetColorU32(ImGuiCol_CheckMark, 0.7f);
            ImU32 col_red = ImGui::ColorConvertFloat4ToU32(ImVec4(0.9f,0.01f,0.01f,1.f));
            for (int i = 0; i < used_syms; i++) {
                if (buffer[i].re > 1.5f || buffer[i].re < -1.5f) { continue; }
                if (buffer[i].im > 1.5f || buffer[i].im < -1.5f) { continue; }
                window->DrawList->AddCircleFilled(ImVec2((((buffer[i].re / 1.5f) + 1) * (size.x * 0.5f)) + min.x, (((buffer[i].im / 1.5f) + 1) * (size.y * 0.5f)) + min.y), 2, col);
            }
            for (int i = 0; i < used_red_syms; i++) {
                if (red_buffer[i].re > 1.5f || red_buffer[i].re < -1.5f) { continue; }
                if (red_buffer[i].im > 1.5f || red_buffer[i].im < -1.5f) { continue; }
                window->DrawList->AddCircleFilled(ImVec2((((red_buffer[i].re / 1.5f) + 1) * (size.x * 0.5f)) + min.x, (((red_buffer[i].im / 1.5f) + 1) * (size.y * 0.5f)) + min.y), 2, col_red);
            }
        }

        dsp::complex_t* acquireBuffer() {
            bufferMtx.lock();
            return buffer;
        }

        void releaseBuffer(uint32_t usedSyms) {
            used_syms = std::min(usedSyms, CONSTDIAG_SIZE);
            bufferMtx.unlock();
        }

        dsp::complex_t* acquireRedBuffer() {
            bufferMtx.lock();
            return red_buffer;
        }

        void releaseRedBuffer(uint32_t usedSyms) {
            used_red_syms = std::min(usedSyms, CONSTDIAG_SIZE);
            bufferMtx.unlock();
        }

    private:
        std::mutex bufferMtx;
        dsp::complex_t buffer[CONSTDIAG_SIZE];
        dsp::complex_t red_buffer[CONSTDIAG_SIZE];
        uint32_t used_syms = 0;
        uint32_t used_red_syms = 0;
    };

    void BoxIndicator(float menuWidth, ImU32 color, const ImVec2& size_arg = ImVec2(0, 0)) {
        ImGuiWindow* window = GetCurrentWindow();
        ImGuiStyle& style = GImGui->Style;

        ImVec2 min = window->DC.CursorPos;
        min.x = menuWidth - (GImGui->FontSize);
        ImVec2 size = CalcItemSize(size_arg, (GImGui->FontSize), (GImGui->FontSize));
        ImRect bb(min, min+size);

        float lineHeight = size.y;

        ItemSize(size, style.FramePadding.y);
        if (!ItemAdd(bb, 0)) {
            return;
        }

        window->DrawList->AddRectFilled(min, min+size, color);
    }

    void SigQualityMeter(float avg, float val_min, float val_max, const ImVec2& size_arg = ImVec2(0, 0)) {
        ImGuiWindow* window = GetCurrentWindow();
        ImGuiStyle& style = GImGui->Style;

        avg = std::clamp<float>(avg, val_min, val_max);

        ImVec2 min = window->DC.CursorPos;
        ImVec2 size = CalcItemSize(size_arg, CalcItemWidth(), (GImGui->FontSize / 2) + style.FramePadding.y);
        ImRect bb(min, min + size);

        float lineHeight = size.y;

        ItemSize(size, style.FramePadding.y);
        if (!ItemAdd(bb, 0)) {
            return;
        }

        float badSig = roundf(0.3f * size.x);

        window->DrawList->AddRectFilled(min, min + ImVec2(badSig, lineHeight), IM_COL32(136, 9, 9, 255));
        window->DrawList->AddRectFilled(min + ImVec2(badSig, 0), min + ImVec2(size.x, lineHeight), IM_COL32(9, 136, 9, 255));

        float end = roundf(((avg - val_min) / (val_max - val_min)) * size.x);

        if (avg <= (val_min+(val_max-val_min)*0.3f)) {
            window->DrawList->AddRectFilled(min, min + ImVec2(end, lineHeight), IM_COL32(230, 5, 5, 255));
        }
        else {
            window->DrawList->AddRectFilled(min, min + ImVec2(badSig, lineHeight), IM_COL32(230, 5, 5, 255));
            window->DrawList->AddRectFilled(min + ImVec2(badSig, 0), min + ImVec2(end, lineHeight), IM_COL32(5, 230, 5, 255));
        }
    }
} 
