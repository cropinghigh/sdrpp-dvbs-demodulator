cmake_minimum_required(VERSION 3.13)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
project(dvbs_demodulator)

set(CMAKE_INSTALL_PREFIX "/usr/")
include(GNUInstallDirs)

file(GLOB SRC LIST_DIRECTORIES false
    "src/*.cpp"
    "src/*.c"
    "src/demod/common/codings/*.cpp"
    "src/demod/common/correct/*.cpp"
    "src/demod/common/correct/reed-solomon/*.c"
    "src/demod/common/dsp/demod/*.cpp"
    "src/demod/common/*.cpp"
    # "src/demod/codings/*.cpp"
    "src/demod/dvbs2/codings/*.cpp"
    "src/demod/dvbs2/codings/bch/*.cpp"
    "src/demod/dvbs2/codings/xdsopl-ldpc-pabr/*.cc"
    "src/demod/dvbs2/*.cpp"
    "src/demod/dvbs/*.cpp"
    "src/demod/dvbs/viterbi/*.cpp"
    )

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")

if (NOT SDRPP_MODULE_CMAKE)
    set(SDRPP_MODULE_CMAKE "/usr/share/cmake/Modules/sdrpp_module.cmake")
endif ()

include(${SDRPP_MODULE_CMAKE})

target_include_directories(dvbs_demodulator PRIVATE "src/" "src/demod")

# SSE41
if(NOT ANDROID)
    include(FindSSE41)
endif()

if(ANDROID)
    set(SSE41_FOUND 0)

    if(ANDROID_ABI STREQUAL "x86")
        set(SSE41_FOUND 1)
    endif()

    if(ANDROID_ABI STREQUAL "x86_64")
        set(SSE41_FOUND 1)
    endif()
endif()

if(SSE41_FOUND)
    message("Found SSE features, enabling building SSE code for DVB")

    if(UNIX OR ANDROID)
        string(REPLACE " " ";" FINAL_FLAGS_SSE41 "${CMAKE_CXX_FLAGS} -msse4.1")
    elseif(MSVC OR BUILD_MSVC)
        string(REPLACE " " ";" FINAL_FLAGS_SSE41 "${CMAKE_CXX_FLAGS} ${SSE41_C_FLAGS}")
    endif()

    target_compile_options(dvbs_demodulator PUBLIC ${FINAL_FLAGS_SSE41})
else()
    message("SSE Features not found! This is only an error on x86 CPUs that should support SSE4.1")
endif()
