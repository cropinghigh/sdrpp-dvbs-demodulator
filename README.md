# sdrpp-dvbs-demodulator
DVB-S/DVB-S2 demodulator module for SDR++

Designed to fully demodulate and decode DVB-S/DVB-S2 signals and send TS frames/GSE packets in GRE over UDP

Module code is mainly using SatDump's DVB plugin, along with parts from gr-dvbs2rx and sdrangel. Thanks to authors of respective programs, you did a great work!

Signal chain:

DVB-S

VFO->QPSK Demodulator(AGC->FLL->RRC->Maximum Likelihood(y[n]y'[n]) timing recovery->Costas loop)->Samples to soft->Viterbi decoder->Deframer->Deinterleaver->Descrambler->UDP sender

DVB-S2

           |-------------------FREQUENCY FEEDBACK----------------------------|

           |                                                                 ^

VFO->AGC->FLL->RRC->Maximum Likelihood(y[n]y'[n]) timing recovery->PL Sync->PLL->Symbols to soft->LDPC decoder->BCH decoder->Descrambler->Frame parser->GSE to GRE packer/TS reassembly->UDP sender

Binary installing:

Visit the Actions page, find latest commit build artifacts, download dvbs_demodulator.so and put it to /usr/lib/sdrpp/plugins/, skipping to the step 3.

Building:

  1.  Install SDR++ core headers to /usr/include/sdrpp_core/, if not installed. Refer to https://cropinghigh.github.io/sdrpp-moduledb/headerguide.html about how to do that

      OR if you don't want to use my header system, add -DSDRPP_MODULE_CMAKE="/path/to/sdrpp_build_dir/sdrpp_module.cmake" to cmake launch arguments

  2.  Build:

          mkdir build
          cd build
          cmake ..
          make
          sudo make install

  3.  Enable new module in Module manager

Usage:

  1.  Select DVB-S or DVB-S2 mode

  2.  Move demodulator VFO to the center of signal you want to receive

  3.  Use automatic MODCOD detector(can be unstable or give wrong results) or configure DVB-S2 demod according to your signal

  4.  Wait for the decoders to start working

  5.  Start network sending and use your TS frames/GRE over udp in any software you need!

