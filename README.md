# sdrpp-dvbs-demodulator
DVB-S/DVB-S2 demodulator module for SDR++

Designed to fully demodulate and decode DVB-S/DVB-S2 signals and send TS frames/GSE packets in GRE over UDP

Module code is mainly using SatDump's DVB plugin, along with parts from gr-dvbs2rx and sdrangel. Thanks to authors of respective programs, you did a great work!

Signal chain:

DVB-S

VFO->QPSK Demodulator(AGC->FLL->RRC->Maximum Likelihood(y[n]y'[n]) timing recovery->Costas loop)->Samples to soft->Viterbi decoder->Deframer->Deinterleaver->Descrambler->UDP sender

DVB-S2

VFO->AGC->FLL->RRC->Maximum Likelihood(y[n]y'[n]) timing recovery->PL Sync->PLL->Symbols to soft->LDPC decoder->BCH decoder->Descrambler->Frame parser->GSE to GRE packer/TS reassembly->UDP sender
           ^                                                                 |
           |-------------------FREQUENCY FEEDBACK----------------------------|

Building:

  1.  Install SDR++ core headers to /usr/include/sdrpp_core/, if not installed (sdrpp-headers-git package for arch-like systems)

          git clone https://github.com/AlexandreRouma/SDRPlusPlus.git
          cd "SDRPlusPlus/core/src"
          sudo mkdir -p "/usr/include/sdrpp_core"
          sudo find . -regex ".*\.\(h\|hpp\)" -exec cp --parents \{\} "/usr/include/sdrpp_core" \;

  2.  Build:

          mkdir build
          cd build
          cmake ..
          make
          sudo make install

  4.  Enable new module in Module manager

Usage:

  1.  Select DVB-S or DVB-S2 mode

  2.  Move demodulator VFO to the center of signal you want to receive

  3.  Use automatic MODCOD detector(can be unstable or give wrong results) or configure DVB-S2 demod according to your signal

  4.  Wait for the decoders to start working

  5.  Start network sending and use your TS frames/GRE over udp in any software you need!

