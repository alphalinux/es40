/* ES40 emulator.
 * Copyright (C) 2026 by Matt Turner
 * All rights reserved.
 *
 * WWW    : https://github.com/ES40-Emu/es40
 *
 * SPDX-License-Identifier: BSD-1-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Philips PCF8584 I2C bus controller, as on the API UP2000 (Swordfish),
 * where it sits at PCI memory 0xfff80000 (S0) and 0xfff80001 (S1).
 *
 * Only master mode is modeled, and no slaves are attached: every address
 * byte goes unacknowledged.  That is what the UP2000 SRM console expects
 * of a board without the CS20's PCF8574.  Without the controller, reads
 * return all-ones, which the console takes as lost arbitration, and it
 * retries the probe forever.
 */

#pragma once
#include "datatypes.h"

class CPCF8584
{
public:
  static bool claims(u64 pci_addr) { return (pci_addr & ~U64(1)) == U64(0xfff80000); }

  u8 read(u64 pci_addr)
  {
    finish_rx();
    if (pci_addr & 1)
      return (u8)((pin << 7) | (lrb << 3) | (lab << 1) | bb_n);
    u8 v = s0;
    if (eso && mode == RX)
      rx_pending = true;  // reading S0 starts the next byte
    return v;
  }

  void write(u64 pci_addr, u8 v)
  {
    if (pci_addr & 1)
    {
      eso = (v >> 6) & 1;
      es1 = (v >> 5) & 1;
      es2 = (v >> 4) & 1;
      ack = v & 1;
      const bool sta = (v >> 2) & 1, sto = (v >> 1) & 1;
      if (!eso)
        reset();
      else if (sta && !sto)
        start();
      else if (sto && !sta)
        stop();
      return;
    }
    if (!eso)
    {
      if (es1)
        s2 = v;
      else if (es2)
        s3 = v;
      else
        own = v;
      return;
    }
    finish_rx();
    s0 = v;
    if (mode == TX)
    {
      lrb = 1;  // no slave acknowledges
      pin = 0;
    }
  }

private:
  enum { IDLE, TX, RX } mode = IDLE;
  u8 s0 = 0, own = 0, s2 = 0, s3 = 0;
  int eso = 0, es1 = 0, es2 = 0, ack = 0;
  int pin = 1, lrb = 0, lab = 0, bb_n = 1;
  bool rx_pending = false;

  void reset()
  {
    mode = IDLE;
    pin = 1;
    lrb = lab = 0;
    bb_n = 1;
    rx_pending = false;
  }

  // send the address byte in S0; nobody answers it
  void start()
  {
    bb_n = 0;
    mode = (s0 & 1) ? RX : TX;
    lrb = 1;
    pin = 0;
  }

  void stop()
  {
    mode = IDLE;
    bb_n = 1;
    pin = 1;
    rx_pending = false;
  }

  // a received byte is acknowledged as the ACK bit stands when it ends
  void finish_rx()
  {
    if (!rx_pending)
      return;
    rx_pending = false;
    s0 = 0xff;
    lrb = !ack;
    pin = 0;
  }
};
