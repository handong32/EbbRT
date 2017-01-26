//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#include "IxgbeDriver.h"

#include "../StaticIOBuf.h"
#include "../UniqueIOBuf.h"
#include "Clock.h"
#include "Debug.h"
#include "EventManager.h"
#include "Ixgbe.h"

void ebbrt::IxgbeDriver::Create(pci::Device& dev) {
  auto ixgbe_dev = new IxgbeDriver(dev);
  ixgbe_dev->Init();
}

void ebbrt::IxgbeDriver::Init() {
  ebbrt::kprintf("%s ", __PRETTY_FUNCTION__);
  bar0_.Map();
  ebbrt::clock::SleepMilli(200);
  ebbrt::kprintf("Sleep 200 ms\n");
  // DeviceInfo();
  // InitStruct();
  StopDevice();
}

void ebbrt::IxgbeDriver::InitStruct() {
  struct IxgbeRegs* r = static_cast<struct IxgbeRegs*>(bar0_.GetVaddr());

  ebbrt::kprintf(
      "0x00000: CTRL        (Device Control)                 0x%08X\n",
      r->kIxgbeCtrl);

  ebbrt::kprintf(
      "0x00008: STATUS      (Device Status)                  0x%08X\n",
      r->kIxgbeStatus);
}

void ebbrt::IxgbeDriver::DeviceInfo() {
  uint32_t reg;

  reg = bar0_.Read32(0x042A4);
  ebbrt::kprintf(
      "0x042A4: LINKS (Link Status register)                 0x%08X\n"
      "       Link Status:                                   %s\n"
      "       Link Speed:                                    %s\n",
      reg, reg & IXGBE_LINKS_UP ? "up" : "down",
      reg & IXGBE_LINKS_SPEED ? "10G" : "1G");

  reg = bar0_.Read32(0x05080);
  ebbrt::kprintf(
      "0x05080: FCTRL (Filter Control register)              0x%08X\n"
      "       Broadcast Accept:                              %s\n"
      "       Unicast Promiscuous:                           %s\n"
      "       Multicast Promiscuous:                         %s\n"
      "       Store Bad Packets:                             %s\n",
      reg, reg & IXGBE_FCTRL_BAM ? "enabled" : "disabled",
      reg & IXGBE_FCTRL_UPE ? "enabled" : "disabled",
      reg & IXGBE_FCTRL_MPE ? "enabled" : "disabled",
      reg & IXGBE_FCTRL_SBP ? "enabled" : "disabled");

  reg = bar0_.Read32(0x04294);
  ebbrt::kprintf(
      "0x04294: MFLCN (TabMAC Flow Control register)         0x%08X\n"
      "       Receive Flow Control Packets:                  %s\n"
      "       Discard Pause Frames:                          %s\n"
      "       Pass MAC Control Frames:                       %s\n"
      "       Receive Priority Flow Control Packets:         %s\n",
      reg, reg & IXGBE_MFLCN_RFCE ? "enabled" : "disabled",
      reg & IXGBE_FCTRL_DPF ? "enabled" : "disabled",
      reg & IXGBE_FCTRL_PMCF ? "enabled" : "disabled",
      reg & IXGBE_FCTRL_RPFCE ? "enabled" : "disabled");

  reg = bar0_.Read32(0x05088);
  ebbrt::kprintf(
      "0x05088: VLNCTRL (VLAN Control register)              0x%08X\n"
      "       VLAN Mode:                                     %s\n"
      "       VLAN Filter:                                   %s\n",
      reg, reg & IXGBE_VLNCTRL_VME ? "enabled" : "disabled",
      reg & IXGBE_VLNCTRL_VFE ? "enabled" : "disabled");

  reg = bar0_.Read32(0x02100);
  ebbrt::kprintf(
      "0x02100: SRRCTL0 (Split and Replic Rx Control 0)      0x%08X\n"
      "       Receive Buffer Size:                           %uKB\n",
      reg, (reg & IXGBE_SRRCTL_BSIZEPKT_MASK) <= 0x10
               ? (reg & IXGBE_SRRCTL_BSIZEPKT_MASK)
               : 0x10);

  reg = bar0_.Read32(0x03D00);
  ebbrt::kprintf(
      "0x03D00: FCCFG (Flow Control Configuration)           0x%08X\n"
      "       Transmit Flow Control:                         %s\n"
      "       Priority Flow Control:                         %s\n",
      reg, reg & IXGBE_FCCFG_TFCE_802_3X ? "enabled" : "disabled",
      reg & IXGBE_FCCFG_TFCE_PRIORITY ? "enabled" : "disabled");

  reg = bar0_.Read32(0x04250);
  ebbrt::kprintf(
      "0x04250: HLREG0 (Highlander Control 0 register)       0x%08X\n"
      "       Transmit CRC:                                  %s\n"
      "       Receive CRC Strip:                             %s\n"
      "       Jumbo Frames:                                  %s\n"
      "       Pad Short Frames:                              %s\n"
      "       Loopback:                                      %s\n",
      reg, reg & IXGBE_HLREG0_TXCRCEN ? "enabled" : "disabled",
      reg & IXGBE_HLREG0_RXCRCSTRP ? "enabled" : "disabled",
      reg & IXGBE_HLREG0_JUMBOEN ? "enabled" : "disabled",
      reg & IXGBE_HLREG0_TXPADEN ? "enabled" : "disabled",
      reg & IXGBE_HLREG0_LPBK ? "enabled" : "disabled");

  /* General Registers */
  ebbrt::kprintf(
      "0x00000: CTRL        (Device Control)                 0x%08X\n",
      bar0_.Read32(0x0));

  ebbrt::kprintf(
      "0x00008: STATUS      (Device Status)                  0x%08X\n",
      bar0_.Read32(0x8));
}

void ebbrt::IxgbeDriver::WriteRxctrl(uint32_t m) {
  // Disable RXCTRL - 8.2.3.8.10
  bar0_.Write32(0x03000, m);
}

void ebbrt::IxgbeDriver::WriteDmatxctl(uint32_t m) {
  uint32_t reg;

  reg = bar0_.Read32(0x04A80);
  ebbrt::kprintf("0x04A80: DMATXCTL 0x%08X - reset to 0x%08X\n", reg, reg & m);

  // DMATXCTL - 8.2.3.9.2
  bar0_.Write32(0x04A80, reg & m);
}

// 8.2.3.5.1 Extended Interrupt Cause Register- EICR (0x00800; RW1C)
void ebbrt::IxgbeDriver::ReadEicr() {
  /* Note
   * The EICR is also cleared on read if GPIE.OCD bit is cleared. When the
   * GPIE.OCD bit is set, then only bits 16...29 are cleared on read.
   */
  // 8.2.3.5.18 General Purpose Interrupt Enable — GPIE (0x00898;RW)
  uint32_t reg;
  reg = bar0_.Read32(0x00898);
  // ebbrt::kprintf("0x00898: GPIE 0x%08X\n", reg);
  assert(!(reg & 0x20));  // make sure GPIE.OCD is cleared

  reg = bar0_.Read32(0x00800);
  ebbrt::kprintf("First Read - 0x00800: EICR 0x%08X, ", reg);

  reg = bar0_.Read32(0x00800);
  ebbrt::kprintf("Second Read - EICR 0x%08X\n", reg);
}

// 8.2.3.5.4 Extended Interrupt Mask Clear Register- EIMC (0x00888; WO)
void ebbrt::IxgbeDriver::WriteEimc(uint32_t m) { bar0_.Write32(0x00888, m); }

// 8.2.3.9.10 Transmit Descriptor Control — TXDCTL[n] (0x06028+0x40*n,
// n=0...127; RW)
void ebbrt::IxgbeDriver::WriteTxdctl(uint32_t n, uint32_t m) {
  // uint32_t reg;
  // reg = bar0_.Read32(0x06028+0x40*n);
  // ebbrt::kprintf("0x%05x: TXDCTL 0x%08X\n", 0x06028+0x40*n, reg);
  bar0_.Write32(0x06028 + (0x40 * n), m);
}

// 8.2.3.8.6 Receive Descriptor Control — RXDCTL[n] (0x01028 +
// 0x40*n, n=0...63 and 0x0D028 + 0x40*(n-64), n=64...127; RW)
void ebbrt::IxgbeDriver::WriteRxdctl_1(uint32_t n, uint32_t m) {
  bar0_.Write32(0x01028 + (0x40 * n), m);
}
void ebbrt::IxgbeDriver::WriteRxdctl_2(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0D028 + (0x40 * n), m);
}

// 8.2.3.1.1 Device Control Register — CTRL (0x00000 / 0x00004;RW)
void ebbrt::IxgbeDriver::WriteCtrl(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0, m);
}

// 8.2.3.1.2 Device Status Register — STATUS (0x00008; RO)
bool ebbrt::IxgbeDriver::ReadStatus() {
  auto reg = bar0_.Read32(0x8);
  ebbrt::kprintf("0x00008: Status 0x%08X\n", reg);
  return !(reg & 0x80000);
}

void ebbrt::IxgbeDriver::StopDevice() {
  ebbrt::kprintf("%s ", __PRETTY_FUNCTION__);

  // disable rx
  WriteRxctrl(0x0);

  // disable tx
  WriteDmatxctl(0xFFFFFFFE);

  // disable interrupts
  WriteEimc(0x7FFFFFFF);
  ReadEicr();

  // disable each rx and tx queue
  for (auto i = 0; i < 128; i++) {
    // Bit 26, transmit software flush
    WriteTxdctl(i, 0x04000000);

    if (i < 64) {
      WriteRxdctl_1(i, 0x0);
    } else {
      WriteRxdctl_2(i - 64, 0x0);
    }
  }

  // from arrakis
  ebbrt::clock::SleepMilli(2);

  // Master disable procedure
  WriteCtrl(0x4);
  while(ReadStatus() != 1);
  ebbrt::kprintf("Ixgbe 82599 stop done\n");
}
