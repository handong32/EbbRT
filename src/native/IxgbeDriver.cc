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
  //DeviceInfo();
  InitStruct();
  //StopDevice();
}

void ebbrt::IxgbeDriver::InitStruct() {
    struct IxgbeRegs *r = static_cast<struct IxgbeRegs*>(bar0_.GetVaddr());
    
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

void ebbrt::IxgbeDriver::StopDevice() {
  // uint32_t reg;
  ebbrt::kprintf("%s ", __PRETTY_FUNCTION__);

  // uint32_t *regs_buff = (uint32_t*)bar0_.GetVaddr()
  // ebbrt::kprintf("vaddr = %p\n", (void*)regs_buff);

  /*reg = bar0_.Read32(0x042A4);
  ebbrt::kprintf(
      "0x042A4: LINKS (Link Status register)                 0x%08X\n"
      "       Link Status:                                   %s\n"
      "       Link Speed:                                    %s\n",
      reg, reg & kIxgbeLinksUp ? "up" : "down",
      reg & kIxgbeLinksSpeed ? "10G" : "1G");
  */

  // Disable RX and TX
}
