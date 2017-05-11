//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#include "IxgbeDriver.h"

#include "../StaticIOBuf.h"
#include "../UniqueIOBuf.h"
#include "../Align.h"
#include "Fls.h"
#include "Pfn.h"
#include "Clock.h"
#include "Debug.h"
#include "EventManager.h"
#include "Ixgbe.h"
#include "Net.h"

#include <atomic>

void ebbrt::IxgbeDriver::Create(pci::Device& dev) {
  auto ixgbe_dev = new IxgbeDriver(dev);

  ixgbe_dev->Init();
  ixgbe_dev->ebb_ =
      IxgbeDriverRep::Create(ixgbe_dev, ebb_allocator->AllocateLocal());
  
  //auto rcv_vector =
  //event_manager->AllocateVector([this]() { ixgbe_dev->ebb_.ReceivePoll(); });
  
  for(size_t i = 0; i < Cpu::Count(); i++) {
    ixgbe_dev->SetupMultiQueue(i);
  }
  
  //ixgbe_dev->SetupQueue(0);
  ebbrt::clock::SleepMilli(200);
  ebbrt::kprintf("intel 82599 card initialzed\n");
}

const ebbrt::EthernetAddress& ebbrt::IxgbeDriver::GetMacAddress() {
  return mac_addr_;
}

void ebbrt::IxgbeDriver::Send(std::unique_ptr<IOBuf> buf, PacketInfo pinfo) {
  ebb_->Send(std::move(buf), std::move(pinfo));
}

void ebbrt::IxgbeDriver::Run() { ebb_->Run(); }

void ebbrt::IxgbeDriverRep::AddContext(uint8_t idx, uint8_t maclen, uint16_t iplen, uint8_t l4len, enum l4_type l4type) {

  tdesc_advance_ctxt_wb_t *actx;

  auto tail = ixgmq_.tx_tail_;
  actx = reinterpret_cast<tdesc_advance_ctxt_wb_t *>(&(ixgmq_.tx_ring_[tail]));

  actx->raw_1 = 0x0;
  actx->raw_2 = 0x0;
  
  memset(actx, 0, sizeof(tdesc_advance_ctxt_wb_t));
  ixgmq_.tx_isctx_[tail] = true;
  
  actx->dytp = 0b0010;
  actx->dext = 1;
  actx->idx = idx;
  actx->maclen = maclen;
  actx->iplen = iplen;
  
  actx->ipv4 = 1;
  actx->l4len = 0; //ignored when TSE not set
  actx->l4t = l4type;
  
  ixgmq_.tx_last_tail_ = ixgmq_.tx_tail_;
  ixgmq_.tx_tail_ = (tail + 1) % ixgmq_.tx_size_;
  
  //ebbrt::kprintf("%s raw_1->0x%llX raw_2->0x%llx head->%d tail->%d \n", __FUNCTION__, actx->raw_1, actx->raw_2, ixgq_.tx_head, ixgq_.tx_tail);
}

void ebbrt::IxgbeDriverRep::AddTx(const unsigned char *pa, uint64_t len, bool first, bool last, uint8_t ctx, bool ip_cksum, bool tcpudp_cksum) {

  tdesc_advance_tx_rf_t* actx;

  auto tail = ixgmq_.tx_tail_;
  actx = reinterpret_cast<tdesc_advance_tx_rf_t *>(&(ixgmq_.tx_ring_[tail]));

  ixgmq_.tx_isctx_[tail] = false;

  actx->raw[0] = 0x0;
  actx->raw[1] = 0x0;
  
  actx->address = reinterpret_cast<uint64_t>(pa);
  actx->dtalen = len;
  if(first) {
    actx->paylen = len;
  }

  actx->dtyp = 0b0011;
  actx->dext = 1;
  actx->rs = 1;
  actx->ifcs = 1;
  actx->eop = last;
  
  if(ctx != -1) {
    actx->idx = ctx;
    actx->cc = 1;
    actx->ixsm = ip_cksum; //no ip checksum
    actx->txsm = tcpudp_cksum; //udp or tcp checksum offload
  }

  ixgmq_.tx_last_tail_ = ixgmq_.tx_tail_;
  ixgmq_.tx_tail_ = (tail + 1) % ixgmq_.tx_size_;
  
  //ebbrt::kprintf("%s raw_1->0x%llX raw_2->0x%llx head->%d tail->%d \n", __FUNCTION__, actx->raw[0], actx->raw[1], ixgq_.tx_head, ixgq_.tx_tail);
}

void ebbrt::IxgbeDriverRep::Send(std::unique_ptr<IOBuf> buf, PacketInfo pinfo) {
  auto dp = buf->GetDataPointer();
  auto len = buf->ComputeChainDataLength();
  //kassert(len < 0xA0 * 1000);
  ebbrt::kbugon(len >= 0xA0 * 1000, "%s packet len bigger than max ether length\n", __FUNCTION__);
  // get a single buffer of data
  auto txbuf = dp.Get(len * sizeof(uint8_t));
  
  if(pinfo.flags & PacketInfo::kNeedsCsum) {
    if(pinfo.csum_offset == 6) {
      AddContext(0, ETHHDR_LEN, IPHDR_LEN, 0, l4_type_udp);
    }
    else if(pinfo.csum_offset == 16) {
      AddContext(0, ETHHDR_LEN, IPHDR_LEN, 0, l4_type_tcp);
    }
    else {
      ebbrt::kabort("%s unknown packet type checksum\n", __FUNCTION__);
    }
    
    AddTx(txbuf, len, true, true, 0, false, true);    
  } 
  else {
    AddTx(txbuf, len, true, true, -1, false, false);
  }
  
  ebbrt::kprintf("** %s **\n", __FUNCTION__);
  
  // bump tx_tail
  WriteTdt_1(Cpu::GetMine(), ixgmq_.tx_tail_); // indicates position beyond last descriptor hw

  // TODO: removing this causes general protection fault in Timer code??
  //ebbrt::clock::SleepMilli(1000);
  
  // TODO: when and where to update tx_head
  //ixgq_.tx_head = ixgq_.tx_hwb[0] % ixgq_.tx_size;
  ebbrt::kprintf("\t tx_head->%d tx_tail->%d \n", ixgmq_.tx_head_[0], ixgmq_.tx_tail_);
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
void ebbrt::IxgbeDriver::WriteDmatxctl_te(uint32_t m) {
  auto reg = bar0_.Read32(0x04A80);
  bar0_.Write32(0x04A80, reg | m);
}

// 8.2.3.5.18 - General Purpose Interrupt Enable — GPIE (0x00898; RW)
void ebbrt::IxgbeDriver::WriteGpie(uint32_t m) {
  uint32_t reg;
  reg = bar0_.Read32(0x00898);
  bar0_.Write32(0x00898, reg | m);
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
  ebbrt::kbugon((reg & 0x20), "GPIE.OCD not cleared\n");
  //kassert(!(reg & 0x20));  // make sure GPIE.OCD is cleared

  reg = bar0_.Read32(0x00800);
  ebbrt::kprintf("First Read - 0x00800: EICR 0x%08X, ", reg);

  reg = bar0_.Read32(0x00800);
  ebbrt::kprintf("Second Read - EICR 0x%08X\n", reg);
}
void ebbrt::IxgbeDriver::WriteEicr(uint32_t m) {
  auto reg = bar0_.Read32(0x00800);
  bar0_.Write32(0x00800, reg | m);
}

// 8.2.3.5.3 Extended Interrupt Mask Set/Read Register- EIMS (0x00880; RWS)
uint32_t ebbrt::IxgbeDriver::ReadEims() { return bar0_.Read32(0x00880); }
void ebbrt::IxgbeDriver::WriteEims(uint32_t m) { bar0_.Write32(0x00880, m); }

// 8.2.3.5.4 Extended Interrupt Mask Clear Register- EIMC (0x00888; WO)
void ebbrt::IxgbeDriver::WriteEimc(uint32_t m) { bar0_.Write32(0x00888, m); }

// 8.2.3.5.5 Extended Interrupt Auto Clear Register — EIAC (0x00810; RW)
void ebbrt::IxgbeDriver::WriteEiac(uint32_t m) {
  auto reg = bar0_.Read32(0x00810);
  bar0_.Write32(0x00810, reg | m);
}

// 8.2.3.5.8 Extended Interrupt Mask Set/Read Registers — EIMS[n] (0x00AA0 +
// 4*(n-1), n=1...2; RWS)
void ebbrt::IxgbeDriver::WriteEimsn(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x00AA0 + 4 * n);
  bar0_.Write32(0x00AA0 + 4 * n, reg | m);
}

// 8.2.3.5.12
// Extended Interrupt Throttle Registers — EITR[n]
// (0x00820 + 4*n, n=0...23 and 0x012300 + 4*(n-24),
// n=24...128; RW)
void ebbrt::IxgbeDriver::WriteEitr(uint32_t n, uint32_t m) {

  ebbrt::kbugon(n > 128, "%s error\n", __FUNCTION__);

  if (n < 24) {
    bar0_.Write32(0x00820 + 4*n, m);
  }
  else {
    bar0_.Write32(0x012300 + 4*(n-24), m);
  }
}

// 8.2.3.9.10 Transmit Descriptor Control — TXDCTL[n] (0x06028+0x40*n,
// n=0...127; RW)
void ebbrt::IxgbeDriver::WriteTxdctl(uint32_t n, uint32_t m) {
  // uint32_t reg;
  // reg = bar0_.Read32(0x06028+0x40*n);
  // ebbrt::kprintf("0x%05x: TXDCTL 0x%08X\n", 0x06028+0x40*n, reg);
  bar0_.Write32(0x06028 + (0x40 * n), m);
}
uint8_t ebbrt::IxgbeDriver::ReadTxdctl_enable(uint32_t n) {
  auto reg = bar0_.Read32(0x06028 + 0x40 * n);
  return (reg >> 25) & 0x1;
}

// 8.2.3.8.6 Receive Descriptor Control — RXDCTL[n] (0x01028 +
// 0x40*n, n=0...63 and 0x0D028 + 0x40*(n-64), n=64...127; RW)
void ebbrt::IxgbeDriver::WriteRxdctl_1(uint32_t n, uint32_t m) {
  bar0_.Write32(0x01028 + (0x40 * n), m);
}
void ebbrt::IxgbeDriver::WriteRxdctl_1_enable(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x01028 + (0x40 * n));
  bar0_.Write32(0x01028 + (0x40 * n), reg | m);
}

uint8_t ebbrt::IxgbeDriver::ReadRxdctl_1_enable(uint32_t n) {
  auto reg = bar0_.Read32(0x01028 + (0x40 * n));
  return (reg >> 25) & 0x1;
}

void ebbrt::IxgbeDriver::WriteRxdctl_2(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0D028 + (0x40 * n), m);
}

// 8.2.3.1.1 Device Control Register — CTRL (0x00000 / 0x00004;RW)
void ebbrt::IxgbeDriver::WriteCtrl(uint32_t m) { bar0_.Write32(0x0, m); }
void ebbrt::IxgbeDriver::ReadCtrl() {
  uint32_t reg;
  reg = bar0_.Read32(0x0);
  ebbrt::kprintf("0x00000: CTRL 0x%08X\n", reg);
}

// 8.2.3.1.3 Extended Device Control Register — CTRL_EXT (0x00018; RW)
void ebbrt::IxgbeDriver::WriteCtrlExt(uint32_t m) {
  auto reg = bar0_.Read32(0x00018);
  bar0_.Write32(0x00018, reg | m);
}

// 8.2.3.7.1 Filter Control Register — FCTRL (0x05080; RW)
void ebbrt::IxgbeDriver::WriteFctrl(uint32_t m) { bar0_.Write32(0x05080, m); }

// 8.2.3.24.9 Flexible Host Filter Table Registers — FHFT (0x09000 — 0x093FC and
// 0x09800 — 0x099FC; RW)
void ebbrt::IxgbeDriver::WriteFhft_1(uint32_t n, uint32_t m) {
  bar0_.Write32(0x09000, m);
}
void ebbrt::IxgbeDriver::WriteFhft_2(uint32_t n, uint32_t m) {
  bar0_.Write32(0x09800, m);
}

// 8.2.3.1.2 Device Status Register — STATUS (0x00008; RO)
bool ebbrt::IxgbeDriver::ReadStatusPcieMes() {
  auto reg = bar0_.Read32(0x8);
  ebbrt::kprintf("0x00008: Status PcieMes 0x%08X\n", reg);
  return !(reg & 0x80000);
}
uint8_t ebbrt::IxgbeDriver::ReadStatusLanId() {
  auto reg = bar0_.Read32(0x8);
  ebbrt::kprintf("0x00008: Status 0x%08X, Lan ID 0x%X\n", reg,
                 (reg >> 2) & 0x3);
  return (reg >> 2) & 0x3;
}

// 8.2.3.3.2 Flow Control Transmit Timer Value n — FCTTVn (0x03200 + 4*n,
// n=0...3; RW)
void ebbrt::IxgbeDriver::WriteFcttv(uint32_t n, uint32_t m) {
  bar0_.Write32(0x03200 + (4 * n), m);
}

// 8.2.3.3.3 Flow Control Receive Threshold Low — FCRTL[n] (0x03220 + 4*n,
// n=0...7; RW)
void ebbrt::IxgbeDriver::WriteFcrtl(uint32_t n, uint32_t m) {
  bar0_.Write32(0x03220 + (4 * n), m);
}

// 8.2.3.3.4 Flow Control Receive Threshold High — FCRTH[n] (0x03260 + 4*n,
// n=0...7; RW)
void ebbrt::IxgbeDriver::WriteFcrth(uint32_t n, uint32_t m) {
  bar0_.Write32(0x03260 + (4 * n), m);
}

// 8.2.3.3.5 Flow Control Refresh Threshold Value — FCRTV (0x032A0; RW)
void ebbrt::IxgbeDriver::WriteFcrtv(uint32_t m) { bar0_.Write32(0x032A0, m); }

// 8.2.3.3.7 Flow Control Configuration — FCCFG (0x03D00; RW)
void ebbrt::IxgbeDriver::WriteFccfg(uint32_t m) { bar0_.Write32(0x03D00, m); }

// 8.2.3.2.2 EEPROM Read Register — EERD (0x10014; RW)
void ebbrt::IxgbeDriver::WriteEerd(uint32_t m) { bar0_.Write32(0x10014, m); }
bool ebbrt::IxgbeDriver::ReadEerdDone() {
  auto reg = bar0_.Read32(0x10014);
  ebbrt::kprintf("0x10014: EERD 0x%08X\n", reg);
  return !!(reg & 0x2);  // return true when Read Done = 1
}

uint16_t ebbrt::IxgbeDriver::ReadEerdData() {
  auto reg = bar0_.Read32(0x10014);
  return (reg >> 16) & 0xFFFF;
}

uint16_t ebbrt::IxgbeDriver::ReadEeprom(uint16_t offset) {
  // ebbrt::kprintf("%s - writing 0x%08X\n", __PRETTY_FUNCTION__, offset << 2 |
  // 1);
  WriteEerd(offset << 2 | 1);
  while (ReadEerdDone() == 0)
    ;  // TODO: Timeout
  return ReadEerdData();
}

// 8.2.3.22.32 - Core Analog Configuration Register — CoreCTL (0x014F00; RW)
void ebbrt::IxgbeDriver::WriteCorectl(uint16_t m) {
  bar0_.Write32(0x014F00, 0x0 | m);
}

// 8.2.3.22.19 Auto Negotiation Control Register — AUTOC (0x042A0; RW)
void ebbrt::IxgbeDriver::WriteAutoc(uint32_t m) {
  auto reg = bar0_.Read32(0x042A0);
  bar0_.Write32(0x042A0, reg | m);
}
uint8_t ebbrt::IxgbeDriver::ReadAutocRestartAn() {
  auto reg = bar0_.Read32(0x042A0);
  return (reg >> 12) & 0x1;
}

// 8.2.3.22.23 Auto Negotiation Link Partner Link Control Word 1 Register —
// ANLP1 (0x042B0; RO)
uint8_t ebbrt::IxgbeDriver::ReadAnlp1() {
  auto reg = bar0_.Read32(0x042B0);
  return (reg >> 16) & 0xFF;
}

// 8.2.3.2.1 EEPROM/Flash Control Register — EEC (0x10010; RW)
uint8_t ebbrt::IxgbeDriver::ReadEecAutoRd() {
  auto reg = bar0_.Read32(0x10010);
  return (reg >> 9) & 0xFF;
}

// 8.2.3.7.7 Multicast Table Array — MTA[n] (0x05200 + 4*n, n=0...127; RW)
void ebbrt::IxgbeDriver::WriteMta(uint32_t n, uint32_t m) {
  bar0_.Write32(0x05200 + (4 * n), m);
}

// 8.2.3.7.11 VLAN Filter Table Array — VFTA[n] (0x0A000 + 4*n,n=0...127; RW)
void ebbrt::IxgbeDriver::WriteVfta(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0A000 + (4 * n), m);
}

// 8.2.3.27.15 PF VM VLAN Pool Filter — PFVLVF[n] (0x0F100 + 4*n, n=0...63; RW)
void ebbrt::IxgbeDriver::WritePfvlvf(uint32_t n, uint32_t m) {
  // auto reg = bar0_.Read32(0x0F100 + 4*n);
  // bar0_.Write32(0x0F100 + 4*n, reg | m);
  bar0_.Write32(0x0F100 + 4 * n, m);
}

// Checks the MAC's EEPROM to see if it supports a given SFP+ module type, if
// 1360
// so it returns the offsets to the phy init sequence block.
// also based on
// http://lxr.free-electrons.com/source/drivers/net/ethernet/intel/ixgbe/ixgbe_phy.c?v=3.14#L1395
// https://github.com/freebsd/freebsd/blob/386ddae58459341ec567604707805814a2128a57/sys/dev/ixgbe/ixgbe_82599.c#L173
void ebbrt::IxgbeDriver::PhyInit() {

  uint16_t list_offset;
  uint16_t data_offset = 0x0;
  uint16_t data_value;
  uint16_t sfp_id;
  uint16_t sfp_type = 0x4; /* SPF_DA_CORE1 */

  /* IXGBE_PHY_INIT_OFFSET_NL */
  list_offset = ReadEeprom(0x002B);
  ebbrt::kprintf("list_offset -> 0x%x\n", list_offset);

  if ((list_offset == 0x0) || (list_offset == 0xFFFF)) {
    return;
  }

  /* Shift offset to first ID word */
  list_offset++;

  sfp_id = ReadEeprom(list_offset);
  ebbrt::kprintf("sfp_id -> %x\n", sfp_id);

  while (sfp_id != 0xFFFF) {
    if (sfp_id == sfp_type) {
      list_offset++;
      data_offset = ReadEeprom(list_offset);
      if ((data_offset == 0x0) || (data_offset == 0xFFFF)) {
        ebbrt::kprintf("sfp init failed\n");
        return;
      } else {
        break;
      }
    } else {
      list_offset += 2;
      sfp_id = ReadEeprom(list_offset);
    }
    list_offset++;
  }

  if (sfp_id == 0xFFFF) {
    ebbrt::kprintf("sfp init failed\n");
    return;
  }

  ebbrt::kprintf("data offset -> 0x%x\n", data_offset);

  SwfwLockPhy();

  data_value = ReadEeprom(++data_offset);
  while (data_value != 0xFFFF) {
    ebbrt::kprintf("data_value -> 0x%x\n", data_value);
    WriteCorectl(data_value);  //??
    data_value = ReadEeprom(++data_offset);
  }
  SwfwUnlockPhy();

  ebbrt::clock::SleepMilli(20);

  WriteAutoc(0x0 << 13 | 0x1 << 12);
  while (ReadAnlp1() != 0)
    ;  // TODO: timeout

  WriteAutoc(0x3 << 13 | 0x1 << 12);
  while (ReadAutocRestartAn() != 0)
    ;  // TODO: timeout

  ebbrt::kprintf("PHY init done\n");
}

// 8.2.3.7.8 Receive Address Low — RAL[n] (0x0A200 + 8*n, n=0...127; RW)
uint32_t ebbrt::IxgbeDriver::ReadRal(uint32_t n) {
  auto reg = bar0_.Read32(0x0A200 + 8 * n);
  // ebbrt::kprintf("%s %x\n", __FUNCTION__, reg);
  return reg;
}
void ebbrt::IxgbeDriver::WriteRal(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0A200 + (8 * n), m);
}

// 8.2.3.7.9 Receive Address High — RAH[n] (0x0A204 + 8*n, n=0...127; RW)
uint16_t ebbrt::IxgbeDriver::ReadRah(uint32_t n) {
  auto reg = bar0_.Read32(0x0A204 + 8 * n);
  return (reg)&0xFFFF;
}
uint8_t ebbrt::IxgbeDriver::ReadRahAv(uint32_t n) {
  return (bar0_.Read32(0x0A204 + 8 * n) >> 31) & 0xFF;
}
void ebbrt::IxgbeDriver::WriteRah(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0A204 + (8 * n), m);
}

// 8.2.3.7.10 MAC Pool Select Array — MPSAR[n] (0x0A600 + 4*n, n=0...255; RW)
void ebbrt::IxgbeDriver::WriteMpsar(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0A600 + 4 * n, m);
}

// 8.2.3.7.19 Five tuple Queue Filter — FTQF[n] (0x0E600 + 4*n,n=0...127; RW)
void ebbrt::IxgbeDriver::WriteFtqf(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0E600 + 4 * n, m);
}

// 8.2.3.7.16 Source Address Queue Filter — SAQF[n] (0x0E000 + 4*n, n=0...127;
// RW)
void ebbrt::IxgbeDriver::WriteSaqf(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0E000 + 4 * n, m);
}

// 8.2.3.7.17 Destination Address Queue Filter — DAQF[n] (0x0E200 + 4*n,
// n=0...127; RW)
void ebbrt::IxgbeDriver::WriteDaqf(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0E200 + 4 * n, m);
}

// 8.2.3.7.18 Source Destination Port Queue Filter — SDPQF[n] (0x0E400 + 4*n,
// n=0...127; RW)
void ebbrt::IxgbeDriver::WriteSdpqf(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0E400 + 4 * n, m);
}

// 8.2.3.27.17 PF Unicast Table Array — PFUTA[n] (0x0F400 + 4*n, n=0...127; RW)
void ebbrt::IxgbeDriver::WritePfuta(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0F400 + 4 * n, m);
}

// 8.2.3.7.3 Multicast Control Register — MCSTCTRL (0x05090; RW)
void ebbrt::IxgbeDriver::WriteMcstctrl(uint32_t m) {
  auto reg = bar0_.Read32(0x05090);
  bar0_.Write32(0x05090, reg & m);
}

// 8.2.3.10.13 DCB Transmit Descriptor Plane Queue Select — RTTDQSEL (0x04904;
// RW)
void ebbrt::IxgbeDriver::WriteRttdqsel(uint32_t m) {
  auto reg = bar0_.Read32(0x04904);
  bar0_.Write32(0x04904, reg | m);
}

// 8.2.3.10.16 DCB Transmit Rate-Scheduler Config — RTTBCNRC (0x04984; RW)
void ebbrt::IxgbeDriver::WriteRttbcnrc(uint32_t m) {
  bar0_.Write32(0x04984, m);
}

// 8.2.3.11.2 Tx DCA Control Registers — DCA_TXCTRL[n] (0x0600C + 0x40*n,
// n=0...127; RW)
void ebbrt::IxgbeDriver::WriteDcaTxctrlTxdescWbro(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x0600C + 0x40 * n);
  // ebbrt::kprintf("%s: 0x%08X -> 0x%08X\n", __FUNCTION__, reg, reg & m);
  bar0_.Write32(0x0600C + 0x40 * n, reg & m);
}

// 8.2.3.11.1 Rx DCA Control Register — DCA_RXCTRL[n] (0x0100C + 0x40*n,
// n=0...63 and 0x0D00C + 0x40*(n-64),
// n=64...127 / 0x02200 + 4*n, [n=0...15]; RW)
void ebbrt::IxgbeDriver::WriteDcaRxctrl_1(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x0100C + 0x40 * n);
  bar0_.Write32(0x0100C + 0x40 * n, reg & m);
}

// void ebbrt::IxgbeDriver::WriteDcaRxctrl_1_RxdataWrro(uint32_t n, uint32_t m);
void ebbrt::IxgbeDriver::WriteDcaRxctrl_2(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x0D00C + 0x40 * n);
  bar0_.Write32(0x0D00C + 0x40 * n, reg & m);
}
// void ebbrt::IxgbeDriver::WriteDcaRxctrl_2_RxdataWrro(uint32_t n, uint32_t m);

// 8.2.3.4.9 - Software Semaphore Register — SWSM (0x10140; RW)
bool ebbrt::IxgbeDriver::SwsmSmbiRead() {
  return !!(bar0_.Read32(0x10140) & 0x1);
}
bool ebbrt::IxgbeDriver::SwsmSwesmbiRead() {
  return !(bar0_.Read32(0x10140) & 0x2);
}
void ebbrt::IxgbeDriver::SwsmSwesmbiSet() {
  auto reg = bar0_.Read32(0x10140);
  ebbrt::kprintf("%s: reg before: 0x%08X, reg after: 0x%08X\n", __FUNCTION__,
                 reg, reg | 0x2);
  bar0_.Write32(0x10140, reg | 0x2);
}
void ebbrt::IxgbeDriver::SwsmSmbiClear() {
  auto reg = bar0_.Read32(0x10140);
  ebbrt::kprintf("%s: reg before: 0x%08X, reg after: 0x%08X\n", __FUNCTION__,
                 reg, reg & 0xFFFFFFFE);
  bar0_.Write32(0x10140, reg & 0xFFFFFFFE);
}
void ebbrt::IxgbeDriver::SwsmSwesmbiClear() {
  auto reg = bar0_.Read32(0x10140);
  ebbrt::kprintf("%s: reg before: 0x%08X, reg after: 0x%08X\n", __FUNCTION__,
                 reg, reg & 0xFFFFFFFD);
  bar0_.Write32(0x10140, reg & 0xFFFFFFFD);
}

// 8.2.3.22.20 Link Status Register — LINKS (0x042A4; RO)
bool ebbrt::IxgbeDriver::ReadLinksLinkUp() {
  auto reg = bar0_.Read32(0x042A4);
  return ((reg >> 30) & 0x1) == 1;
}

// 8.2.3.4.11 Software-Firmware Synchronization - SW_FW_SYNC (0x10160; RW)
uint32_t ebbrt::IxgbeDriver::ReadSwfwSyncSmBits(uint32_t m) {
  auto reg = bar0_.Read32(0x10160);
  return (reg & m) & 0x3FF;  // masking bits 9:0
}
void ebbrt::IxgbeDriver::WriteSwfwSyncSmBits(uint32_t m) {
  auto reg = bar0_.Read32(0x10160);
  bar0_.Write32(0x10160, reg | m);
}
void ebbrt::IxgbeDriver::WriteSwfwSyncSmBits2(uint32_t m) {
  auto reg = bar0_.Read32(0x10160);
  bar0_.Write32(0x10160, reg & m);
}

// 8.2.3.8.1 Receive Descriptor Base Address Low — RDBAL[n] (0x01000 + 0x40*n,
// n=0...63 and 0x0D000 + 0x40*(n-64), n=64...127; RW)
void ebbrt::IxgbeDriver::WriteRdbal_1(uint32_t n, uint32_t m) {
  //ebbrt::kprintf("%s 0x%X\n", __FUNCTION__, m);
  bar0_.Write32(0x01000 + 0x40 * n, m);
}
void ebbrt::IxgbeDriver::WriteRdbal_2(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0D000 + 0x40 * n, m);
}

// 8.2.3.8.2 Receive Descriptor Base Address High — RDBAH[n] (0x01004 + 0x40*n,
// n=0...63 and 0x0D004 + 0x40*(n-64), n=64...127; RW)
void ebbrt::IxgbeDriver::WriteRdbah_1(uint32_t n, uint32_t m) {
  //ebbrt::kprintf("%s 0x%X\n", __FUNCTION__, m);
  bar0_.Write32(0x01004 + 0x40 * n, m);
}
void ebbrt::IxgbeDriver::WriteRdbah_2(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0D004 + 0x40 * n, m);
}

// 8.2.3.9.5 Transmit Descriptor Base Address Low — TDBAL[n] (0x06000+0x40*n,
// n=0...127; RW)
void ebbrt::IxgbeDriver::WriteTdbal(uint32_t n, uint32_t m) {
  bar0_.Write32(0x06000 + 0x40 * n, m);
}

// 8.2.3.9.6 Transmit Descriptor Base Address High — TDBAH[n] (0x06004+0x40*n,
// n=0...127; RW)
void ebbrt::IxgbeDriver::WriteTdbah(uint32_t n, uint32_t m) {
  bar0_.Write32(0x06004 + 0x40 * n, m);
}

// 8.2.3.9.7 Transmit Descriptor Length — TDLEN[n] (0x06008+0x40*n, n=0...127;
// RW)
void ebbrt::IxgbeDriver::WriteTdlen(uint32_t n, uint32_t m) {
  bar0_.Write32(0x06008 + 0x40 * n, m);
}

// 8.2.3.9.8 Transmit Descriptor Head — TDH[n] (0x06010+0x40*n, n=0...127; RO)
void ebbrt::IxgbeDriver::WriteTdh(uint32_t n, uint32_t m) {
  bar0_.Write32(0x06010 + 0x40 * n, m);
}
uint16_t ebbrt::IxgbeDriver::ReadTdh(uint32_t n) {
  auto reg = bar0_.Read32(0x06010 + 0x40 * n);
  return reg & 0xFFFF;
}

// 8.2.3.9.11 Tx Descriptor Completion Write Back Address Low —
// TDWBAL[n] (0x06038+0x40*n, n=0...127; RW)
void ebbrt::IxgbeDriver::WriteTdwbal(uint32_t n, uint32_t m) {
  bar0_.Write32(0x06038 + 0x40 * n, m);
}
// 8.2.3.9.12 Tx Descriptor Completion Write Back Address High —
// TDWBAH[n] (0x0603C+0x40*n, n=0...127; RW)
void ebbrt::IxgbeDriver::WriteTdwbah(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0603C + 0x40 * n, m);
}

// 8.2.3.9.9 Transmit Descriptor Tail — TDT[n] (0x06018+0x40*n, n=0...127; RW)
void ebbrt::IxgbeDriver::WriteTdt(uint32_t n, uint32_t m) {
  bar0_.Write32(0x06018 + 0x40 * n, m);
}

// 8.2.3.8.3 Receive Descriptor Length — RDLEN[n] (0x01008 + 0x40*n, n=0...63
// and 0x0D008 + 0x40*(n-64), n=64...127; RW)
void ebbrt::IxgbeDriver::WriteRdlen_1(uint32_t n, uint32_t m) {
  //ebbrt::kprintf("%s %d\n", __FUNCTION__, m);
  bar0_.Write32(0x01008 + 0x40 * n, m);
}
void ebbrt::IxgbeDriver::WriteRdlen_2(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0D008 + 0x40 * n, m);
}

// 8.2.3.8.7 Split Receive Control Registers — SRRCTL[n] (0x01014 + 0x40*n,
// n=0...63 and 0x0D014 + 0x40*(n-64), n=64...127 / 0x02100 + 4*n, [n=0...15];
// RW)
void ebbrt::IxgbeDriver::WriteSrrctl_1(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x01014 + 0x40 * n);
  bar0_.Write32(0x01014 + 0x40 * n, reg | m);
}
/*void ebbrt::IxgbeDriver::WriteSrrctl_1_bsizepacket(uint32_t n, uint32_t m) {
    auto reg = bar0_.Read32(0x01014 + 0x40*n);
    bar0_.Write32(0x01014 + 0x40*n, reg | m);
    }*/

void ebbrt::IxgbeDriver::WriteSrrctl_1_desctype(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x01014 + 0x40 * n);
  bar0_.Write32(0x01014 + 0x40 * n, reg & m);
}

// 8.2.3.8.8 Receive DMA Control Register — RDRXCTL (0x02F00; RW)
void ebbrt::IxgbeDriver::WriteRdrxctl(uint32_t m) {
  auto reg = bar0_.Read32(0x02F00);
  bar0_.Write32(0x02F00, reg | m);
}
uint8_t ebbrt::IxgbeDriver::ReadRdrxctlDmaidone() {
  auto reg = bar0_.Read32(0x02F00);
  return (reg >> 3) & 0x1;
}

// 8.2.3.22.8 MAC Core Control 0 Register — HLREG0 (0x04240; RW)
void ebbrt::IxgbeDriver::WriteHlreg0(uint32_t m) {
  auto reg = bar0_.Read32(0x04240);
  bar0_.Write32(0x04240, reg | m);
}

// 8.2.3.8.5 Receive Descriptor Tail — RDT[n] (0x01018 + 0x40*n, n=0...63 and
// 0x0D018 + 0x40*(n-64), n=64...127; RW)
void ebbrt::IxgbeDriver::WriteRdt_1(uint32_t n, uint32_t m) {
  bar0_.Write32(0x01018 + 0x40 * n, m);
}
void ebbrt::IxgbeDriver::WriteRdt_2(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0D018 + 0x40 * n, m);
}

// 8.2.3.8.4 Receive Descriptor Head — RDH[n] (0x01010 + 0x40*n, n=0...63 and
// 0x0D010 + 0x40*(n-64), n=64...127; RO)
void ebbrt::IxgbeDriver::WriteRdh_1(uint32_t n, uint32_t m) {
  bar0_.Write32(0x01010 + 0x40 * n, m);
}
uint16_t ebbrt::IxgbeDriver::ReadRdh_1(uint32_t n) {
  auto reg = bar0_.Read32(0x01010 + 0x40 * n);
  return reg & 0xFFFF;
}

void ebbrt::IxgbeDriver::SwfwSemRelease() {
  SwsmSwesmbiClear();
  SwsmSmbiClear();
  ebbrt::kprintf("%s\n", __FUNCTION__);
}

// 8.2.3.5.16 Interrupt Vector Allocation Registers — IVAR[n] (0x00900 + 4*n,
// n=0...63; RW)
void ebbrt::IxgbeDriver::WriteIvarAlloc0(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  //reg = reg & ~(0x3F);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
}
void ebbrt::IxgbeDriver::WriteIvarAllocval0(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
}

void ebbrt::IxgbeDriver::WriteIvarAlloc1(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  //reg = reg & ~(0x3F << 8);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
}
void ebbrt::IxgbeDriver::WriteIvarAllocval1(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
}

void ebbrt::IxgbeDriver::WriteIvarAlloc2(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  //reg = reg & ~(0x3F << 8);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
}
void ebbrt::IxgbeDriver::WriteIvarAllocval2(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
}

void ebbrt::IxgbeDriver::WriteIvarAlloc3(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  //reg = reg & ~(0x3F << 8);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
}
void ebbrt::IxgbeDriver::WriteIvarAllocval3(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
}

// 8.2.3.12.5 Security Rx Control — SECRXCTRL (0x08D00; RW)
void ebbrt::IxgbeDriver::WriteSecrxctrl_Rx_Dis(uint32_t m) {
  auto reg = bar0_.Read32(0x08D00);
  if (m) {
    bar0_.Write32(0x08D00, reg | m);
  } else {
    bar0_.Write32(0x08D00, reg & ~(0x1 << 1));
  }
}

// 8.2.3.12.6 Security Rx Status — SECRXSTAT (0x08D04; RO)
uint8_t ebbrt::IxgbeDriver::ReadSecrxstat_Sr_Rdy() {
  auto reg = bar0_.Read32(0x08D04);
  return reg & 0x1;
}

// 8.2.3.23.59 Total Packets Received — TPR (0x040D0; RC)
uint32_t ebbrt::IxgbeDriver::ReadTpr() {
  auto reg = bar0_.Read32(0x040D0);
  ebbrt::kprintf("%s %d\n", __FUNCTION__, reg);
  return reg;
}

// 8.2.3.23.26 Good Packets Received Count — GPRC (0x04074; RO)
uint32_t ebbrt::IxgbeDriver::ReadGprc() {
  auto reg = bar0_.Read32(0x04074);
  ebbrt::kprintf("%s %d\n", __FUNCTION__, reg);
  return reg;
}

bool ebbrt::IxgbeDriver::SwfwSemAcquire() {
  // polls SWSM.SMBI until 0b is read or timeout
  while (SwsmSmbiRead())
    ;  // TODO: timeout after 10 ms

  // writes 1b to SWSM.SWESMBI bit
  SwsmSwesmbiSet();

  // polls SWSM.SWESMBI bit until read as 1b
  while (SwsmSwesmbiRead())
    ;  // TODO: timeout of 3 secs

  return true;
}

// 10.5.4 Software and Firmware Synchronization
bool ebbrt::IxgbeDriver::SwfwLockPhy() {
  bool good = false;

again:
  if (!SwfwSemAcquire()) {
    ebbrt::kabort("SwfwSemAcquire failed\n");
  } else {
    ebbrt::kprintf("SWSM Sem acquired\n");
  }

  if ((ReadStatusLanId() == 0) && (ReadSwfwSyncSmBits(0x2) == 0)  // SW_PHY_SM0
      && (ReadSwfwSyncSmBits(0x40) == 0))  // FW_PHY_SM0
  {
    WriteSwfwSyncSmBits(0x2);  // SW_PHY_SM0
    ebbrt::kprintf("SW_PHY_SMO written\n");
    good = true;
  } else if ((ReadSwfwSyncSmBits(0x4) == 0)  // SW_PHY_SM1
             && (ReadSwfwSyncSmBits(0x80) == 0))  // FW_PHY_SM1
  {
    WriteSwfwSyncSmBits(0x4);  // SW_PHY_SM1
    ebbrt::kprintf("SW_PHY_SM1 written\n");
    good = true;
  }

  SwfwSemRelease();

  if (!good) {
    ebbrt::kprintf("%s: failed, trying again\n", __FUNCTION__);
    ebbrt::clock::SleepMilli(20);
    goto again;
  }

  return true;
}
void ebbrt::IxgbeDriver::SwfwUnlockPhy() {
  if (!SwfwSemAcquire()) {
    ebbrt::kabort("SwfwSemAcquire failed\n");
  } else {
    ebbrt::kprintf("SWSM Sem acquired\n");
  }

  if (ReadStatusLanId() == 0) {
    WriteSwfwSyncSmBits2(~0x2);  // SW_PHY_SM0
  } else {
    WriteSwfwSyncSmBits2(~0x4);  // SW_PHY_SM1
  }

  SwfwSemRelease();

  ebbrt::clock::SleepMilli(10);
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
  WriteCtrl(0x4);  // PCIe Master Disable
  while (ReadStatusPcieMes() != 1)
    ;
  ebbrt::kprintf("Ixgbe 82599 stop done\n");
}

void ebbrt::IxgbeDriver::GlobalReset() {
  ebbrt::kprintf("%s ", __PRETTY_FUNCTION__);

  WriteCtrl(0x8);  // Link Reset
  WriteCtrl(0x4000000);  // Device Reset

  // Note: To ensure that a global device reset has fully completed and that the
  // 82599 responds to  subsequent accesses, programmers must wait
  // before  approximately 1 ms after setting attempting to check
  // if the bit has cleared or to access (read or write) any other device
  // register.
  ebbrt::clock::SleepMilli(2);
  ReadCtrl();
}

/**
 *  ixgbe_init_hw_generic - Generic hardware initialization
 *  @hw: pointer to hardware structure
 *
 *  Initialize the hardware by resetting the hardware, filling the bus info
 *  structure and media type, clears all on chip counters, initializes receive
 *  address registers, multicast table, VLAN filter table, calls routine to set
 *  up link and flow control settings, and leaves transmit and receive units
 *  disabled and uninitialized
 **/
void ebbrt::IxgbeDriver::Init() {
  uint64_t d_mac;

  ebbrt::kprintf("%s ", __PRETTY_FUNCTION__);
  bar0_.Map();  // allocate virtual memory
  ebbrt::clock::SleepMilli(200);
  ebbrt::kprintf("Sleep 200 ms\n");

  // DeviceInfo();
  // InitStruct();

  StopDevice();
  GlobalReset();
  ebbrt::clock::SleepMilli(50);
  GlobalReset();
  ebbrt::clock::SleepMilli(250);

  // disable interrupts
  WriteEimc(0x7FFFFFFF);
  ReadEicr();

  // Let firmware know we have taken over
  WriteCtrlExt(0x1 << 28);  // DRV_LOAD

  // No snoop disable from FreeBSD ??
  WriteCtrlExt(0x1 << 16);  // NS_DIS

  // Initialize flow-control registers
  for (auto i = 0; i < 8; i++) {
    if (i < 4) {
      WriteFcttv(i, 0x0);
    }
    WriteFcrtl(i, 0x0);
    WriteFcrth(i, 0x0);
  }

  WriteFcrtv(0x0);
  WriteFccfg(0x0);

  // Initialize Phy
  PhyInit();

  // Wait for EEPROM auto read
  while (ReadEecAutoRd() == 0)
    ;  // TODO: Timeout
  ebbrt::kprintf("EEPROM auto read done\n");

  ebbrt::clock::SleepMilli(200);
  d_mac = ReadRal(0) | ((uint64_t)ReadRah(0) << 32);
  // ebbrt::kprintf("mac %p valid = %x\n", d_mac, ReadRahAv(0));
  for (auto i = 0; i < 6; i++) {
    mac_addr_[i] = (d_mac >> (i * 8)) & 0xFF;
  }
  ebbrt::kprintf(
      "Mac Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
      static_cast<uint8_t>(mac_addr_[0]), static_cast<uint8_t>(mac_addr_[1]),
      static_cast<uint8_t>(mac_addr_[2]), static_cast<uint8_t>(mac_addr_[3]),
      static_cast<uint8_t>(mac_addr_[4]), static_cast<uint8_t>(mac_addr_[5]));

  // Wait for DMA initialization
  while (ReadRdrxctlDmaidone() == 0)
  ;  // TODO: Timeout

  // Wait for link to come up
  while (!ReadLinksLinkUp())
    ;  // TODO: timeout
  ebbrt::kprintf("Link is up\n");
  ebbrt::clock::SleepMilli(50);

  // Initialize interrupts
  WriteEicr(0xFFFFFFFF);

  /* setup msix */
  // switch to msix mode
  WriteGpie(0x1 << 4);  // Multiple_MSIX
  WriteGpie(0x1 << 31);  // PBA_support
  WriteGpie(0x1 << 5);  // OCD

  // TODO: Set up management interrupt handler

  // Enable auto masking of interrupt
  WriteGpie(0x1 << 30);  // EIAME

  /* FreeBSD:
   * ixgbe_common.c - s32 ixgbe_init_rx_addrs_generic(struct ixgbe_hw *hw)
   * Places the MAC address in receive address register 0 and clears the rest
   *  of the receive address registers. Clears the multicast table. Assumes
   *  the receiver is in reset when the routine is called.
   */
  // Initialize RX filters

  /* Zero out the other receive addresses. */
  for (auto i = 1; i < 128; i++) {
    WriteRal(i, 0x0);
    WriteRah(i, 0x0);
  }

  // clear mta
  for (auto i = 0; i < 128; i++) {
    WriteMta(i, 0x0);
  }

  // No init uta tables?

  // set vlan filter table
  for (auto i = 0; i < 128; i++) {
    WriteVfta(i, 0x0);
  }

  for (auto i = 0; i < 64; i++) {
    WritePfvlvf(i, 0x1 << 31);  // VI_En bit 31
  }

  for (auto i = 0; i < 256; i++) {
    WriteMpsar(i, 0x0);
  }

  for (auto i = 0; i < 128; i++) {
    WriteFtqf(i, 0x0);
    WriteSaqf(i, 0x0);
    WriteDaqf(i, 0x0);
    WriteSdpqf(i, 0x0);
  }

  // FreeBSD if_ix.c - ixgbe_initialize_receive_units - Enable broadcast accept
  WriteFctrl(0x1 << 10);  // Set BAM = 1

  // not sure why initing these tables?
  for (auto i = 0; i < 128; i++) {
    WriteFhft_1(i, 0x0);
    if (i < 64) {
      WriteFhft_2(i, 0x0);
    }
  }

  // PF Unicast Table Array
  for (auto i = 0; i < 128; i++) {
    WritePfuta(i, 0x0);
  }

  // Multicast Control Register
  // WriteMcstctrl(0x1 << 2);
  WriteMcstctrl(~(0x1 << 2));  // disable MFE

  // Make sure RX CRC strip enabled in HLREG0 and RDRXCTL
  WriteHlreg0(0x1 << 1);  // CRCStrip
  WriteRdrxctl(0x1);  // CRCStrip

  // from freeBSD/arrakis - ixgbe_common.c - ixgbe_start_hw_gen2
  // clear the rate limiters
  for (auto i = 0; i < 128; i++) {
    WriteRttdqsel(i);
    WriteRttbcnrc(0x0);
  }

  // disable relaxed ordering
  for (auto i = 0; i < 128; i++) {
    WriteDcaTxctrlTxdescWbro(i, ~(0x1 << 11));  // Txdesc_Wbro

    if (i < 64) {
      WriteDcaRxctrl_1(
          i, ~(0x1 << 15));  // Rx split header relax order enable, bit 15
      WriteDcaRxctrl_1(
          i, ~(0x1 << 13));  // Rx data Write Relax Order Enable, bit 13
    } else {
      WriteDcaRxctrl_1(
          i - 64, ~(0x1 << 15));  // Rx split header relax order enable, bit 15
      WriteDcaRxctrl_1(
          i - 64, ~(0x1 << 13));  // Rx data Write Relax Order Enable, bit 13
    }
  }
}

void ebbrt::IxgbeDriver::SetupMultiQueue(uint32_t i) {
  if(!rcv_vector) {
    rcv_vector =
      event_manager->AllocateVector([this]() { ebb_->ReceivePoll(); });
  }
  
  ebbrt::kprintf("%s for Core %d, rcv_vector = 0x%X\n", __FUNCTION__, i, rcv_vector);

  // allocate memory for descriptor rings
  ixgmq[i].reset(new e10Kq(i, Cpu::GetMyNode()));
  
  // not going to set up receive descripts greater than 63
  ebbrt::kbugon(i >= 64, "can't set up descriptors greater than 63\n");
  
  // update register RDBAL, RDBAH with receive descriptor base address
  WriteRdbal_1(i, ixgmq[i]->rxaddr_ & 0xFFFFFFFF);
  WriteRdbah_1(i, (ixgmq[i]->rxaddr_ >> 32) & 0xFFFFFFFF);

  // set to number of bytes allocated for receive descriptor ring
  WriteRdlen_1(i, ixgmq[i]->rx_size_bytes_);

  // program srrctl register
  WriteSrrctl_1(i, RXBUFSZ / 1024);  // bsizepacket = 2 KB
  // TODO use adv?
  WriteSrrctl_1_desctype(i, ~(0x7 << 25));  // desctype legacy
  WriteSrrctl_1(i, 0x1 << 28);  // Drop_En

  // Set Enable bit in receive queue
  WriteRxdctl_1_enable(i, 0x1 << 25);
  // till set
  // TODO: Timeout
  while (ReadRxdctl_1_enable(i) == 0);

  // Set head and tail pointers
  WriteRdt_1(i, 0x0);
  WriteRdh_1(i, 0x0);
  //ebbrt::kprintf("RX queue enabled\n");
  
  // setup RX interrupts for queue i
  
  dev_.SetMsixEntry(i, rcv_vector, ebbrt::Cpu::GetByIndex(i)->apic_id());
  ebbrt::kprintf("apic_id = %d\n", ebbrt::Cpu::GetByIndex(i)->apic_id());
  
  // don't set up interrupts for tx since we have head writeback??
  auto qn = i / 2; // put into correct IVAR
  
  if((i % 2) == 0) { // check if 2xN or 2xN + 1
    WriteIvarAlloc0(qn, i); // rx interrupt allocation corresponds to index i * 2 in MSI-X table
    
    WriteIvarAllocval0(qn, 0x1 << 7);
    
    ebbrt::kprintf("IVAR %d, INT_Alloc 0x%X, INT_Alloc_val 0x%X\n", qn, i, 0x1 << 7);
  }
  else {
    WriteIvarAlloc2(qn, i << 16);
    
    WriteIvarAllocval2(qn, 0x1 << 23);
    
    ebbrt::kprintf("IVAR %d, INT_Alloc 0x%X, INT_Alloc_val 0x%X\n", qn, i << 16, 0x1 << 23);
  }
  
  // no interrupt throttling for msix index i
  WriteEitr(i, 0x0);
  
  // 7.3.1.4 - Note that there are no EIAC(1)...EIAC(2) registers.
  // The hardware setting for interrupts 16...63 is always auto clear.
  if(i < 16) {
    // enable auto clear
    WriteEiac(0x1 << i);
  }
  
  // enable interrupt
  WriteEimsn(i / 32, (0x1 << (i % 32)));
  
  // make sure interupt is cleared
  if (i < 16) {
    WriteEicr(0x1 << i);
  }

  // Enable RX
  // disable RX_DIS
  WriteSecrxctrl_Rx_Dis(0x1 << 1);
  // TODO Timeout
  while (ReadSecrxstat_Sr_Rdy() == 0);  
  WriteRxctrl(0x1);
  // enable RX_DIS
  WriteSecrxctrl_Rx_Dis(0x0 << 1);
  //ebbrt::kprintf("RX enabled\n");
  
  // add buffer to each descriptor
  for (auto j = 0; j < 256 - 1; j++) {
    auto rxphys =reinterpret_cast<uint64_t>((ixgmq[i]->circ_buffer_[j])->MutData());
    auto tail = ixgmq[i]->rx_tail_;
    
    // update buffer address for descriptor
    ixgmq[i]->rx_ring_[tail].buffer_address = rxphys;
    ixgmq[i]->rx_tail_ = (tail + 1) % ixgmq[i]->rx_size_;
  }

  // bump tail pts via register rdt to enable descriptor fetching by setting to
  // length of ring minus one
  WriteRdt_1(i, ixgmq[i]->rx_tail_);
  
  //ebbrt::kprintf("RX Queue setup complete - head=%d tail=%d\n", ixgmq[i]->rx_head_,
  //ixgmq[i]->rx_tail_);
  
  // program base address registers
  WriteTdbal(i, ixgmq[i]->txaddr_ & 0xFFFFFFFF);
  WriteTdbah(i, (ixgmq[i]->txaddr_ >> 32) & 0xFFFFFFFF);

  // length must also be 128 byte aligned
  WriteTdlen(i, ixgmq[i]->tx_size_bytes_);

   //Head_WB_En = 1
  WriteTdwbal(i, (ixgmq[i]->txhwbaddr_ & 0xFFFFFFFF) | 0x1);
  WriteTdwbah(i, (ixgmq[i]->txhwbaddr_ >> 32) & 0xFFFFFFFF);
  
  // enable transmit path
  WriteDmatxctl_te(0x1);
  
  // transmit queue enable
  WriteTxdctl(i, 0x1 << 25);

  // poll until set, TODO: Timeout
  while (ReadTxdctl_enable(i) == 0);
  //ebbrt::kprintf("TX queue enabled\n");
  
  // TODO: set up dca txctrl FreeBSD?
  WriteDcaTxctrlTxdescWbro(i, ~(0x1 << 11));  // clear TXdescWBROen

  ebbrt::kprintf("%s DONE\n\n", __FUNCTION__);
}

void ebbrt::IxgbeDriver::SetupQueue(uint32_t i) {  
  // allocate memory for descriptor rings
  ixgq = (e10k_queue_t*)malloc(sizeof(e10k_queue_t));

  // TX
  auto tx_size = sizeof(tdesc_legacy_t) * NTXDESCS;
  ixgq->tx_ring = (tdesc_legacy_t*)malloc(tx_size);
  //ixgq->tx_head = 0;
  //TODO: not sure how much to allocate
  ixgq->tx_head = (uint32_t*)malloc(4 * sizeof(uint32_t));
  memset(ixgq->tx_head, 0, 4 * sizeof(uint32_t));
  
  ixgq->tx_tail = 0;
  ixgq->tx_last_tail = 0;
  ixgq->tx_isctx = (bool*)malloc(NTXDESCS * sizeof(bool));
  ixgq->tx_size = NTXDESCS;
  
  // RX - allocate a ring of 256 receive legacy descriptors
  auto rx_size = sizeof(rdesc_legacy_t) * NRXDESCS;
  ixgq->rx_ring = (rdesc_legacy_t*)malloc(rx_size);

  // head and tail point to same descriptor
  ixgq->rx_head = 0;
  ixgq->rx_tail = 0;
  ixgq->rx_size = NRXDESCS;
  ixgq->circ_buffer.reserve(NRXDESCS);
  
  for(uint32_t k=0; k < NRXDESCS; k ++)
  {
    ixgq->circ_buffer.emplace_back(MakeUniqueIOBuf(RXBUFSZ, true));
  }

  // zero out rings
  memset(ixgq->tx_ring, 0, tx_size);
  memset(ixgq->rx_ring, 0, rx_size);

  ebbrt::kprintf("rx_ring addr: %p\ntx_ring: %p\n", ixgq->rx_ring,
                 ixgq->tx_ring);

  // Initialize RX queue in HW
  uint64_t rxaddr = reinterpret_cast<uint64_t>(ixgq->rx_ring);
  uint32_t rxaddrl = rxaddr & 0xFFFFFFFF;
  uint32_t rxaddrh = (rxaddr >> 32) & 0xFFFFFFFF;
  ebbrt::kprintf("rxaddr: %p rxaddrl: 0x%X rxaddrh: 0x%X\n", rxaddr, rxaddrl,
                 rxaddrh);

  // rxaddrl must be 128 byte aligned, lower 7 bits == 0
  
  //kassert((rxaddrl & 0x7F) == 0);

  // update register RDBAL, RDBAH with receive descriptor base address
  WriteRdbal_1(i, rxaddrl);
  WriteRdbah_1(i, rxaddrh);

  // length must also be 128 byte aligned
  //kassert((rx_size & 0x7F) == 0);
  // set to number of bytes allocated for receive descriptor ring
  WriteRdlen_1(i, rx_size);

  // program srrctl register
  WriteSrrctl_1(i, RXBUFSZ / 1024);  // bsizepacket = 2 KB
  WriteSrrctl_1_desctype(i, ~(0x7 << 25));  // desctype legacy
  WriteSrrctl_1(i, 0x1 << 28);  // Drop_En

  // TODO: enable rssctl

  // Set Enable bit in receive queue
  WriteRxdctl_1_enable(i, 0x1 << 25);
  while (ReadRxdctl_1_enable(i) == 0)  // till set
    ;  // TODO: Timeout

  // Set head and tail pointers
  WriteRdt_1(i, 0x0);
  WriteRdh_1(i, 0x0);

  ebbrt::kprintf("RX queue enabled\n");

  // setup RX interrupts for queue 0
  //auto rcv_vector =
  //  event_manager->AllocateVector([this]() { ebb_->ReceivePoll(); });
  //dev_.SetMsixEntry(i * 2, rcv_vector, i);  // TODO: fix

  //auto ii = i / 2;
  // if((i % 2) == 0) {
  WriteIvarAlloc0(i, 0);
  WriteIvarAllocval0(i, 0x1 << 7);
  WriteIvarAlloc1(i, 0 << 8);
  WriteIvarAllocval1(i, 0x1 << 15);
  //}

  // enable autoclear for queue 0
  WriteEiac(0x1 << i);

  // enable interrupt
  WriteEimsn(i, 0x1 << i);
  WriteEicr(0x1 << i);

  // Enable RX
  WriteSecrxctrl_Rx_Dis(0x1 << 1);  // disable RX_DIS
  while (ReadSecrxstat_Sr_Rdy() == 0)
    ;  // TODO Timeout
  WriteRxctrl(0x1);
  WriteSecrxctrl_Rx_Dis(0x0 << 1);  // enable RX_DIS

  ebbrt::kprintf("RX enabled\n");

  // Add RX Buffers to ring
  //rxbuf = malloc(RXBUFSZ * (NRXDESCS - 1));
  //kassert(rxbuf != NULL);
  //memset(rxbuf, 0, RXBUFSZ * (NRXDESCS - 1));

  //ebbrt::kprintf("Allocated RX buffer: %p\n", rxbuf);

  // add buffer to each descriptor
  for (auto j = 0; j < 256 - 1; j++) {
    
    /*auto rxphys =
        reinterpret_cast<uint64_t>(static_cast<char*>(rxbuf) + (i * RXBUFSZ));
    
    auto tail = ixgq->rx_tail;
    // update buffer address for descriptor
    ixgq->rx_ring[tail].buffer_address = rxphys;  */
    
    auto rxphys =reinterpret_cast<uint64_t>((ixgq->circ_buffer[j])->MutData());
    auto tail = ixgq->rx_tail;
    // update buffer address for descriptor
    ixgq->rx_ring[tail].buffer_address = rxphys;

    ebbrt::kprintf("Descriptor #%d: %p %p %p\n", tail, rxphys,
		   ixgq->rx_ring[tail].buffer_address, (ixgq->circ_buffer[j])->Data());

    ixgq->rx_tail = (tail + 1) % ixgq->rx_size;

    // TODO: check if queue is full
  }

  // bump tail pts via register rdt to enable descriptor fetching by setting to
  // length of ring minus one
  WriteRdt_1(i, ixgq->rx_tail);

  ebbrt::kprintf("RX Queue setup complete - head=%d tail=%d\n\n", ixgq->rx_head,
                 ixgq->rx_tail);

  // Initialize TX queue in HW
  uint64_t txaddr = reinterpret_cast<uint64_t>(ixgq->tx_ring);
  uint32_t txaddrl = txaddr & 0xFFFFFFFF;
  uint32_t txaddrh = (txaddr >> 32) & 0xFFFFFFFF;
  ebbrt::kprintf("txaddr: %p txaddrl: %p txaddrh: %p\n", txaddr, txaddrl,
                 txaddrh);
  // txaddrl must be 128 byte aligned, lower 7 bits == 0
  //kassert((txaddrl & 0x7F) == 0);

  // program base address registers
  WriteTdbal(i, txaddrl);
  WriteTdbah(i, txaddrh);

  // length must also be 128 byte aligned
  //kassert((tx_size & 0x7F) == 0);
  WriteTdlen(i, tx_size);

  // set up head wb
  uint64_t txhwbaddr = reinterpret_cast<uint64_t>(ixgq->tx_head);
  //kassert((txhwbaddr & 0x3) == 0); //lower 2 bits = 0
  uint32_t txhwbaddrl = txhwbaddr & 0xFFFFFFFF;
  uint32_t txhwbaddrh = (txhwbaddr >> 32) & 0xFFFFFFFF;
  WriteTdwbal(i, txhwbaddrl | 0x1); //HGead_WB_En = 1
  WriteTdwbah(i, txhwbaddrh);
  
  // init head and tail ptr
  WriteTdh(i, 0x0);
  WriteTdt(i, 0x0);
  
  // enable transmit path
  WriteDmatxctl_te(0x1);
  
  // transmit queue enable
  WriteTxdctl(i, 0x1 << 25);
  // poll until set, TODO: Timeout
  while (ReadTxdctl_enable(i) == 0);
  ebbrt::kprintf("TX queue enabled\n");
  
  // TODO: set up dca txctrl FreeBSD?
  WriteDcaTxctrlTxdescWbro(i, ~(0x1 << 11));  // clear TXdescWBROen
}

// IxgbeDriverRep
uint32_t ebbrt::IxgbeDriverRep::GetRxBuf(uint32_t* len, uint64_t* bAddr) {
  rdesc_legacy_t tmp;
  tmp = ixgmq_.rx_ring_[ixgmq_.rx_head_];

  std::atomic_thread_fence(std::memory_order_release);

  // if got new packet
  if (tmp.dd && tmp.eop) {

    // set len and address
    *len = tmp.length;
    *bAddr = tmp.buffer_address;

    // reset descriptor
    ixgmq_.rx_ring_[ixgmq_.rx_head_].raw[0] = 0;
    ixgmq_.rx_ring_[ixgmq_.rx_head_].raw[1] = 0;

    // bump head ptr
    ixgmq_.rx_head_ = (ixgmq_.rx_head_ + 1) % ixgmq_.rx_size_;

    return 0;
  }
  return 1;
}

void ebbrt::IxgbeDriverRep::ReceivePoll() {
  uint32_t len;
  uint64_t bAddr;
  auto count = 0;

  // get address of buffer with data
  while (GetRxBuf(&len, &bAddr) == 0) {

    // done with buffer addr above, now to reuse it
    auto tail = ixgmq_.rx_tail_;
    ixgmq_.rx_ring_[tail].buffer_address = bAddr;

    // bump tail ptr
    ixgmq_.rx_tail_ = (tail + 1) % ixgmq_.rx_size_;

    count++;
    
    if (count > 0) {
      ebbrt::kprintf("** %s **\n", __FUNCTION__);
      root_.itf_.Receive(std::move(ixgmq_.circ_buffer_[ixgmq_.rx_tail_]));
    }
  }
  
  // TODO: Update tail register here or above?
  if(count > 0)
  {
    // update reg
    WriteRdt_1(Cpu::GetMine(), ixgmq_.rx_tail_);
    //ebbrt::kprintf("\t rx_head->%d rx_tail->%d \n", ixgq_.rx_head, ixgq_.rx_tail);
  }
}

ebbrt::IxgbeDriverRep::IxgbeDriverRep(const IxgbeDriver& root)
  : root_(root), ixgq_(root_.GetQueue()), ixgmq_(root.GetMultiQueue(Cpu::GetMine())),
    receive_callback_([this]() { ReceivePoll(); }) {
  ebbrt::kprintf("%s for Core %d\n", __FUNCTION__, Cpu::GetMine());
}

uint16_t ebbrt::IxgbeDriverRep::ReadRdh_1(uint32_t n) {
  auto reg = root_.bar0_.Read32(0x01010 + 0x40 * n);
  return reg & 0xFFFF;
}
void ebbrt::IxgbeDriverRep::WriteRdt_1(uint32_t n, uint32_t m) {
  root_.bar0_.Write32(0x01018 + 0x40 * n, m);
}

void ebbrt::IxgbeDriverRep::Run() {
  ebbrt::kprintf("%s\n", __FUNCTION__);
  while (1) {
    ReceivePoll();
  }
}
void ebbrt::IxgbeDriverRep::WriteTdt_1(uint32_t n, uint32_t m) {
  root_.bar0_.Write32(0x06018 + 0x40 * n, m);
}
