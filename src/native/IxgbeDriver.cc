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
#include <cinttypes>
#include <mutex>

//#define JUMBO_EN
#ifdef JUMBO_EN
#define JUMBO_SZ 9728 //largest size allowed
#endif
#define DCA_EN

void ebbrt::IxgbeDriver::Create(pci::Device& dev) {
  auto ixgbe_dev = new IxgbeDriver(dev);
  ixgbe_dev->ebb_ =
      IxgbeDriverRep::Create(ixgbe_dev, ebb_allocator->AllocateLocal());
  ixgbe_dev->Init();

  // cpu init must be called explicitly for creation core
  ixgbe_dev->CpuInit();
  
  //for(size_t i = 0; i < Cpu::Count(); i++) {
  //  ixgbe_dev->SetupMultiQueue(i);
  //}
  
  ebbrt::clock::SleepMilli(200);
  ebbrt::kprintf("intel 82599 card initialzed\n");
  //ebbrt::kabort("exiting\n");
}

ebbrt::IxgbeDriver::IxgbeDriver(pci::Device& dev)
  : itf_(network_manager->NewInterface(*this)), dev_(dev),
    bar0_(dev.GetBar(0)) {
  
  dev_.SetBusMaster(true);
  
  // set up interrupts, polling won't work after this
  auto msix = dev_.MsixEnable();
  kbugon(!msix, "Ixgbe without msix is unsupported\n");
  
  // each core gets a queue struct
  ebbrt::kprintf("Cpu Count = %d\n", Cpu::Count());
  
  auto rcv_vector =
    event_manager->AllocateVector([this]() { ebb_->ReceivePoll(); });

  ebbrt::kprintf("Interrupt Allocation: \n");
  for(size_t i = 0; i < Cpu::Count(); i++) {
    auto myapic = Cpu::GetByIndex(i)->apic_id();
    dev_.SetMsixEntry(i, rcv_vector, myapic);
    ebbrt::kprintf("Core: %d, Vector: %d, APIC: %d\n", i, rcv_vector, myapic);
  }
}
const ebbrt::EthernetAddress& ebbrt::IxgbeDriver::GetMacAddress() {
  return mac_addr_;
}

void ebbrt::IxgbeDriver::Send(std::unique_ptr<IOBuf> buf,
			      PacketInfo pinfo) {
  ebb_->Send(std::move(buf), std::move(pinfo));
}

void ebbrt::IxgbeDriver::CpuInit() {
  ebb_->SetupMultiQueue(ebbrt::Cpu::GetMine());
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

void ebbrt::IxgbeDriverRep::AddTx(const uint8_t *pa, uint64_t len, uint64_t totallen, bool first, bool last, uint8_t ctx, bool ip_cksum, bool tcpudp_cksum) {

  tdesc_advance_tx_rf_t* actx;

  auto tail = ixgmq_.tx_tail_;
  actx = reinterpret_cast<tdesc_advance_tx_rf_t *>(&(ixgmq_.tx_ring_[tail]));

  ixgmq_.tx_isctx_[tail] = false;

  actx->raw[0] = 0x0;
  actx->raw[1] = 0x0;
  
  actx->address = reinterpret_cast<uint64_t>(pa);
  actx->dtalen = len;
  if(first) {
    actx->paylen = totallen;
  }
  //ebbrt::kprintf("%s %p %d %d\n", __FUNCTION__, actx->address, actx->dtalen, actx->paylen);
  
  actx->dtyp = 0b0011;
  actx->dext = 1;

  // rs bit should only be set when eop is set
  if(last) {
    actx->rs = 1;
  }
  else {
    actx->rs = 0;
  }
  
  actx->ifcs = 1;
  
  if(last) {
    actx->eop = 1;
  }
  else {
    actx->eop = 0;
  }
  
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
  auto count = buf->CountChainElements();
  auto free_desc = IxgbeDriver::NTXDESCS - (ixgmq_.tx_tail_ - *(reinterpret_cast<uint64_t*>(ixgmq_.txhwbaddr_)));
  /*int c = static_cast<int>(Cpu::GetMine());
  if(c > 0) {
    ebbrt::kprintf("%s Core:%d\n", __FUNCTION__, c);
    }*/
  //ebbrt::kprintf("%s Core:%d len=%d count=%d free_desc=%d\n", __FUNCTION__, c, len, count, free_desc);
  
  ebbrt::kbugon(len >= 0xA0 * 1000, "%s packet len bigger than max ether length\n", __FUNCTION__);

  ebbrt::kbugon(free_desc < count, "%s not enough free descriptors\n", __FUNCTION__);
  
  // get a single buffer of data
  //auto txbuf = dp.Get(len * sizeof(uint8_t));
  
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

    // if buffer is chained
    if(buf->IsChained()) {
      size_t counter = 0;
      for(auto& buf_it : *buf) {	
        counter ++;
	
	//first buffer
	if(counter == 1) {
	  AddTx(buf_it.Data(), reinterpret_cast<uint64_t>(buf_it.Length()), len, true, false, 0, false, true);
	}
	else
	{
	  //last buffer
	  if(counter == count) {
	    AddTx(buf_it.Data(), reinterpret_cast<uint64_t>(buf_it.Length()), len, false, true, 0, false, true);
	  }
	  else {
	    AddTx(buf_it.Data(), reinterpret_cast<uint64_t>(buf_it.Length()), len, false, false, 0, false, true);
	  }
	}
      }
    }
    // not chained
    else {
      AddTx(buf->Data(), len, len, true, true, 0, false, true);
    }
  } 
  else {
    // if buffer is chained
    if(buf->IsChained()) {
      size_t counter = 0;
      for(auto& buf_it : *buf) {	
        counter ++;
	
	//first buffer
	if(counter == 1) {
	  AddTx(buf_it.Data(), reinterpret_cast<uint64_t>(buf_it.Length()), len, true, false, 0, false, false);
	}
	else
	{
	  //last buffer
	  if(counter == count) {
	    AddTx(buf_it.Data(), reinterpret_cast<uint64_t>(buf_it.Length()), len, false, true, 0, false, false);
	  }
	  else {
	    AddTx(buf_it.Data(), reinterpret_cast<uint64_t>(buf_it.Length()), len, false, false, 0, false, false);
	  }
	}
      }
    }
    // not chained
    else {
      AddTx(buf->Data(), len, len, true, true, 0, false, false);
    }
    //ebbrt::kabort("Always need checksum\n");
    //AddTx(len, true, true, -1, false, false);
  }
    
  // bump tx_tail
  WriteTdt_1(Cpu::GetMine(), ixgmq_.tx_tail_); // indicates position beyond last descriptor hw

  //ebbrt::kprintf("%s DONE\n", __FUNCTION__);
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

void ebbrt::IxgbeDriver::WriteRxctrl(uint32_t m) const{
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
void ebbrt::IxgbeDriver::WriteDmatxctl_te(uint32_t m) const{
  auto reg = bar0_.Read32(0x04A80);
  bar0_.Write32(0x04A80, reg | m);
}

// 8.2.3.5.18 - General Purpose Interrupt Enable — GPIE (0x00898; RW)
void ebbrt::IxgbeDriver::WriteGpie(uint32_t m) {
  uint32_t reg;
  reg = bar0_.Read32(0x00898);
  bar0_.Write32(0x00898, reg | m);
}

void ebbrt::IxgbeDriver::ReadGpie() {
  auto reg = bar0_.Read32(0x00898);
  ebbrt::kprintf("%s 0x%X\n", __FUNCTION__, reg);
}

void ebbrt::IxgbeDriver::ReadEicrCause() {
  auto reg = bar0_.Read32(0x00800);
  auto reg2 = bar0_.Read32(0x00804);
  ebbrt::kprintf("%s 0x%X 0x%X\n", __FUNCTION__, reg, reg2);
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
void ebbrt::IxgbeDriver::WriteEicr(uint32_t m) const{
  auto reg = bar0_.Read32(0x00800);
  //ebbrt::kprintf("%s 0x%X\n", __FUNCTION__, reg | m);
  bar0_.Write32(0x00800, reg | m);
}

// 8.2.3.5.3 Extended Interrupt Mask Set/Read Register- EIMS (0x00880; RWS)
void ebbrt::IxgbeDriver::WriteEims(uint32_t m) { bar0_.Write32(0x00880, m); }

// 8.2.3.5.4 Extended Interrupt Mask Clear Register- EIMC (0x00888; WO)
void ebbrt::IxgbeDriver::WriteEimc(uint32_t m) { bar0_.Write32(0x00888, m); }

// 8.2.3.5.5 Extended Interrupt Auto Clear Register — EIAC (0x00810; RW)
void ebbrt::IxgbeDriver::WriteEiac(uint32_t m) const{
  auto reg = bar0_.Read32(0x00810);
  bar0_.Write32(0x00810, reg | m);
}

// 8.2.3.5.8 Extended Interrupt Mask Set/Read Registers — EIMS[n] (0x00AA0 +
// 4*(n-1), n=1...2; RWS)
void ebbrt::IxgbeDriver::WriteEimsn(uint32_t n, uint32_t m) const{
  auto reg = bar0_.Read32(0x00AA0 + 4 * n);
  bar0_.Write32(0x00AA0 + 4 * n, reg | m);
}

// 8.2.3.5.12
// Extended Interrupt Throttle Registers — EITR[n]
// (0x00820 + 4*n, n=0...23 and 0x012300 + 4*(n-24),
// n=24...128; RW)
void ebbrt::IxgbeDriver::WriteEitr(uint32_t n, uint32_t m) const{

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
void ebbrt::IxgbeDriver::WriteTxdctl(uint32_t n, uint32_t m) const {
  // uint32_t reg;
  // reg = bar0_.Read32(0x06028+0x40*n);
  // ebbrt::kprintf("0x%05x: TXDCTL 0x%08X\n", 0x06028+0x40*n, reg);
  bar0_.Write32(0x06028 + (0x40 * n), m);
}
uint8_t ebbrt::IxgbeDriver::ReadTxdctl_enable(uint32_t n) const {
  auto reg = bar0_.Read32(0x06028 + 0x40 * n);
  return (reg >> 25) & 0x1;
}

// 8.2.3.8.6 Receive Descriptor Control — RXDCTL[n] (0x01028 +
// 0x40*n, n=0...63 and 0x0D028 + 0x40*(n-64), n=64...127; RW)
void ebbrt::IxgbeDriver::WriteRxdctl_1(uint32_t n, uint32_t m) {
  bar0_.Write32(0x01028 + (0x40 * n), m);
}
void ebbrt::IxgbeDriver::WriteRxdctl_1_enable(uint32_t n, uint32_t m) const {
  auto reg = bar0_.Read32(0x01028 + (0x40 * n));
  bar0_.Write32(0x01028 + (0x40 * n), reg | m);
}

uint8_t ebbrt::IxgbeDriver::ReadRxdctl_1_enable(uint32_t n) const {
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

// 8.2.3.22.13 Max Frame Size — MAXFRS (0x04268; RW)
void ebbrt::IxgbeDriver::WriteMaxfrs(uint32_t m) {
  auto reg = bar0_.Read32(0x04268);
  bar0_.Write32(0x04268, reg | m);
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

// 8.2.3.11.4 DCA Control Register — DCA_CTRL (0x11074; RW)
void ebbrt::IxgbeDriver::WriteDcaCtrl(uint32_t m) {
  auto reg = bar0_.Read32(0x11074);
  bar0_.Write32(0x11074, reg | m);
}

// 8.2.3.11.2 Tx DCA Control Registers — DCA_TXCTRL[n] (0x0600C + 0x40*n,
// n=0...127; RW)
void ebbrt::IxgbeDriver::WriteDcaTxctrl(uint32_t n, uint32_t m) const{
  auto reg = bar0_.Read32(0x0600C + 0x40 * n);
  bar0_.Write32(0x0600C + 0x40 * n, reg | m);
  //ebbrt::kprintf("%s 0x%X\n", __FUNCTION__, reg);
}
void ebbrt::IxgbeDriver::WriteDcaTxctrlTxdescWbro(uint32_t n, uint32_t m) const{
  auto reg = bar0_.Read32(0x0600C + 0x40 * n);
  // ebbrt::kprintf("%s: 0x%08X -> 0x%08X\n", __FUNCTION__, reg, reg & m);
  bar0_.Write32(0x0600C + 0x40 * n, reg & m);
}

// 8.2.3.11.1 Rx DCA Control Register — DCA_RXCTRL[n] (0x0100C + 0x40*n,
// n=0...63 and 0x0D00C + 0x40*(n-64),
// n=64...127 / 0x02200 + 4*n, [n=0...15]; RW)
void ebbrt::IxgbeDriver::WriteDcaRxctrl(uint32_t n, uint32_t m) const {
  auto reg = bar0_.Read32(0x0100C + 0x40 * n);
  bar0_.Write32(0x0100C + 0x40 * n, reg | m);
} 
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
void ebbrt::IxgbeDriver::WriteRdbal_1(uint32_t n, uint32_t m) const {
  //ebbrt::kprintf("%s 0x%X\n", __FUNCTION__, m);
  bar0_.Write32(0x01000 + 0x40 * n, m);
}

void ebbrt::IxgbeDriver::WriteRdbal_2(uint32_t n, uint32_t m) const {
  bar0_.Write32(0x0D000 + 0x40 * n, m);
}

// 8.2.3.8.2 Receive Descriptor Base Address High — RDBAH[n] (0x01004 + 0x40*n,
// n=0...63 and 0x0D004 + 0x40*(n-64), n=64...127; RW)
void ebbrt::IxgbeDriver::WriteRdbah_1(uint32_t n, uint32_t m) const {
  bar0_.Write32(0x01004 + 0x40 * n, m);
}

void ebbrt::IxgbeDriver::WriteRdbah_2(uint32_t n, uint32_t m) const {
  bar0_.Write32(0x0D004 + 0x40 * n, m);
}

// 8.2.3.9.5 Transmit Descriptor Base Address Low — TDBAL[n] (0x06000+0x40*n,
// n=0...127; RW)
void ebbrt::IxgbeDriver::WriteTdbal(uint32_t n, uint32_t m) const {
  bar0_.Write32(0x06000 + 0x40 * n, m);
}

// 8.2.3.9.6 Transmit Descriptor Base Address High — TDBAH[n] (0x06004+0x40*n,
// n=0...127; RW)
void ebbrt::IxgbeDriver::WriteTdbah(uint32_t n, uint32_t m) const{
  bar0_.Write32(0x06004 + 0x40 * n, m);
}

// 8.2.3.9.7 Transmit Descriptor Length — TDLEN[n] (0x06008+0x40*n, n=0...127;
// RW)
void ebbrt::IxgbeDriver::WriteTdlen(uint32_t n, uint32_t m) const{
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
void ebbrt::IxgbeDriver::WriteTdwbal(uint32_t n, uint32_t m) const{
  bar0_.Write32(0x06038 + 0x40 * n, m);
}
// 8.2.3.9.12 Tx Descriptor Completion Write Back Address High —
// TDWBAH[n] (0x0603C+0x40*n, n=0...127; RW)
void ebbrt::IxgbeDriver::WriteTdwbah(uint32_t n, uint32_t m) const{
  bar0_.Write32(0x0603C + 0x40 * n, m);
}

// 8.2.3.9.9 Transmit Descriptor Tail — TDT[n] (0x06018+0x40*n, n=0...127; RW)
void ebbrt::IxgbeDriver::WriteTdt(uint32_t n, uint32_t m) {
  bar0_.Write32(0x06018 + 0x40 * n, m);
}

// 8.2.3.8.3 Receive Descriptor Length — RDLEN[n] (0x01008 + 0x40*n, n=0...63
// and 0x0D008 + 0x40*(n-64), n=64...127; RW)
void ebbrt::IxgbeDriver::WriteRdlen_1(uint32_t n, uint32_t m) const {
  //ebbrt::kprintf("%s %d\n", __FUNCTION__, m);
  bar0_.Write32(0x01008 + 0x40 * n, m);
}
void ebbrt::IxgbeDriver::WriteRdlen_2(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0D008 + 0x40 * n, m);
}

// 8.2.3.8.7 Split Receive Control Registers — SRRCTL[n] (0x01014 + 0x40*n,
// n=0...63 and 0x0D014 + 0x40*(n-64), n=64...127 / 0x02100 + 4*n, [n=0...15];
// RW)
void ebbrt::IxgbeDriver::WriteSrrctl_1(uint32_t n, uint32_t m) const {
  auto reg = bar0_.Read32(0x01014 + 0x40 * n);
  bar0_.Write32(0x01014 + 0x40 * n, reg | m);
}
/*void ebbrt::IxgbeDriver::WriteSrrctl_1_bsizepacket(uint32_t n, uint32_t m) {
    auto reg = bar0_.Read32(0x01014 + 0x40*n);
    bar0_.Write32(0x01014 + 0x40*n, reg | m);
    }*/

void ebbrt::IxgbeDriver::WriteSrrctl_1_desctype(uint32_t n, uint32_t m) const {
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
void ebbrt::IxgbeDriver::WriteRdt_1(uint32_t n, uint32_t m) const {
  bar0_.Write32(0x01018 + 0x40 * n, m);
}
void ebbrt::IxgbeDriver::WriteRdt_2(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0D018 + 0x40 * n, m);
}

// 8.2.3.8.4 Receive Descriptor Head — RDH[n] (0x01010 + 0x40*n, n=0...63 and
// 0x0D010 + 0x40*(n-64), n=64...127; RO)
void ebbrt::IxgbeDriver::WriteRdh_1(uint32_t n, uint32_t m)  const {
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
void ebbrt::IxgbeDriver::WriteIvarAlloc0(uint32_t n, uint32_t m) const {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  //reg = reg & ~(0x3F);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
}
void ebbrt::IxgbeDriver::WriteIvarAllocval0(uint32_t n, uint32_t m) const {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
}

void ebbrt::IxgbeDriver::WriteIvarAlloc1(uint32_t n, uint32_t m){
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  //reg = reg & ~(0x3F << 8);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
}
void ebbrt::IxgbeDriver::WriteIvarAllocval1(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
}

void ebbrt::IxgbeDriver::WriteIvarAlloc2(uint32_t n, uint32_t m) const {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  //reg = reg & ~(0x3F << 8);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
}
void ebbrt::IxgbeDriver::WriteIvarAllocval2(uint32_t n, uint32_t m) const {
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
void ebbrt::IxgbeDriver::WriteSecrxctrl_Rx_Dis(uint32_t m) const{
  auto reg = bar0_.Read32(0x08D00);
  if (m) {
    bar0_.Write32(0x08D00, reg | m);
  } else {
    bar0_.Write32(0x08D00, reg & ~(0x1 << 1));
  }
}

// 8.2.3.12.6 Security Rx Status — SECRXSTAT (0x08D04; RO)
uint8_t ebbrt::IxgbeDriver::ReadSecrxstat_Sr_Rdy() const{
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

  // jumbo frames
#ifdef JUMBO_EN
  WriteHlreg0(0x1 << 2);
  WriteMaxfrs(JUMBO_SZ << 16);
  ebbrt::kprintf("Jumbo frames enabled of size: 0x%X\n", JUMBO_SZ);
#endif
  
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

#ifdef DCA_EN
  //DCA_MODE = DCA 1.0
  WriteDcaCtrl(0x1 << 1);
  ebbrt::kprintf("DCA enabled\n");
#endif
}

void ebbrt::IxgbeDriverRep::SetupMultiQueue(uint32_t i) {
  std::lock_guard<SpinLock> lock(lock_);
  
  //ebbrt::kprintf("**** %s for Core %d\n", __FUNCTION__, i);
  
  // not going to set up receive descripts greater than 63
  ebbrt::kbugon(i >= 64, "can't set up descriptors greater than 63\n");

  // update register RDBAL, RDBAH with receive descriptor base address
  root_.WriteRdbal_1(i, ixgmq_.rxaddr_ & 0xFFFFFFFF);
  
  //root_.WriteRdbah_1(i, (ixgmq_.rxaddr_ >> 32) & 0xFFFFFFFF);

  // set to number of bytes allocated for receive descriptor ring
  root_.WriteRdlen_1(i, ixgmq_.rx_size_bytes_);
  
  // program srrctl register
  root_.WriteSrrctl_1(i, IxgbeDriver::RXBUFSZ / 1024);  // bsizepacket
  //WriteSrrctl_1(i, (128 / 64) << 8);  // bsizeheader 
  
  // TODO use adv?
  root_.WriteSrrctl_1_desctype(i, ~(0x7 << 25));  // desctype legacy
  root_.WriteSrrctl_1(i, 0x1 << 28);  // Drop_En
  
  // Set Enable bit in receive queue
  root_.WriteRxdctl_1_enable(i, 0x1 << 25);
  // till set
  // TODO: Timeout
  while (root_.ReadRxdctl_1_enable(i) == 0);
  
  // Set head and tail pointers
  root_.WriteRdt_1(i, 0x0);
  root_.WriteRdh_1(i, 0x0);
  
  // don't set up interrupts for tx since we have head writeback
  auto qn = i / 2; // put into correct IVAR
  
  if((i % 2) == 0) { // check if 2xN or 2xN + 1
    root_.WriteIvarAlloc0(qn, i); // rx interrupt allocation corresponds to index i * 2 in MSI-X table
    
    root_.WriteIvarAllocval0(qn, 0x1 << 7);
    
    //ebbrt::kprintf("IVAR %d, INT_Alloc 0x%X, INT_Alloc_val 0x%X\n", qn, i, 0x1 << 7);
  }
  else {
    root_.WriteIvarAlloc2(qn, i << 16);
    
    root_.WriteIvarAllocval2(qn, 0x1 << 23);
    
    //ebbrt::kprintf("IVAR %d, INT_Alloc 0x%X, INT_Alloc_val 0x%X\n", qn, i << 16, 0x1 << 23);
  }
  
  // no interrupt throttling for msix index i
  root_.WriteEitr(i, 0x0);
  
  // 7.3.1.4 - Note that there are no EIAC(1)...EIAC(2) registers.
  // The hardware setting for interrupts 16...63 is always auto clear.
  if(i < 16) {
    // enable auto clear
    root_.WriteEiac(0x1 << i);
  }
  
  // enable interrupt
  root_.WriteEimsn(i / 32, (0x1 << (i % 32)));
  
  // make sure interupt is cleared
  if (i < 16) {
    root_.WriteEicr(0x1 << i);
  }
  
  // Enable RX
  // disable RX_DIS
  root_.WriteSecrxctrl_Rx_Dis(0x1 << 1);
  // TODO Timeout
  while (root_.ReadSecrxstat_Sr_Rdy() == 0);  
  root_.WriteRxctrl(0x1);
  // enable RX_DIS
  root_.WriteSecrxctrl_Rx_Dis(0x0 << 1);
  //ebbrt::kprintf("RX enabled\n");
  
  // add buffer to each descriptor
  for (size_t j = 0; j < IxgbeDriver::NRXDESCS - 1; j++) {
    auto rxphys =reinterpret_cast<uint64_t>((ixgmq_.circ_buffer_[j])->MutData());
    auto tail = ixgmq_.rx_tail_;
    
    // update buffer address for descriptor
    ixgmq_.rx_ring_[tail].buffer_address = rxphys;
    ixgmq_.rx_tail_ = (tail + 1) % ixgmq_.rx_size_;
  }

  // bump tail pts via register rdt to enable descriptor fetching by setting to
  // length of ring minus one
  root_.WriteRdt_1(i, ixgmq_.rx_tail_);

#ifdef DCA_EN
  {
    auto myapic = ebbrt::Cpu::GetByIndex(i)->apic_id();
    
    root_.WriteDcaRxctrl(i, 0x1 << 5); //Descriptor DCA EN
    root_.WriteDcaRxctrl(i, 0x1 << 6); //Rx Header DCA EN
    root_.WriteDcaRxctrl(i, 0x1 << 7); //Payload DCA EN
    
    root_.WriteDcaRxctrl(i, myapic << 24); // CPUID = apic id
    //printf("DCA enabled on RX queue Core %d with APIC ID %d\n", i, myapic);
  }
#endif
  
  //ebbrt::kprintf("RX Queue setup complete - head=%d tail=%d\n", ixgmq[i]->rx_head_,
  //ixgmq[i]->rx_tail_);
  
  // program base address registers
  root_.WriteTdbal(i, ixgmq_.txaddr_ & 0xFFFFFFFF);
  root_.WriteTdbah(i, (ixgmq_.txaddr_ >> 32) & 0xFFFFFFFF);

  // length must also be 128 byte aligned
  root_.WriteTdlen(i, ixgmq_.tx_size_bytes_);

   //Head_WB_En = 1
  root_.WriteTdwbal(i, (ixgmq_.txhwbaddr_ & 0xFFFFFFFF) | 0x1);
  root_.WriteTdwbah(i, (ixgmq_.txhwbaddr_ >> 32) & 0xFFFFFFFF);
  
  // enable transmit path
  root_.WriteDmatxctl_te(0x1);
  
  // transmit queue enable
  root_.WriteTxdctl(i, 0x1 << 25);

  // poll until set, TODO: Timeout
  while (root_.ReadTxdctl_enable(i) == 0);
  //ebbrt::kprintf("TX queue enabled\n");
  
  // not sure why need to disable relax ordering
  // do not want device to reorder messages, could have race issues
  //root_.WriteDcaTxctrlTxdescWbro(i, ~(0x1 << 11));  // clear TXdescWBROen

#ifdef DCA_EN
  {
    auto myapic = ebbrt::Cpu::GetByIndex(i)->apic_id();
    root_.WriteDcaTxctrl(i, 0x1 << 5); //DCA Enable
    root_.WriteDcaTxctrl(i, myapic << 24); // CPUID = apic id
    //printf("DCA enabled on TX queue Core %d with APIC ID %d\n", i, myapic);
  }
#endif
  
  //ebbrt::kprintf("%s DONE\n\n", __FUNCTION__);
}

// IxgbeDriverRep
uint32_t ebbrt::IxgbeDriverRep::GetRxBuf(uint32_t* len, uint64_t* bAddr) {
  rdesc_legacy_t tmp;
  tmp = ixgmq_.rx_ring_[ixgmq_.rx_head_];

  //std::atomic_thread_fence(std::memory_order_release);

  // if got new packet
  if (tmp.dd && tmp.eop) {

    // set len and address
    *len = tmp.length;
    //*bAddr = tmp.buffer_address;

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
    //ixgmq_.rx_ring_[tail].buffer_address = bAddr;

    // bump tail ptr
    ixgmq_.rx_tail_ = (tail + 1) % ixgmq_.rx_size_;
    
    count++;
    
    if (count > 0) {
      auto tail = ixgmq_.rx_tail_;
      
      //TODO hack - need to set actual length of data
      ixgmq_.circ_buffer_[tail]->SetLength(len);

      //TODO hack - need to reallocate IOBuf after its been moved to Receive
      auto b = std::move(ixgmq_.circ_buffer_[tail]);
      
      ixgmq_.circ_buffer_[tail] = std::move(MakeUniqueIOBuf(IxgbeDriver::RXBUFSZ));
      auto rxphys = reinterpret_cast<uint64_t>((ixgmq_.circ_buffer_[tail])->MutData());
      
      ixgmq_.rx_ring_[tail].buffer_address = rxphys;
      //ReadEicr();
      //ReadEims();
      root_.itf_.Receive(std::move(b));
    }
  }
  
  // TODO: Update tail register here or above?
  if(count > 0)
  {
    // update reg
    WriteRdt_1(Cpu::GetMine(), ixgmq_.rx_tail_);
  }
}

ebbrt::IxgbeDriverRep::IxgbeDriverRep(const IxgbeDriver& root)
  : root_(root),
    ixgmq_(Cpu::GetMine(), Cpu::GetMyNode()),
    //TODO const_cast potential undefined behavior
    lock_(const_cast<SpinLock&>(root.lock)),
    receive_callback_([this]() { this->ReceivePoll(); }) {
  //int c = static_cast<int>(Cpu::GetMine());
  //ebbrt::kprintf("%s for Core %d\n", __FUNCTION__, c);
}

uint16_t ebbrt::IxgbeDriverRep::ReadRdh_1(uint32_t n) {
  auto reg = root_.bar0_.Read32(0x01010 + 0x40 * n);
  return reg & 0xFFFF;
}
void ebbrt::IxgbeDriverRep::WriteRdt_1(uint32_t n, uint32_t m) {
  root_.bar0_.Write32(0x01018 + 0x40 * n, m);
}

void ebbrt::IxgbeDriverRep::Run() {
  //ebbrt::kprintf("%s\n", __FUNCTION__);
  while (1) {
    ReceivePoll();
  }
}
void ebbrt::IxgbeDriverRep::WriteTdt_1(uint32_t n, uint32_t m) {
  root_.bar0_.Write32(0x06018 + 0x40 * n, m);
}

// 8.2.3.5.9 Extended Interrupt Mask Clear Registers — EIMC[n]
// (0x00AB0 + 4*(n-1), n=1...2; WO)
void ebbrt::IxgbeDriverRep::WriteEimcn(uint32_t n, uint32_t m) {
  auto reg = root_.bar0_.Read32(0x00AB0 + 4 * n);
  root_.bar0_.Write32(0x00AB0 + 4 * n, reg | m);
}

// 8.2.3.5.1 Extended Interrupt Cause Register- EICR (0x00800; RW1C)
void ebbrt::IxgbeDriverRep::ReadEicr() {
  auto reg = root_.bar0_.Read32(0x00800);
  ebbrt::kprintf("%s 0x%X\n", __FUNCTION__, reg);
}

void ebbrt::IxgbeDriverRep::ReadEims() {
  auto reg = root_.bar0_.Read32(0x00880);
  ebbrt::kprintf("%s 0x%X\n", __FUNCTION__, reg);
}
