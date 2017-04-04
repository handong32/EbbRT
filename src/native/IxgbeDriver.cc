//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#include "IxgbeDriver.h"

#include "../StaticIOBuf.h"
#include "../UniqueIOBuf.h"
#include "../Align.h"
#include "Fls.h"
#include "PageAllocator.h"
#include "Pfn.h"
#include "Clock.h"
#include "Debug.h"
#include "EventManager.h"
#include "Ixgbe.h"

#include <atomic>

static uint8_t txbuf2[128] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
			      0x90, 0xE2, 0xBA, 0x82, 0x33, 0x24,
			      0x80, 0x00, 0xDE, 0xAD, 0xBE, 0xFF,
			      0xDE, 0xAD, 0xBE, 0xEF, 0x22, 0x22,
			      0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
			      0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
			      0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
			      0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
			      0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
			      0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
			      0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
			      0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
			      0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
			      0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
			      0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
			      0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
			      0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
			      0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
			      0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
			      0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
			      0x22, 0x22, 0x22, 0x22, 0x22, 0x22
};

static uint8_t txbuf3[90] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			     0x90, 0xE2, 0xBA, 0x82, 0x33, 0x24,
			     0x86, 0xDD, 0x60, 0x00, 0x00, 0x00,
			     0x00, 0x24, 0x00, 0x01, 0x00, 0x00,
			     0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			     0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			     0x00, 0x00, 0xFF, 0x02, 0x00, 0x00,
			     0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			     0x00, 0x00, 0x00, 0x00, 0x00, 0x16,
			     0x3A, 0x00, 0x05, 0x02, 0x00, 0x00,
			     0x01, 0x00, 0x8F, 0x00, 0x3B, 0xE4,
			     0x00, 0x00, 0x00, 0x01, 0x04, 0x00,
			     0x00, 0x00, 0xFF, 0x02, 0x00, 0x00,
			     0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			     0x00, 0x01, 0xFF, 0x82, 0x33, 0x24
};

void ebbrt::IxgbeDriver::Create(pci::Device& dev) {
  auto ixgbe_dev = new IxgbeDriver(dev);

  // ixgbe_dev->Init();
  ixgbe_dev->ixgbe_probe();
  ixgbe_dev->ixgbe_open();
  
  ixgbe_dev->ebb_ =
      IxgbeDriverRep::Create(ixgbe_dev, ebb_allocator->AllocateLocal());

  // ixgbe_dev->SetupQueue(0);
  ebbrt::clock::SleepMilli(200);
  ebbrt::kprintf("intel 82599 card initialzed\n");

  // Send test packet
  ixgbe_dev->SendPacket(0);

  mdelay(100);
  /*while (1) {
    // ebbrt::clock::SleepMilli(10000);
    // ebbrt::kprintf("Slept 10s: ");
    // ixgbe_dev->ReadGprc();
    ixgbe_dev->ProcessPacket(0);
    }*/

  // ixgbe_dev->Run();
}

const ebbrt::EthernetAddress& ebbrt::IxgbeDriver::GetMacAddress() {
  return mac_addr_;
}

void ebbrt::IxgbeDriver::Send(std::unique_ptr<IOBuf> buf, PacketInfo pinfo) {
  ebb_->Send(std::move(buf), std::move(pinfo));
}

void ebbrt::IxgbeDriver::Run() { ebb_->Run(); }

void ebbrt::IxgbeDriverRep::Send(std::unique_ptr<IOBuf> buf, PacketInfo pinfo) {

  // std::unique_ptr<StaticIOBuf> b;
  // auto b = MakeUniqueIOBuf(0);
  VirtioNetHeader* header;

  // auto len = buf->ComputeChainDataLength();
  // ebbrt::kprintf("%s chain elements = %d, %d\n", __PRETTY_FUNCTION__,
  // buf->CountChainElements(), buf->ComputeChainDataLength());

  // auto dp = buf->GetDataPointer();
  // auto len = buf->ComputeChainDataLength();
  // auto txbuf = dp.Get(len * sizeof(uint8_t));

  /*auto t = dp.Get(buf->ComputeChainDataLength() * sizeof(uint8_t));
  for(size_t i = 0; i < buf->ComputeChainDataLength(); i++)
  {
      ebbrt::kprintf("%02X ", t[i]);
  }
  ebbrt::kprintf("\n\n");*/

  // we have enough descriptors to avoid a copy
  auto b = MakeUniqueIOBuf(sizeof(VirtioNetHeader), true);
  header = reinterpret_cast<VirtioNetHeader*>(b->MutData());
  b->PrependChain(std::move(buf));

  // auto len = buf->ComputeChainDataLength();

  kassert(header != nullptr);
  if (pinfo.flags & PacketInfo::kNeedsCsum) {
    header->flags |= VirtioNetHeader::kNeedsCsum;
    header->csum_start = pinfo.csum_start;
    header->csum_offset = pinfo.csum_offset;
  }
  if (pinfo.gso_type != PacketInfo::kGsoNone) {
    header->gso_type = pinfo.gso_type;
    header->hdr_len = pinfo.hdr_len;
    header->gso_size = pinfo.gso_size;
  }

  // auto elements = b->CountChainElements();
  // uint64_t tmp1 = static_cast<uint64_t>(elements);

  // auto txphys = reinterpret_cast<uint64_t>(b->MutData());
  // auto txbuf = (uint8_t*)malloc(sizeof(uint8_t) * len);
  // memset(txbuf, 0, len * sizeof(uint8_t));
  // auto dp = buf->GetDataPointer();

  auto dp = b->GetDataPointer();
  auto len = b->ComputeChainDataLength();
  auto txbuf = dp.Get(len * sizeof(uint8_t));

  auto tail = ixgq_.tx_tail;
  // update buffer address for descriptor
  ixgq_.tx_ring[tail].buffer_address = reinterpret_cast<uint64_t>(txbuf);
  ixgq_.tx_ring[tail].length = len;
  ixgq_.tx_ring[tail].eop = 1;  // indicate end of packet
  // so that hw will modify dd bit after packet transmitted
  ixgq_.tx_ring[tail].rs = 1;

  ebbrt::kprintf("taddr - %p %p len:%ld\n", txbuf,
                 ixgq_.tx_ring[tail].buffer_address,
                 ixgq_.tx_ring[tail].length);

  for (size_t i = 0; i < len; i++) {
    ebbrt::kprintf("%02X ", txbuf[i]);
  }
  ebbrt::kprintf("\n\n");

  auto n = 0;
  // bump tx_tail
  ixgq_.tx_tail = (tail + 1) % ixgq_.tx_size;
  WriteTdt_1(n, ixgq_.tx_tail);  // indicates position beyond last descriptor hw
  // can process

  auto head = ixgq_.tx_head;
  // std::atomic_thread_fence(std::memory_order_release);
  ebbrt::kprintf("dd = %d\n", ixgq_.tx_ring[head].dd);
  while (ixgq_.tx_ring[head].dd == 0) {
  }  // hw will transmit packet pointed by head, after tranmission, dd bit is
  // set and head is incremented, finish processing when head == tail

  // header should be incremented by hw, need to update tx_head
  // ebbrt::kprintf("header = %d, %d\n", head, ReadTdh(n));
  ixgq_.tx_head = (head + 1) % ixgq_.tx_size;

  ebbrt::kprintf("TX dma complete - tx_head = %d tx_tail = %d\n", ixgq_.tx_head,
                 ixgq_.tx_tail);
}

void ebbrt::IxgbeDriver::ixgbe_regdump(struct ixgbe_reg_info *reginfo) {
    int i = 0, j = 0;
    char rname[16];
    u32 regs[64];
    
    switch (reginfo->ofs) {
    case IXGBE_SRRCTL(0):
	for (i = 0; i < 64; i++)
	    regs[i] = IXGBE_READ_REG(IXGBE_SRRCTL(i));
	break;
    case IXGBE_DCA_RXCTRL(0):
	for (i = 0; i < 64; i++)
	    regs[i] = IXGBE_READ_REG(IXGBE_DCA_RXCTRL(i));
	break;
    case IXGBE_RDLEN(0):
	for (i = 0; i < 64; i++)
	    regs[i] = IXGBE_READ_REG(IXGBE_RDLEN(i));
	break;
    case IXGBE_RDH(0):
	for (i = 0; i < 64; i++)
	    regs[i] = IXGBE_READ_REG(IXGBE_RDH(i));
	break;
    case IXGBE_RDT(0):
	for (i = 0; i < 64; i++)
	    regs[i] = IXGBE_READ_REG(IXGBE_RDT(i));
	break;
    case IXGBE_RXDCTL(0):
	for (i = 0; i < 64; i++)
	    regs[i] = IXGBE_READ_REG(IXGBE_RXDCTL(i));
	break;
    case IXGBE_RDBAL(0):
	for (i = 0; i < 64; i++)
	    regs[i] = IXGBE_READ_REG(IXGBE_RDBAL(i));
	break;
    case IXGBE_RDBAH(0):
	for (i = 0; i < 64; i++)
	    regs[i] = IXGBE_READ_REG(IXGBE_RDBAH(i));
	break;
    case IXGBE_TDBAL(0):
	for (i = 0; i < 64; i++)
	    regs[i] = IXGBE_READ_REG(IXGBE_TDBAL(i));
	break;
    case IXGBE_TDBAH(0):
	for (i = 0; i < 64; i++)
	    regs[i] = IXGBE_READ_REG(IXGBE_TDBAH(i));
	break;
    case IXGBE_TDLEN(0):
	for (i = 0; i < 64; i++)
	    regs[i] = IXGBE_READ_REG(IXGBE_TDLEN(i));
	break;
    case IXGBE_TDH(0):
	for (i = 0; i < 64; i++)
	    regs[i] = IXGBE_READ_REG(IXGBE_TDH(i));
	break;
    case IXGBE_TDT(0):
	for (i = 0; i < 64; i++)
	    regs[i] = IXGBE_READ_REG(IXGBE_TDT(i));
	break;
    case IXGBE_TXDCTL(0):
	for (i = 0; i < 64; i++)
	    regs[i] = IXGBE_READ_REG(IXGBE_TXDCTL(i));
	break;
    default:
        KPRINTF("%-15s %08x\n", reginfo->name,
		IXGBE_READ_REG(reginfo->ofs));
	return;
    }
    
	for (i = 0; i < 8; i++) {
	    KPRINTF("%s[%d-%d]", reginfo->name, i*8, i*8+7);
	    KPRINTF("%-15s", rname);
	    for (j = 0; j < 8; j++) {
		KPRINTF(" %08x", regs[i*8+j]);
	    }
	    KPRINTF("\n");
        }

}

void ebbrt::IxgbeDriver::ixgbe_dump() {
    struct ixgbe_reg_info *reginfo;

    KPRINTF("Register Dump\n");
    KPRINTF(" Register Name   Value\n");
    
    for (reginfo = (struct ixgbe_reg_info *)ixgbe_reg_info_tbl;
	 reginfo->name; reginfo++) 
    {
	ixgbe_regdump(reginfo);
    }

}
void ebbrt::IxgbeDriver::SendPacket(uint32_t n) {
    //u32 tlen = 64;

    KPRINTF("%s\n", __FUNCTION__);

    tx_ring2->buffer_address = reinterpret_cast<uint64_t>(hantxbuf);
    tx_ring2->word2_raw = 0xB000156;

    /*tx_ring2->length = tlen;
    tx_ring2->dext = 0;
    tx_ring2->eop = 1;
    tx_ring2->
    tx_ring2->rs = 1;*/
    
    //tx_ring2->read.buffer_addr = reinterpret_cast<uint64_t>(txbuf3);
    //tx_ring2->read.cmd_type_len = 0x2B30003F;
    //tx_ring2->read.olinfo_status = 0x00168000;

    std::atomic_thread_fence(std::memory_order_seq_cst);
	  
    tx_tail2 ++;
    //KPRINTF("0x%llX 0x%X 0x%X\n", tx_ring2->read.buffer_addr, tx_ring2->read.cmd_type_len, tx_ring2->read.olinfo_status);
    //KPRINTF("0x%llX 0x%X 0x%X\n", tx_ring2->wb.rsvd, tx_ring2->wb.nxtseq_seed, tx_ring2->wb.status);
	
    KPRINTF("0x%llX 0x%llX\n", tx_ring2->buffer_address, tx_ring2->word2_raw); 
    
    std::atomic_thread_fence(std::memory_order_seq_cst);
    
    //asm volatile("wbinvd");
    
    ixgbe_dump();
    ixgbe_dump_hw_cntrs();
	
    IXGBE_WRITE_REG(IXGBE_TDT(n), tx_tail2);
    
    while(IXGBE_READ_REG(IXGBE_TDH(0)) == 0) {
	
    }

    ixgbe_dump();
    ixgbe_dump_hw_cntrs();
    
  // auto tlen = 0;
  // auto txbuf = (uint8_t*)malloc(sizeof(uint8_t) * 60);

  // // dest 90:e2:ba:84:d7:38
  // txbuf[tlen++] = 0x90;
  // txbuf[tlen++] = 0xE2;
  // txbuf[tlen++] = 0xBA;
  // txbuf[tlen++] = 0x84;
  // txbuf[tlen++] = 0xD7;
  // txbuf[tlen++] = 0x38;

  // // src
  // txbuf[tlen++] = 0x90;
  // txbuf[tlen++] = 0xE2;
  // txbuf[tlen++] = 0xBA;
  // txbuf[tlen++] = 0x82;
  // txbuf[tlen++] = 0x33;
  // txbuf[tlen++] = 0x24;

  // // eth type
  // txbuf[tlen++] = 0x08;
  // txbuf[tlen++] = 0x00;

  // // payload
  // txbuf[tlen++] = 0xDE;
  // txbuf[tlen++] = 0xAD;
  // txbuf[tlen++] = 0xBE;
  // txbuf[tlen++] = 0xFF;

  // txbuf[tlen++] = 0xDE;
  // txbuf[tlen++] = 0xAD;
  // txbuf[tlen++] = 0xBE;
  // txbuf[tlen++] = 0xFF;

  // for (auto i = tlen; i < 60; i++) {
  //   txbuf[i] = 0x22;
  // }

  // ebbrt::kprintf("%s\n", __PRETTY_FUNCTION__);
  // for (auto i = 0; i < 60; i++) {
  //   ebbrt::kprintf("%02X ", txbuf[i]);
  // }
  // ebbrt::kprintf("\n\n");

  // auto txphys = reinterpret_cast<uint64_t>(txbuf);
  // auto tail = ixgq->tx_tail;
  // // update buffer address for descriptor
  // ixgq->tx_ring[tail].buffer_address = txphys;
  // ixgq->tx_ring[tail].length = 60;
  // ixgq->tx_ring[tail].eop = 1;  // indicate end of packet
  // ixgq->tx_ring[tail].rs =
  //     1;  // so that hw will modify dd bit after packet transmitted

  // ebbrt::kprintf("taddr - %p %p len:%d\n", txbuf,
  //                ixgq->tx_ring[tail].buffer_address,
  //                ixgq->tx_ring[tail].length);

  // // bump tx_tail
  // ixgq->tx_tail = (tail + 1) % ixgq->tx_size;
  // WriteTdt(n, ixgq->tx_tail);  // indicates position beyond last descriptor hw
  // // can process

  // auto head = ixgq->tx_head;
  // // std::atomic_thread_fence(std::memory_order_seq_cst);
  // ebbrt::kprintf("dd = %d\n", ixgq->tx_ring[head].dd);
  // while (ixgq->tx_ring[head].dd == 0) {
  // }  // hw will transmit packet pointed by head, after tranmission, dd bit is
  // // set and head is incremented, finish processing when head == tail

  // // header should be incremented by hw, need to update tx_head
  // // ebbrt::kprintf("header = %d, %d\n", head, ReadTdh(n));
  // ixgq->tx_head = (head + 1) % ixgq->tx_size;

  // ebbrt::kprintf("TX dma complete - tx_head = %d tx_tail = %d\n", ixgq->tx_head,
  //                ixgq->tx_tail);
}

/*
 * Note: if packet len is hardcoded to be > 60, even
 * if the data sent is less than 60 bytes
 */
uint32_t ebbrt::IxgbeDriver::GetRxBuf(uint32_t* len, uint64_t* bAddr) {
  rdesc_legacy_t tmp;
  tmp = ixgq->rx_ring[ixgq->rx_head];

  std::atomic_thread_fence(std::memory_order_release);

  // if got new packet
  if (tmp.dd && tmp.eop) {

    // set len and address
    *len = tmp.length;
    *bAddr = tmp.buffer_address;

    // reset descriptor
    ixgq->rx_ring[ixgq->rx_head].raw[0] = 0;
    ixgq->rx_ring[ixgq->rx_head].raw[1] = 0;
    ebbrt::kprintf("reset descriptor: %d %ld %ld\n", ixgq->rx_head,
                   ixgq->rx_ring[ixgq->rx_head].raw[0],
                   ixgq->rx_ring[ixgq->rx_head].raw[1]);

    // bump head ptr
    ixgq->rx_head = (ixgq->rx_head + 1) % ixgq->rx_size;
    ebbrt::kprintf("BUMP NEW head ptr: %d, HW head ptr (auto increment): %d\n",
                   ixgq->rx_head, ReadRdh_1(0));

    return 0;
  }
  return 1;
}

void ebbrt::IxgbeDriver::ProcessPacket(uint32_t n) {
  uint32_t len;
  uint64_t bAddr;
  auto count = 0;

  // get address of buffer with data
  while (GetRxBuf(&len, &bAddr) == 0) {
    ebbrt::kprintf("%s: len=%d, bAddr=%p\n", __FUNCTION__, len, bAddr);

    // dump eth packet info
    auto p1 = reinterpret_cast<uint8_t*>(bAddr);
    for (uint32_t i = 0; i < len; i++) {
      ebbrt::kprintf("0x%02X ", *p1);
      p1++;
    }
    ebbrt::kprintf("\n");

    // done with buffer addr above, now to reuse it
    auto tail = ixgq->rx_tail;
    ixgq->rx_ring[tail].buffer_address = bAddr;

    // bump tail ptr
    ixgq->rx_tail = (tail + 1) % ixgq->rx_size;

    count++;
  }

  if (count > 0) {
    ebbrt::kprintf("NEW head=%d tail=%d\n", ixgq->rx_head, ixgq->rx_tail);
    // update reg
    WriteRdt_1(n, ixgq->rx_tail);
  }
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
  assert(!(reg & 0x20));  // make sure GPIE.OCD is cleared

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
  ebbrt::kprintf("%s 0x%X\n", __FUNCTION__, m);
  bar0_.Write32(0x01000 + 0x40 * n, m);
}
void ebbrt::IxgbeDriver::WriteRdbal_2(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0D000 + 0x40 * n, m);
}

// 8.2.3.8.2 Receive Descriptor Base Address High — RDBAH[n] (0x01004 + 0x40*n,
// n=0...63 and 0x0D004 + 0x40*(n-64), n=64...127; RW)
void ebbrt::IxgbeDriver::WriteRdbah_1(uint32_t n, uint32_t m) {
  ebbrt::kprintf("%s 0x%X\n", __FUNCTION__, m);
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

// 8.2.3.9.9 Transmit Descriptor Tail — TDT[n] (0x06018+0x40*n, n=0...127; RW)
void ebbrt::IxgbeDriver::WriteTdt(uint32_t n, uint32_t m) {
  bar0_.Write32(0x06018 + 0x40 * n, m);
}

// 8.2.3.8.3 Receive Descriptor Length — RDLEN[n] (0x01008 + 0x40*n, n=0...63
// and 0x0D008 + 0x40*(n-64), n=64...127; RW)
void ebbrt::IxgbeDriver::WriteRdlen_1(uint32_t n, uint32_t m) {
  ebbrt::kprintf("%s %d\n", __FUNCTION__, m);
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
  reg = reg & ~(0x3F);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
}
void ebbrt::IxgbeDriver::WriteIvarAllocval0(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
}
void ebbrt::IxgbeDriver::WriteIvarAlloc1(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  reg = reg & ~(0x3F << 8);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
}
void ebbrt::IxgbeDriver::WriteIvarAllocval1(uint32_t n, uint32_t m) {
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
  ebbrt::kprintf("%s \n", __PRETTY_FUNCTION__);

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

uint32_t ebbrt::IxgbeDriver::IXGBE_READ_REG(uint32_t reg) {
  return bar0_.Read32(reg);
}

void ebbrt::IxgbeDriver::IXGBE_WRITE_REG(uint32_t reg, uint32_t value) {
    KPRINTF("WRITE -> VAL:0x%X REG:0x%X\n", value, reg);
    bar0_.Write32(reg, value);
}

void ebbrt::IxgbeDriver::ixgbe_get_invariants_82599() {
    ebbrt::kprintf("%s \n", __FUNCTION__);
    mac->mcft_size = IXGBE_82599_MC_TBL_SIZE;
    mac->vft_size = IXGBE_82599_VFT_TBL_SIZE;
    mac->num_rar_entries = IXGBE_82599_RAR_ENTRIES;
    mac->max_rx_queues = IXGBE_82599_MAX_RX_QUEUES;
    mac->max_tx_queues = IXGBE_82599_MAX_TX_QUEUES;
    mac->max_msix_vectors = 64;
}

void ebbrt::IxgbeDriver::ixgbe_init_eeprom_params_generic() {
    u32 eec;
    u16 eeprom_size;
  
    // ixgbe_init_eeprom_params_generic
    if (eeprom->type == ixgbe_eeprom_uninitialized) {
	KPRINTF(" ixgbe_init_eeprom_params_generic a\n");
	
	eeprom->type = ixgbe_eeprom_none;
	/* Set default semaphore delay to 10ms which is a well
	 * tested value */
	eeprom->semaphore_delay = 10;
	/* Clear EEPROM page size, it will be initialized as needed */
	eeprom->word_page_size = 0;
	
	eec = IXGBE_READ_REG(IXGBE_EEC);
	if (eec & IXGBE_EEC_PRES) {
	    KPRINTF(" ixgbe_init_eeprom_params_generic b\n");
	    eeprom->type = ixgbe_eeprom_spi;
	    
	    /*
	     * SPI EEPROM is assumed here.  This code would need to
	     * change if a future EEPROM is not SPI.
	     */
	    eeprom_size = (u16)((eec & IXGBE_EEC_SIZE) >>
				IXGBE_EEC_SIZE_SHIFT);
	    
	    eeprom->word_size = 1 << (eeprom_size +
				      IXGBE_EEPROM_WORD_SIZE_SHIFT);
	}

	if (eec & IXGBE_EEC_ADDR_SIZE) {
	    eeprom->address_bits = 16;
	    KPRINTF(" ixgbe_init_eeprom_params_generic c\n");
	}
	else {
	    eeprom->address_bits = 8;
	    KPRINTF(" ixgbe_init_eeprom_params_generic d\n");
	}

	KPRINTF("Eeprom params: type = %d, size = %d, address bits: "
	       "%d\n", eeprom->type, eeprom->word_size,
	       eeprom->address_bits);
    }
}

bool ebbrt::IxgbeDriver::ixgbe_mng_enabled() {
    u32 fwsm, manc, factps;
    
    fwsm = IXGBE_READ_REG(IXGBE_FWSM);
    if ((fwsm & IXGBE_FWSM_MODE_MASK) != IXGBE_FWSM_FW_MODE_PT)
	return false;
    
    manc = IXGBE_READ_REG(IXGBE_MANC);
    if (!(manc & IXGBE_MANC_RCV_TCO_EN))
	return false;
    
    factps = IXGBE_READ_REG(IXGBE_FACTPS);
    if (factps & IXGBE_FACTPS_MNGCG)
	return false;
    
    return true;
}

void ebbrt::IxgbeDriver::ixgbe_clear_tx_pending() {
    KPRINTF("%s\n", __FUNCTION__);
}

s32 ebbrt::IxgbeDriver::ixgbe_disable_pcie_master() {
    s32 status = 0;
    
    KPRINTF("%s\n", __FUNCTION__);

    /* Always set this bit to ensure any future transactions are blocked */
    IXGBE_WRITE_REG(IXGBE_CTRL, IXGBE_CTRL_GIO_DIS);
    
    /* Exit if master requests are blocked */
    if (!(IXGBE_READ_REG(IXGBE_STATUS) & IXGBE_STATUS_GIO)) {
	return status;
    }

    return -1;    
}

s32 ebbrt::IxgbeDriver::ixgbe_stop_adapter_generic() {
    u32 reg_val;
    u16 i;

    KPRINTF("%s\n", __FUNCTION__);
    
    /*
     * Set the adapter_stopped flag so other driver functions stop touching
     * the hardware
     */
    hw->adapter_stopped = true;
    
    /* Disable the receive unit */
    IXGBE_WRITE_REG(IXGBE_RXCTRL, 0);
    
    /* Clear interrupt mask to stop interrupts from being generated */
    IXGBE_WRITE_REG(IXGBE_EIMC, IXGBE_IRQ_CLEAR_MASK);
    
    /* Clear any pending interrupts, flush previous writes */
    IXGBE_READ_REG(IXGBE_EICR);
    
    /* Disable the transmit unit.  Each queue must be disabled. */
    for (i = 0; i < mac->max_tx_queues; i++)
	IXGBE_WRITE_REG(IXGBE_TXDCTL(i), IXGBE_TXDCTL_SWFLSH);

    /* Disable the receive unit by stopping each queue */
    for (i = 0; i < mac->max_rx_queues; i++) {
	reg_val = IXGBE_READ_REG(IXGBE_RXDCTL(i));
	reg_val &= ~IXGBE_RXDCTL_ENABLE;
	reg_val |= IXGBE_RXDCTL_SWFLSH;
	IXGBE_WRITE_REG(IXGBE_RXDCTL(i), reg_val);
    }

    /* flush all queues disables */
    IXGBE_WRITE_FLUSH();
    udelay(2000);

    /*
     * Prevent the PCI-E bus from from hanging by disabling PCI-E master
     * access and verify no pending requests
     */
    return ixgbe_disable_pcie_master();
}

s32 ebbrt::IxgbeDriver::ixgbe_identify_phy_generic() {
     s32 status = IXGBE_ERR_PHY_ADDR_INVALID;
     static int hc = 0;
     
     if(hc == 0){
	 hc ++;
	 phy->type = ixgbe_phy_unknown; 
	 phy->mdio.prtad = 0x0; 
	 phy->mdio.mmds = 0x0;
	 status = 0;
     }
     else if(hc == 1){
	 hc ++;
	 phy->type = ixgbe_phy_sfp_passive_unknown;
	 phy->mdio.prtad = 0x0; 
	 phy->mdio.mmds = 0x0;
	 status = 0;
     }
     
     KPRINTF("%s 0x%X 0x%X 0x%X\n", __FUNCTION__, phy->type, phy->mdio.prtad, phy->mdio.mmds);

     return status;
}

s32 ebbrt::IxgbeDriver::ixgbe_identify_phy_82599() {
    s32 status = IXGBE_ERR_PHY_ADDR_INVALID;
    
    KPRINTF("%s\n", __FUNCTION__);
    
    /* Detect PHY if not unknown - returns success if already detected. */
    status = ixgbe_identify_phy_generic();

    /* Set PHY type none if no PHY detected */
    if (phy->type == ixgbe_phy_unknown) {
	phy->type = ixgbe_phy_none;
	status = 0;
    }

    /* Return error if SFP module has been detected but is not supported */
    if (phy->type == ixgbe_phy_sfp_unsupported)
	status = IXGBE_ERR_SFP_NOT_SUPPORTED;

    return status;
}

s32 ebbrt::IxgbeDriver::ixgbe_init_phy_ops_82599() {
    s32 ret_val = 0;

    KPRINTF("%s\n", __FUNCTION__);

    /* Identify the PHY or SFP module */
    ret_val = ixgbe_identify_phy_82599();

    /* Setup function pointers based on detected SFP module and speeds */
    //ixgbe_init_mac_link_ops_82599();

    return ret_val;
}

s32 ebbrt::IxgbeDriver::ixgbe_reset_phy_generic() {
    s32 status = 0;
    
    KPRINTF("%s\n", __FUNCTION__);

    if (phy->type == ixgbe_phy_unknown)
	status = ixgbe_identify_phy_generic();

    if (status != 0 || phy->type == ixgbe_phy_none) {	   
	goto out;
    }
    
out:
    return status;
}

s32 ebbrt::IxgbeDriver::ixgbe_check_mac_link_generic(ixgbe_link_speed *speed,
						     bool *link_up, bool link_up_wait_to_complete) {
    
    u32 links_reg, links_orig;
    u32 i;
    
    KPRINTF("%s\n", __FUNCTION__);
    
    /* clear the old state */
    links_orig = IXGBE_READ_REG(IXGBE_LINKS);
    
    links_reg = IXGBE_READ_REG(IXGBE_LINKS);

    KPRINTF("%s links_orig:0x%X links_reg:0x%X\n", __FUNCTION__, links_orig, links_reg);
    
    if (link_up_wait_to_complete) {
	for (i = 0; i < IXGBE_LINK_UP_TIME; i++) {
	    if (links_reg & IXGBE_LINKS_UP) {
		*link_up = true;
		break;
	    } else {
		*link_up = false;
	    }
	    mdelay(100);
	    links_reg = IXGBE_READ_REG(IXGBE_LINKS);
	}
    } else {
	if (links_reg & IXGBE_LINKS_UP)
	    *link_up = true;
	else
	    *link_up = false;
    }
    
    if ((links_reg & IXGBE_LINKS_SPEED_82599) ==
	IXGBE_LINKS_SPEED_10G_82599)
	*speed = IXGBE_LINK_SPEED_10GB_FULL;
    else if ((links_reg & IXGBE_LINKS_SPEED_82599) ==
	     IXGBE_LINKS_SPEED_1G_82599)
	*speed = IXGBE_LINK_SPEED_1GB_FULL;
    else if ((links_reg & IXGBE_LINKS_SPEED_82599) ==
	     IXGBE_LINKS_SPEED_100_82599)
	*speed = IXGBE_LINK_SPEED_100_FULL;
    else
	*speed = IXGBE_LINK_SPEED_UNKNOWN;
    
    KPRINTF("%s link_up:%d speed:0x%X\n", __FUNCTION__, *link_up, *speed);
    
    if(*link_up) return 1;
    else return 0;
}

s32 ebbrt::IxgbeDriver::ixgbe_get_mac_addr_generic(u8 *mac_addr) {
    u32 rar_high;
    u32 rar_low;
    u16 i;

    KPRINTF("%s\n", __FUNCTION__);

    rar_high = IXGBE_READ_REG(IXGBE_RAH(0));
    rar_low = IXGBE_READ_REG(IXGBE_RAL(0));
    
    for (i = 0; i < 4; i++) {
	mac_addr[i] = (u8)(rar_low >> (i*8));
	//KPRINTF("%02X ", mac_addr[i]);
    }
    
    for (i = 0; i < 2; i++) {
	mac_addr[i+4] = (u8)(rar_high >> (i*8));
	//KPRINTF("%02X ", mac_addr[i+4]);
    }
    
    return 0;
}

s32 ebbrt::IxgbeDriver::ixgbe_init_uta_tables_generic() {
    int i;
    
    KPRINTF("%s\n", __FUNCTION__);
    
    for (i = 0; i < 128; i++)
	IXGBE_WRITE_REG(IXGBE_UTA(i), 0);
    
    return 0;
}

s32 ebbrt::IxgbeDriver::ixgbe_init_rx_addrs_generic() {
    u32 i;
    u32 rar_entries = mac->num_rar_entries;

    KPRINTF("%s\n", __FUNCTION__);
    
    /*
     * If the current mac address is valid, assume it is a software override
     * to the permanent address.
     * Otherwise, use the permanent address from the eeprom.
     */
    ixgbe_get_mac_addr_generic(mac->addr);
    KPRINTF(" Keeping Current RAR0 Addr = %02X:%02X:%02X:%02X:%02X:%02X\n", mac->addr[0], mac->addr[1], mac->addr[2], mac->addr[3], mac->addr[4], mac->addr[5]);
    
    hw->addr_ctrl.overflow_promisc = 0;
    
    hw->addr_ctrl.rar_used_count = 1;
    
    /* Zero out the other receive addresses. */
    KPRINTF("Clearing RAR[1-%d]\n", rar_entries - 1);
    for (i = 1; i < rar_entries; i++) {
	IXGBE_WRITE_REG(IXGBE_RAL(i), 0);
	IXGBE_WRITE_REG(IXGBE_RAH(i), 0);
    }
    
    /* Clear the MTA */
    hw->addr_ctrl.mta_in_use = 0;
    IXGBE_WRITE_REG(IXGBE_MCSTCTRL, mac->mc_filter_type);
    
    KPRINTF(" Clearing MTA\n");
    for (i = 0; i < mac->mcft_size; i++)
	IXGBE_WRITE_REG(IXGBE_MTA(i), 0);
    
    ixgbe_init_uta_tables_generic();
    
    return 0;
}

void ebbrt::IxgbeDriver::ixgbe_reset_hw_82599() {
    ixgbe_link_speed link_speed;
    s32 status;
    u32 ctrl, i, autoc2;
    u32 curr_lms;
    bool link_up = false;    

    KPRINTF("%s\n", __FUNCTION__);

    status = ixgbe_stop_adapter_generic();
    if(status != 0) {
	ebbrt::kabort("ixgbe_stop_adapter_generic error\n");
    }

    /* flush pending Tx transactions */
    ixgbe_clear_tx_pending();

    /* PHY ops must be identified and initialized prior to reset */
    
    /* Identify PHY and related function pointers */
    status = ixgbe_init_phy_ops_82599();
    if (status == IXGBE_ERR_SFP_NOT_SUPPORTED) {
        ebbrt::kabort("ixgbe_init_phy_ops_82599 error\n");
    }
    
    /* Reset PHY */
    ixgbe_reset_phy_generic();
    
    /*
     * Issue global reset to the MAC. Needs to be SW reset if link is up.
     * If link reset is used when link is up, it might reset the PHY when
     * mng is using it.  If link is down or the flag to force full link
     * reset is set, then perform link reset.
     */
    ctrl = IXGBE_CTRL_LNK_RST;
    if (!hw->force_full_reset) {
        ixgbe_check_mac_link_generic(&link_speed, &link_up, false);
	
	if (link_up) {
	    ctrl = IXGBE_CTRL_RST;
	}
    }

    ctrl |= IXGBE_READ_REG(IXGBE_CTRL);
    IXGBE_WRITE_REG(IXGBE_CTRL, ctrl);
    IXGBE_WRITE_FLUSH();
 
    /* Poll for reset bit to self-clear indicating reset is complete */
    for (i = 0; i < 10; i++) {
	udelay(1);
	ctrl = IXGBE_READ_REG(IXGBE_CTRL);
	if (!(ctrl & IXGBE_CTRL_RST_MASK))
	    break;
    }

    if (ctrl & IXGBE_CTRL_RST_MASK) {
	status = IXGBE_ERR_RESET_FAILED;
    }
    
    mdelay(50);
    
    mac->cached_autoc = IXGBE_READ_REG(IXGBE_AUTOC);
    autoc2 = IXGBE_READ_REG(IXGBE_AUTOC2);
    
    /* Enable link if disabled in NVM */
    if (autoc2 & IXGBE_AUTOC2_LINK_DISABLE_MASK) {
	autoc2 &= ~IXGBE_AUTOC2_LINK_DISABLE_MASK;
	IXGBE_WRITE_REG(IXGBE_AUTOC2, autoc2);
	IXGBE_WRITE_FLUSH();
    }

    if (mac->orig_link_settings_stored == false) {
	mac->orig_autoc = mac->cached_autoc;
        mac->orig_autoc2 = autoc2;
        mac->orig_link_settings_stored = true;
	KPRINTF("%s d\n", __FUNCTION__);
    }

    ixgbe_get_mac_addr_generic(mac->perm_addr);
    
    /*
     * Store MAC address from RAR0, clear receive address registers, and
     * clear the multicast table.  Also reset num_rar_entries to 128,
     * since we modify this value when programming the SAN MAC address.
     */
    mac->num_rar_entries = 128;
    ixgbe_init_rx_addrs_generic();
}

enum ixgbe_bus_width ixgbe_convert_bus_width(u16 link_status)
{

    KPRINTF("%s\n", __FUNCTION__);

	switch (link_status & IXGBE_PCI_LINK_WIDTH) {
	case IXGBE_PCI_LINK_WIDTH_1:
		return ixgbe_bus_width_pcie_x1;
	case IXGBE_PCI_LINK_WIDTH_2:
		return ixgbe_bus_width_pcie_x2;
	case IXGBE_PCI_LINK_WIDTH_4:
		return ixgbe_bus_width_pcie_x4;
	case IXGBE_PCI_LINK_WIDTH_8:
		return ixgbe_bus_width_pcie_x8;
	default:
		return ixgbe_bus_width_unknown;
	}
}

enum ixgbe_bus_speed ixgbe_convert_bus_speed(u16 link_status)
{
    
    KPRINTF("%s\n", __FUNCTION__);

	switch (link_status & IXGBE_PCI_LINK_SPEED) {
	case IXGBE_PCI_LINK_SPEED_2500:
		return ixgbe_bus_speed_2500;
	case IXGBE_PCI_LINK_SPEED_5000:
		return ixgbe_bus_speed_5000;
	case IXGBE_PCI_LINK_SPEED_8000:
		return ixgbe_bus_speed_8000;
	default:
		return ixgbe_bus_speed_unknown;
	}
}

void ebbrt::IxgbeDriver::ixgbe_set_lan_id_multi_port_pcie()
{
	u32 reg;
	
	KPRINTF("%s\n", __FUNCTION__);

	reg = IXGBE_READ_REG(IXGBE_STATUS);
        hw->bus.func = (reg & IXGBE_STATUS_LAN_ID) >> IXGBE_STATUS_LAN_ID_SHIFT;
        hw->bus.lan_id = hw->bus.func;

	/* check for a port swap */
	reg = IXGBE_READ_REG(IXGBE_FACTPS);
	if (reg & IXGBE_FACTPS_LFS)
	    hw->bus.func ^= 0x1;
}

int ebbrt::IxgbeDriver::ixgbe_enumerate_functions() {
    KPRINTF("%s \n", __FUNCTION__);
    return 2;
}

void ebbrt::IxgbeDriver::ixgbe_get_bus_info_generic() {
    u16 link_status;
    KPRINTF("%s\n", __FUNCTION__);
    
    hw->bus.type = ixgbe_bus_type_pci_express;
    
    /* Get the negotiated link width and speed from PCI config space */
    link_status = dev_.GetLinkStatus();

    KPRINTF("link_status = 0x%X\n", link_status);

    hw->bus.width = ixgbe_convert_bus_width(link_status);
    hw->bus.speed = ixgbe_convert_bus_speed(link_status);
    
    ixgbe_set_lan_id_multi_port_pcie();
}

s32 ebbrt::IxgbeDriver::ixgbe_read_eerd_buffer_generic(u16 offset,
						       u16 words, u16 *data) {
    u32 eerd;
    s32 status = 0;
    u32 i;
    
    KPRINTF("%s\n", __FUNCTION__);
    
    for (i = 0; i < words; i++) {
	eerd = ((offset + i) << IXGBE_EEPROM_RW_ADDR_SHIFT) |
	    IXGBE_EEPROM_RW_REG_START;
	
	IXGBE_WRITE_REG(IXGBE_EERD, eerd);
	
	//status = ixgbe_poll_eerd_eewr_done(hw, IXGBE_NVM_POLL_READ);
	while((IXGBE_READ_REG(IXGBE_EERD) & IXGBE_EEPROM_RW_REG_DONE) == 0) {
	}
	
	if (status == 0) {
	    data[i] = (IXGBE_READ_REG(IXGBE_EERD) >>
		       IXGBE_EEPROM_RW_REG_DATA);
	}
    }
    
    return status;
}

s32 ebbrt::IxgbeDriver::ixgbe_read_pba_string_generic(u8 *pba_num,
						      u32 pba_num_size) {
    s32 ret_val;
    u16 data;
    u16 pba_ptr;
    u16 offset;
    u16 length;
    
    KPRINTF("%s\n", __FUNCTION__);
    
    ret_val = ixgbe_read_eerd_buffer_generic(IXGBE_PBANUM0_PTR, 1, &data);
    ret_val = ixgbe_read_eerd_buffer_generic(IXGBE_PBANUM1_PTR, 1, &pba_ptr);

    /*
     * if data is not ptr guard the PBA must be in legacy format which
     * means pba_ptr is actually our second data word for the PBA number
     * and we can decode it into an ascii string
     */
    if (data != IXGBE_PBANUM_PTR_GUARD) {	
	/* we will need 11 characters to store the PBA */
	if (pba_num_size < 11) {
	    return IXGBE_ERR_NO_SPACE;
	}
	
	/* extract hex string from data and pba_ptr */
	pba_num[0] = (data >> 12) & 0xF;
	pba_num[1] = (data >> 8) & 0xF;
	pba_num[2] = (data >> 4) & 0xF;
	pba_num[3] = data & 0xF;
	pba_num[4] = (pba_ptr >> 12) & 0xF;
	pba_num[5] = (pba_ptr >> 8) & 0xF;
	pba_num[6] = '-';
	pba_num[7] = 0;
	pba_num[8] = (pba_ptr >> 4) & 0xF;
	pba_num[9] = pba_ptr & 0xF;
	
	/* put a null character on the end of our string */
	pba_num[10] = '\0';
	
	/* switch all the data but the '-' to hex char */
	for (offset = 0; offset < 10; offset++) {
	    if (pba_num[offset] < 0xA)
		pba_num[offset] += '0';
	    else if (pba_num[offset] < 0x10)
		pba_num[offset] += 'A' - 0xA;
	}
	
	return 0;
    }
    return 0;
}

void ebbrt::IxgbeDriver::ixgbe_dump_hw_cntrs() {
    u16 i = 0;

    KPRINTF("%s\n", __FUNCTION__);

    KPRINTF("CRCERRS %X\n", IXGBE_READ_REG(IXGBE_CRCERRS));
	KPRINTF("ILLERRC %X\n", IXGBE_READ_REG(IXGBE_ILLERRC));
	KPRINTF("ERRBC %X\n", IXGBE_READ_REG(IXGBE_ERRBC));
	KPRINTF("MSPDC %X\n", IXGBE_READ_REG(IXGBE_MSPDC));
	for (i = 0; i < 8; i++)
	    KPRINTF("MPC%d %X\n", i, IXGBE_READ_REG(IXGBE_MPC(i)));

	KPRINTF("MLFC %X\n", IXGBE_READ_REG(IXGBE_MLFC));
	KPRINTF("MRFC %X\n", IXGBE_READ_REG(IXGBE_MRFC));
	KPRINTF("RLEC %X\n", IXGBE_READ_REG(IXGBE_RLEC));
	KPRINTF("LXONTXC %X\n", IXGBE_READ_REG(IXGBE_LXONTXC));
	KPRINTF("LXOFFTXC %X\n", IXGBE_READ_REG(IXGBE_LXOFFTXC));
	if (mac->type >= ixgbe_mac_82599EB) {
		KPRINTF("IXGBE_LXONRXCNT %X\n", IXGBE_READ_REG(IXGBE_LXONRXCNT));
		KPRINTF("IXGBE_LXOFFRRXCNT%X\n", IXGBE_READ_REG(IXGBE_LXOFFRXCNT));
	} else {
		KPRINTF("IXGBE_LXONRXC %X\n", IXGBE_READ_REG(IXGBE_LXONRXC));
		KPRINTF("IXGBE_LXOFFRXC %X\n", IXGBE_READ_REG(IXGBE_LXOFFRXC));
	}

	for (i = 0; i < 8; i++) {
	    KPRINTF("IXGBE_PXONTXC %d %X\n", i, IXGBE_READ_REG(IXGBE_PXONTXC(i)));
	    KPRINTF("IXGBE_PXOFFTXC %d %X\n", i, IXGBE_READ_REG(IXGBE_PXOFFTXC(i)));
		if (mac->type >= ixgbe_mac_82599EB) {
		    KPRINTF("IXGBE_PXONRXCNT %d %X\n", i, IXGBE_READ_REG(IXGBE_PXONRXCNT(i)));
		    KPRINTF("IXGBE_PXOFFRXCNT %d %X\n", i, IXGBE_READ_REG(IXGBE_PXOFFRXCNT(i)));
		} else {
		    KPRINTF("IXGBE_PXONRXC %d %X\n", i, IXGBE_READ_REG(IXGBE_PXONRXC(i)));
		    KPRINTF("IXGBE_PXOFFRXC %d %X\n", i, IXGBE_READ_REG(IXGBE_PXOFFRXC(i)));
		}
	}
	if (mac->type >= ixgbe_mac_82599EB)
		for (i = 0; i < 8; i++)
		    KPRINTF("IXGBE_PXON2OFFCNT %d %X\n", i, IXGBE_READ_REG(IXGBE_PXON2OFFCNT(i)));

	KPRINTF("PRC64 %X\n", IXGBE_READ_REG(IXGBE_PRC64));
	KPRINTF("PRC127 %X\n", IXGBE_READ_REG(IXGBE_PRC127));
	KPRINTF("PRC255 %X\n", IXGBE_READ_REG(IXGBE_PRC255));
	KPRINTF("PRC511 %X\n", IXGBE_READ_REG(IXGBE_PRC511));
	KPRINTF("PRC1023 %X\n", IXGBE_READ_REG(IXGBE_PRC1023));
	KPRINTF("PRC1522 %X\n", IXGBE_READ_REG(IXGBE_PRC1522));
	KPRINTF("GPRC %X\n", IXGBE_READ_REG(IXGBE_GPRC));
	KPRINTF("BPRC %X\n", IXGBE_READ_REG(IXGBE_BPRC));
	KPRINTF("MPRC %X\n", IXGBE_READ_REG(IXGBE_MPRC));
	KPRINTF("GPTC %X\n", IXGBE_READ_REG(IXGBE_GPTC));
	KPRINTF("GORCL %X\n", IXGBE_READ_REG(IXGBE_GORCL));
	KPRINTF("GORCH %X\n", IXGBE_READ_REG(IXGBE_GORCH));
	KPRINTF("GOTCL %X\n", IXGBE_READ_REG(IXGBE_GOTCL));
	KPRINTF("GOTCH %X\n", IXGBE_READ_REG(IXGBE_GOTCH));

	KPRINTF("RUC %X\n", IXGBE_READ_REG(IXGBE_RUC));
	KPRINTF("RFC %X\n", IXGBE_READ_REG(IXGBE_RFC));
	KPRINTF("ROC %X\n", IXGBE_READ_REG(IXGBE_ROC));
	KPRINTF("RJC %X\n", IXGBE_READ_REG(IXGBE_RJC));
	KPRINTF("MNGPRC %X\n", IXGBE_READ_REG(IXGBE_MNGPRC));
	KPRINTF("MNGPDC %X\n", IXGBE_READ_REG(IXGBE_MNGPDC));
	KPRINTF("MNGTPC %X\n", IXGBE_READ_REG(IXGBE_MNGPTC));
	KPRINTF("TORL %X\n", IXGBE_READ_REG(IXGBE_TORL));
	KPRINTF("TORH %X\n", IXGBE_READ_REG(IXGBE_TORH));
	KPRINTF("TPR %X\n", IXGBE_READ_REG(IXGBE_TPR));
	KPRINTF("TPT %X\n", IXGBE_READ_REG(IXGBE_TPT));
	KPRINTF("PTC64 %X\n", IXGBE_READ_REG(IXGBE_PTC64));
	KPRINTF("PTC127 %X\n", IXGBE_READ_REG(IXGBE_PTC127));
	KPRINTF("PTC255 %X\n", IXGBE_READ_REG(IXGBE_PTC255));
	KPRINTF("PTC511 %X\n", IXGBE_READ_REG(IXGBE_PTC511));
	KPRINTF("PTC1023 %X\n", IXGBE_READ_REG(IXGBE_PTC1023));
	KPRINTF("PTC1522 %X\n", IXGBE_READ_REG(IXGBE_PTC1522));
	KPRINTF("MPTC %X\n", IXGBE_READ_REG(IXGBE_MPTC));
	KPRINTF("BPTC %X\n", IXGBE_READ_REG(IXGBE_BPTC));
	
	for (i = 0; i < 16; i++) {
	    KPRINTF("%d\n", i);
		KPRINTF("IXGBE_QPRC %X\n", IXGBE_READ_REG(IXGBE_QPRC(i)));
		KPRINTF("IXGBE_QPTC %X\n", IXGBE_READ_REG(IXGBE_QPTC(i)));
		KPRINTF("IXGBE_QBRC_L %X\n", IXGBE_READ_REG(IXGBE_QBRC_L(i)));
		KPRINTF("IXGBE_QBRC_H %X\n", IXGBE_READ_REG(IXGBE_QBRC_H(i)));
		KPRINTF("IXGBE_QBTC_L %X\n", IXGBE_READ_REG(IXGBE_QBTC_L(i)));
		KPRINTF("IXGBE_QBTC_H %X\n", IXGBE_READ_REG(IXGBE_QBTC_H(i)));
		KPRINTF("IXGBE_QPRDC %X\n", IXGBE_READ_REG(IXGBE_QPRDC(i)));
	}
}

void ebbrt::IxgbeDriver::ixgbe_clear_hw_cntrs_generic() {
    u16 i = 0;

    KPRINTF("%s\n", __FUNCTION__);

    IXGBE_READ_REG(IXGBE_CRCERRS);
	IXGBE_READ_REG(IXGBE_ILLERRC);
	IXGBE_READ_REG(IXGBE_ERRBC);
	IXGBE_READ_REG(IXGBE_MSPDC);
	for (i = 0; i < 8; i++)
		IXGBE_READ_REG(IXGBE_MPC(i));

	IXGBE_READ_REG(IXGBE_MLFC);
	IXGBE_READ_REG(IXGBE_MRFC);
	IXGBE_READ_REG(IXGBE_RLEC);
	IXGBE_READ_REG(IXGBE_LXONTXC);
	IXGBE_READ_REG(IXGBE_LXOFFTXC);
	if (mac->type >= ixgbe_mac_82599EB) {
		IXGBE_READ_REG(IXGBE_LXONRXCNT);
		IXGBE_READ_REG(IXGBE_LXOFFRXCNT);
	} else {
		IXGBE_READ_REG(IXGBE_LXONRXC);
		IXGBE_READ_REG(IXGBE_LXOFFRXC);
	}

	for (i = 0; i < 8; i++) {
		IXGBE_READ_REG(IXGBE_PXONTXC(i));
		IXGBE_READ_REG(IXGBE_PXOFFTXC(i));
		if (mac->type >= ixgbe_mac_82599EB) {
			IXGBE_READ_REG(IXGBE_PXONRXCNT(i));
			IXGBE_READ_REG(IXGBE_PXOFFRXCNT(i));
		} else {
			IXGBE_READ_REG(IXGBE_PXONRXC(i));
			IXGBE_READ_REG(IXGBE_PXOFFRXC(i));
		}
	}
	if (mac->type >= ixgbe_mac_82599EB)
		for (i = 0; i < 8; i++)
			IXGBE_READ_REG(IXGBE_PXON2OFFCNT(i));
	IXGBE_READ_REG(IXGBE_PRC64);
	IXGBE_READ_REG(IXGBE_PRC127);
	IXGBE_READ_REG(IXGBE_PRC255);
	IXGBE_READ_REG(IXGBE_PRC511);
	IXGBE_READ_REG(IXGBE_PRC1023);
	IXGBE_READ_REG(IXGBE_PRC1522);
	IXGBE_READ_REG(IXGBE_GPRC);
	IXGBE_READ_REG(IXGBE_BPRC);
	IXGBE_READ_REG(IXGBE_MPRC);
	IXGBE_READ_REG(IXGBE_GPTC);
	IXGBE_READ_REG(IXGBE_GORCL);
	IXGBE_READ_REG(IXGBE_GORCH);
	IXGBE_READ_REG(IXGBE_GOTCL);
	IXGBE_READ_REG(IXGBE_GOTCH);

	IXGBE_READ_REG(IXGBE_RUC);
	IXGBE_READ_REG(IXGBE_RFC);
	IXGBE_READ_REG(IXGBE_ROC);
	IXGBE_READ_REG(IXGBE_RJC);
	IXGBE_READ_REG(IXGBE_MNGPRC);
	IXGBE_READ_REG(IXGBE_MNGPDC);
	IXGBE_READ_REG(IXGBE_MNGPTC);
	IXGBE_READ_REG(IXGBE_TORL);
	IXGBE_READ_REG(IXGBE_TORH);
	IXGBE_READ_REG(IXGBE_TPR);
	IXGBE_READ_REG(IXGBE_TPT);
	IXGBE_READ_REG(IXGBE_PTC64);
	IXGBE_READ_REG(IXGBE_PTC127);
	IXGBE_READ_REG(IXGBE_PTC255);
	IXGBE_READ_REG(IXGBE_PTC511);
	IXGBE_READ_REG(IXGBE_PTC1023);
	IXGBE_READ_REG(IXGBE_PTC1522);
	IXGBE_READ_REG(IXGBE_MPTC);
	IXGBE_READ_REG(IXGBE_BPTC);
	for (i = 0; i < 16; i++) {
		IXGBE_READ_REG(IXGBE_QPRC(i));
		IXGBE_READ_REG(IXGBE_QPTC(i));
		IXGBE_READ_REG(IXGBE_QBRC_L(i));
		IXGBE_READ_REG(IXGBE_QBRC_H(i));
		IXGBE_READ_REG(IXGBE_QBTC_L(i));
		IXGBE_READ_REG(IXGBE_QBTC_H(i));
		IXGBE_READ_REG(IXGBE_QPRDC(i));
	}
}

void ebbrt::IxgbeDriver::ixgbe_clear_vfta_generic() {
    u32 offset;
    
    KPRINTF("%s \n", __FUNCTION__);
    
    for (offset = 0; offset < mac->vft_size; offset++)
	IXGBE_WRITE_REG(IXGBE_VFTA(offset), 0);
    
    for (offset = 0; offset < IXGBE_VLVF_ENTRIES; offset++) {
	IXGBE_WRITE_REG(IXGBE_VLVF(offset), 0);
	IXGBE_WRITE_REG(IXGBE_VLVFB(offset*2), 0);
	IXGBE_WRITE_REG(IXGBE_VLVFB((offset*2)+1), 0);
    }
}

void ebbrt::IxgbeDriver::ixgbe_start_hw_generic() {
    
    u32 ctrl_ext;
    
    KPRINTF("%s\n", __FUNCTION__);
    
    /* Set the media type */
    phy->media_type = ixgbe_media_type_fiber;

    /* Identify the PHY */
    ixgbe_identify_phy_82599();
    
    /* Clear the VLAN filter table */
    ixgbe_clear_vfta_generic();

    /* Clear statistics registers */
    ixgbe_clear_hw_cntrs_generic();
 
    /* Set No Snoop Disable */
    ctrl_ext = IXGBE_READ_REG(IXGBE_CTRL_EXT);
    ctrl_ext |= IXGBE_CTRL_EXT_NS_DIS;
    IXGBE_WRITE_REG(IXGBE_CTRL_EXT, ctrl_ext);
    IXGBE_WRITE_FLUSH();

    /* Clear adapter stopped flag */
    hw->adapter_stopped = false;
}

void ebbrt::IxgbeDriver::ixgbe_start_hw_gen2() {
    u32 i;
    u32 regval;
    
    KPRINTF("%s\n", __FUNCTION__);
    
    /* Clear the rate limiters */
    for (i = 0; i < mac->max_tx_queues; i++) {
	IXGBE_WRITE_REG(IXGBE_RTTDQSEL, i);
	IXGBE_WRITE_REG(IXGBE_RTTBCNRC, 0);
    }
    IXGBE_WRITE_FLUSH();

    /* Disable relaxed ordering */
    for (i = 0; i < mac->max_tx_queues; i++) {
	regval = IXGBE_READ_REG(IXGBE_DCA_TXCTRL_82599(i));
	regval &= ~IXGBE_DCA_TXCTRL_DESC_WRO_EN;
	IXGBE_WRITE_REG(IXGBE_DCA_TXCTRL_82599(i), regval);
    }
    
    for (i = 0; i < mac->max_rx_queues; i++) {
	regval = IXGBE_READ_REG(IXGBE_DCA_RXCTRL(i));
	regval &= ~(IXGBE_DCA_RXCTRL_DATA_WRO_EN |
		    IXGBE_DCA_RXCTRL_HEAD_WRO_EN);
	IXGBE_WRITE_REG(IXGBE_DCA_RXCTRL(i), regval);
    }
}

s32 ebbrt::IxgbeDriver::ixgbe_start_hw_82599() {
    
    KPRINTF("%s\n", __FUNCTION__);
    
    ixgbe_start_hw_generic();

    ixgbe_start_hw_gen2();

    /* We need to run link autotry after the driver loads */
    mac->autotry_restart = true;
    mac->rx_pb_size = IXGBE_82599_RX_PB_SIZE;

}

void ebbrt::IxgbeDriver::ixgbe_disable_tx_laser_multispeed_fiber() {
    
    KPRINTF("%s\n", __FUNCTION__);
    
    u32 esdp_reg = IXGBE_READ_REG(IXGBE_ESDP);
    
    /* Disable tx laser; allow 100us to go dark per spec */
    esdp_reg |= IXGBE_ESDP_SDP3;
    IXGBE_WRITE_REG(IXGBE_ESDP, esdp_reg);
    IXGBE_WRITE_FLUSH();
    udelay(100);
}

void ebbrt::IxgbeDriver::ixgbe_probe() {
    
    int expected_gts;
    u8 part_str[IXGBE_PBANUM_LENGTH];
    
  eeprom = (struct ixgbe_eeprom_info*)malloc(sizeof(struct ixgbe_eeprom_info));
  memset(eeprom, 0, sizeof(struct ixgbe_eeprom_info));
  
  mac = (struct ixgbe_mac_info*) malloc(sizeof(struct ixgbe_mac_info));
  memset(mac, 0, sizeof(struct ixgbe_mac_info));
  
  hw = (struct ixgbe_hw *) malloc(sizeof(struct ixgbe_hw));
  memset(hw, 0, sizeof(struct ixgbe_hw));
  
  phy = (struct ixgbe_phy_info*) malloc (sizeof(struct ixgbe_phy_info));
  memset(phy, 0, sizeof(struct ixgbe_phy_info));

  KPRINTF("%s \n", __PRETTY_FUNCTION__);
  bar0_.Map();  // allocate virtual memory
  
  ebbrt::clock::SleepMilli(200);
  KPRINTF("Sleep 200 ms\n");

  ixgbe_get_invariants_82599();

  ixgbe_init_eeprom_params_generic();
  hw->mng_fw_enabled = ixgbe_mng_enabled();
  KPRINTF("mng_fw_enabled = %d\n", hw->mng_fw_enabled);

  IXGBE_WRITE_REG(IXGBE_WUS, ~0);
  IXGBE_WRITE_REG(IXGBE_WUS+32, ~0);
  
  ixgbe_reset_hw_82599();

  //init_interrupt_scheme()

  ixgbe_get_bus_info_generic();
  expected_gts = ixgbe_enumerate_functions() * 10;
  ixgbe_read_pba_string_generic(part_str, IXGBE_PBANUM_LENGTH);
  mac->type = ixgbe_mac_82599EB;
  KPRINTF("MAC: %d, PHY: %d, PBA No: %s\n",
	  mac->type, phy->type, part_str);
  
  /* reset the hardware with the new settings */
  ixgbe_start_hw_82599();

  /* power down the optics for 82599 SFP+ fiber */
  //ixgbe_disable_tx_laser_multispeed_fiber();
  
}

void ebbrt::IxgbeDriver::ixgbe_setup_all_tx_resources() {
    // TX
    auto sz = align::Up(sizeof(tdesc_legacy_t) * NTXDESCS, 4096); //align up to 4K
    auto order = Fls(sz - 1) - pmem::kPageShift + 1;
    auto page = page_allocator->Alloc(order);
    kbugon(page == Pfn::None(), "ixgbe: page allocation failed");
    auto addr = reinterpret_cast<void*>(page.ToAddr());
    memset(addr, 0, sz);
    
    tx_ring2 = static_cast<tdesc_legacy_t*>(addr); //(tdesc_legacy_t*)malloc(tx_size);
    tx_head2 = 0;
    tx_tail2 = 0;
    tx_size2 = NTXDESCS * sizeof(tdesc_legacy_t);

    auto sz2 = align::Up(sizeof(uint8_t) * 512, 4096);
    auto order2 = Fls(sz2 - 1) - pmem::kPageShift + 1;
    auto page2 = page_allocator->Alloc(order2);
    kbugon(page2 == Pfn::None(), "ixgbe: page allocation failed");
    auto addr2 = reinterpret_cast<void*>(page2.ToAddr());
    memset(addr2, 0, sz2);
    hantxbuf = reinterpret_cast<volatile std::atomic<uint8_t>*>(addr2);
    
    auto tlen = 0;
    // // dest 90:e2:ba:84:d7:38
    hantxbuf[tlen++] = 0xFF;
    hantxbuf[tlen++] = 0xFF;
    hantxbuf[tlen++] = 0xFF;
    hantxbuf[tlen++] = 0xFF;
    hantxbuf[tlen++] = 0xFF;
    hantxbuf[tlen++] = 0xFF;
    
    // src
    hantxbuf[tlen++] = 0x90;
    hantxbuf[tlen++] = 0xE2;
    hantxbuf[tlen++] = 0xBA;
    hantxbuf[tlen++] = 0x82;
    hantxbuf[tlen++] = 0x33;
    hantxbuf[tlen++] = 0x24;
    
    // eth type
    hantxbuf[tlen++] = 0x08;
    hantxbuf[tlen++] = 0x00;
    
    // payload
    hantxbuf[tlen++] = 0xDE;
    hantxbuf[tlen++] = 0xAD;
    hantxbuf[tlen++] = 0xBE;
    hantxbuf[tlen++] = 0xFF;
    
    hantxbuf[tlen++] = 0xDE;
    hantxbuf[tlen++] = 0xAD;
    hantxbuf[tlen++] = 0xBE;
    hantxbuf[tlen++] = 0xFF;
    
    for (auto i = tlen; i < 512; i++) {
	hantxbuf[i] = 0x22;
    }

}

void ebbrt::IxgbeDriver::ixgbe_configure_tx_ring() {
    u32 txdctl = IXGBE_TXDCTL_ENABLE;
    u32 reg_idx = 0;
    
    ebbrt::kprintf("%s \n", __FUNCTION__);
    
    /* disable queue to avoid issues while updating state */
    IXGBE_WRITE_REG(IXGBE_TXDCTL(reg_idx), 0);
    IXGBE_WRITE_FLUSH();

    uint64_t txaddr = reinterpret_cast<uint64_t>(tx_ring2);
    uint32_t txaddrl = txaddr & 0xFFFFFFFF;
    uint32_t txaddrh = txaddr >> 32;
    ebbrt::kprintf("tx_ring2: %p txaddr: %p txaddrl: %p txaddrh: %p\n", tx_ring2, txaddr, txaddrl,
		   txaddrh);

    IXGBE_WRITE_REG(IXGBE_TDBAL(reg_idx), txaddrl);
    IXGBE_WRITE_REG(IXGBE_TDBAH(reg_idx), txaddrh);
    IXGBE_WRITE_REG(IXGBE_TDLEN(reg_idx), tx_size2);
    IXGBE_WRITE_REG(IXGBE_TDH(reg_idx), 0);
    IXGBE_WRITE_REG(IXGBE_TDT(reg_idx), 0);

    /* enable queue */
    IXGBE_WRITE_REG(IXGBE_TXDCTL(reg_idx), txdctl);
 
    /* poll to verify queue is enabled */
    while(!(IXGBE_READ_REG(IXGBE_TXDCTL(reg_idx)) & IXGBE_TXDCTL_ENABLE)) {
	udelay(2000);
    }
}

void ebbrt::IxgbeDriver::ixgbe_configure_tx() {
    u32 dmatxctl;

    ebbrt::kprintf("%s \n", __FUNCTION__);

    dmatxctl = IXGBE_READ_REG(IXGBE_DMATXCTL);
    dmatxctl |= IXGBE_DMATXCTL_TE;
    IXGBE_WRITE_REG(IXGBE_DMATXCTL, dmatxctl);

    ixgbe_configure_tx_ring();
}

void ebbrt::IxgbeDriver::ixgbe_enable_tx_laser_multispeed_fiber()
{
    u32 esdp_reg = IXGBE_READ_REG(IXGBE_ESDP);
    
    ebbrt::kprintf("%s \n", __FUNCTION__);

    /* Enable tx laser; allow 100ms to light up */
    esdp_reg &= ~IXGBE_ESDP_SDP3;
    IXGBE_WRITE_REG(IXGBE_ESDP, esdp_reg);
    IXGBE_WRITE_FLUSH();
    mdelay(100);
}

void ebbrt::IxgbeDriver::ixgbe_get_hw_control() {
    
    u32 ctrl_ext;
    
    ebbrt::kprintf("%s \n", __FUNCTION__);
    
    /* Let firmware know the driver has taken over */
    ctrl_ext = IXGBE_READ_REG(IXGBE_CTRL_EXT);
    IXGBE_WRITE_REG(IXGBE_CTRL_EXT,
		    ctrl_ext | IXGBE_CTRL_EXT_DRV_LOAD);
}

void ebbrt::IxgbeDriver::ixgbe_up_complete() {
    ebbrt::kprintf("%s \n", __FUNCTION__);

    ixgbe_get_hw_control();
    //ixgbe_enable_tx_laser_multispeed_fiber();

    /* clear any pending interrupts, may auto mask */
    IXGBE_READ_REG(IXGBE_EICR);
    
}

void ebbrt::IxgbeDriver::ixgbe_open()
{
    ixgbe_link_speed link_speed;
    bool link_up = false;
    ebbrt::kprintf("%s \n", __FUNCTION__);
    
    ixgbe_setup_all_tx_resources();
    ixgbe_configure_tx();
    ixgbe_up_complete();

    while(ixgbe_check_mac_link_generic(&link_speed, &link_up, false) == 0){
	udelay(2000);
    }
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

  ebbrt::kprintf("%s \n", __PRETTY_FUNCTION__);
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

void ebbrt::IxgbeDriver::SetupQueue(uint32_t i) {
  ebbrt::kprintf("sizeof(rdesc_legacy_t) = %d\n", sizeof(rdesc_legacy_t));
  /*ebbrt::kprintf("sizeof(rdesc_advance_rf_t) = %d\n",
                 sizeof(rdesc_advance_rf_t));
  ebbrt::kprintf("sizeof(rdesc_advance_wbf_t) = %d\n",
  sizeof(rdesc_advance_wbf_t));*/
  ebbrt::kprintf("sizeof(tdesc_legacy_t) = %d\n", sizeof(tdesc_legacy_t));
  /*ebbrt::kprintf("sizeof(tdesc_advance_ctxt_wb_t) = %d\n",
                 sizeof(tdesc_advance_ctxt_wb_t));
  ebbrt::kprintf("sizeof(tdesc_advance_tx_rf_t) = %d\n",
                 sizeof(tdesc_advance_tx_rf_t));
  ebbrt::kprintf("sizeof(tdesc_advance_tx_wbf_t) = %d\n",
  sizeof(tdesc_advance_tx_wbf_t));*/

  // allocate memory for descriptor rings
  ixgq = (e10k_queue_t*)malloc(sizeof(e10k_queue_t));

  // TX
  auto tx_size = sizeof(tdesc_legacy_t) * NTXDESCS;
  ixgq->tx_ring = (tdesc_legacy_t*)malloc(tx_size);
  ixgq->tx_head = 0;
  ixgq->tx_tail = 0;
  ixgq->tx_size = NTXDESCS;

  // RX - allocate a ring of 256 receive legacy descriptors
  auto rx_size = sizeof(rdesc_legacy_t) * NRXDESCS;
  ixgq->rx_ring = (rdesc_legacy_t*)malloc(rx_size);

  // head and tail point to same descriptor
  ixgq->rx_head = 0;
  ixgq->rx_tail = 0;
  ixgq->rx_size = NRXDESCS;

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
  assert((rxaddrl & 0x7F) == 0);

  // update register RDBAL, RDBAH with receive descriptor base address
  WriteRdbal_1(i, rxaddrl);
  WriteRdbah_1(i, rxaddrh);

  // length must also be 128 byte aligned
  assert((rx_size & 0x7F) == 0);
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
  auto rcv_vector =
      event_manager->AllocateVector([this, i]() { ebb_->ReceivePoll(i); });
  dev_.SetMsixEntry(i * 2, rcv_vector, i);  // TODO: fix

  // auto ii = i / 2;
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
  rxbuf = malloc(RXBUFSZ * (NRXDESCS - 1));
  assert(rxbuf != NULL);
  memset(rxbuf, 0, RXBUFSZ * (NRXDESCS - 1));

  /*int* p1 = static_cast<int*>(rxbuf);
  ebbrt::kprintf("p1 = %d\n", *p1);
  p1++;
  ebbrt::kprintf("p1 = %d\n", *p1);*/

  ebbrt::kprintf("Allocated RX buffer: %p\n", rxbuf);

  // add buffer to each descriptor
  for (auto i = 0; i < 256 - 1; i++) {
    auto rxphys =
        reinterpret_cast<uint64_t>(static_cast<char*>(rxbuf) + (i * RXBUFSZ));
    // auto rxphys = (uint64_t)rxbuf + (i * RXBUFSZ);
    auto tail = ixgq->rx_tail;
    ixgq->rx_ring[tail].buffer_address =
        rxphys;  // update buffer address for descriptor

    // ebbrt::kprintf("Descriptor #%d: %p %p\n", tail, rxphys,
    //		   ixgq->rx_ring[tail].buffer_address);

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
  assert((txaddrl & 0x7F) == 0);

  // program base address registers
  WriteTdbal(i, txaddrl);
  WriteTdbah(i, txaddrh);

  // length must also be 128 byte aligned
  assert((tx_size & 0x7F) == 0);
  WriteTdlen(i, tx_size);

  // init head and tail ptr
  WriteTdh(i, 0x0);
  WriteTdt(i, 0x0);

  // enable transmit path
  WriteDmatxctl_te(0x1);

  // transmit queue enable
  WriteTxdctl(i, 0x1 << 25);
  // poll until set, TODO: Timeout
  while (ReadTxdctl_enable(i) == 0)
    ;
  ebbrt::kprintf("TX queue enabled\n");

  // TODO: set up dca txctrl FreeBSD?
  WriteDcaTxctrlTxdescWbro(i, ~(0x1 << 11));  // clear TXdescWBROen
}

// IxgbeDriverRep
uint32_t ebbrt::IxgbeDriverRep::GetRxBuf(uint32_t* len, uint64_t* bAddr) {
  rdesc_legacy_t tmp;
  tmp = ixgq_.rx_ring[ixgq_.rx_head];

  std::atomic_thread_fence(std::memory_order_release);

  // if got new packet
  if (tmp.dd && tmp.eop) {

    // set len and address
    *len = tmp.length;
    *bAddr = tmp.buffer_address;

    // reset descriptor
    ixgq_.rx_ring[ixgq_.rx_head].raw[0] = 0;
    ixgq_.rx_ring[ixgq_.rx_head].raw[1] = 0;
    ebbrt::kprintf("reset descriptor: %d %ld %ld\n", ixgq_.rx_head,
                   ixgq_.rx_ring[ixgq_.rx_head].raw[0],
                   ixgq_.rx_ring[ixgq_.rx_head].raw[1]);

    // bump head ptr
    ixgq_.rx_head = (ixgq_.rx_head + 1) % ixgq_.rx_size;
    ebbrt::kprintf("BUMP NEW head ptr: %d, HW head ptr (auto increment): %d\n",
                   ixgq_.rx_head, ReadRdh_1(0));

    return 0;
  }
  return 1;
}

void ebbrt::IxgbeDriverRep::ReceivePoll(uint32_t n) {
  uint32_t len;
  uint64_t bAddr;
  auto count = 0;

  // get address of buffer with data
  while (GetRxBuf(&len, &bAddr) == 0) {
    ebbrt::kprintf("%s: len=%d, bAddr=%p\n", __FUNCTION__, len, bAddr);

    // dump eth packet info
    auto p1 = reinterpret_cast<uint8_t*>(bAddr);
    for (uint32_t i = 0; i < len; i++) {
      ebbrt::kprintf("0x%02X ", *p1);
      p1++;
    }
    ebbrt::kprintf("\n");

    // done with buffer addr above, now to reuse it
    auto tail = ixgq_.rx_tail;
    ixgq_.rx_ring[tail].buffer_address = bAddr;

    // bump tail ptr
    ixgq_.rx_tail = (tail + 1) % ixgq_.rx_size;

    count++;
  }

  if (count > 0) {
    ebbrt::kprintf("NEW head=%d tail=%d\n", ixgq_.rx_head, ixgq_.rx_tail);
    // update reg
    WriteRdt_1(n, ixgq_.rx_tail);

    // pass it up
    auto rbuf = MakeUniqueIOBuf(0);
    auto tmp = std::make_unique<StaticIOBuf>(
        reinterpret_cast<const uint8_t*>(bAddr),
        static_cast<size_t>(len * sizeof(uint8_t)));
    rbuf->PrependChain(std::move(tmp));
    root_.itf_.Receive(std::move(rbuf));
  }
}

ebbrt::IxgbeDriverRep::IxgbeDriverRep(const IxgbeDriver& root)
    : root_(root), ixgq_(root_.GetQueue()),
      receive_callback_([this]() { ReceivePoll(0); }) {}

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
    ReceivePoll(0);
  }
}
void ebbrt::IxgbeDriverRep::WriteTdt_1(uint32_t n, uint32_t m) {
  root_.bar0_.Write32(0x06018 + 0x40 * n, m);
}
