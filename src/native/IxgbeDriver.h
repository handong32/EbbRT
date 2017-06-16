//          Copyright Boston University SESA Group 2013 - 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#ifndef BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_DRIVER_H_
#define BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_DRIVER_H_

#include "../Align.h"
#include "../MulticoreEbb.h"
#include "../SpinLock.h"
#include "../StaticIOBuf.h"
#include "../UniqueIOBuf.h"
#include "Debug.h"
#include "Fls.h"
#include "Ixgbe.h"
#include "Net.h"
#include "PageAllocator.h"
#include "Pci.h"
#include "Pfn.h"
#include "SlabAllocator.h"

//#define JUMBO_EN
#ifdef JUMBO_EN
#define JUMBO_SZ 9728 //largest size allowed
#endif
#define DCA_EN

#define TX_HEAD_WB

namespace ebbrt {

// Queue
typedef struct {
  rdesc_legacy_t* rx_ring;
  size_t rx_head;
  size_t rx_tail;
  size_t rx_size;

  tdesc_legacy_t* tx_ring;
  uint32_t *tx_head;
  size_t tx_tail;
  size_t tx_last_tail;
  size_t tx_size;
  bool* tx_isctx;

  std::vector<std::unique_ptr<MutIOBuf>> circ_buffer;
} e10k_queue_t;

class IxgbeDriverRep;

class IxgbeDriver : public EthernetDevice {
 public:
  explicit IxgbeDriver(pci::Device& dev);
  
  static void Create(pci::Device& dev);
  static bool Probe(pci::Device& dev) {
    if (dev.GetVendorId() == kIxgbeVendorId &&
        dev.GetDeviceId() == kIxgbeDeviceId && dev.GetFunc() == 0) {
      IxgbeDriver::Create(dev);
      return true;
    }
    return false;
  }

  void Run();
  void Send(std::unique_ptr<IOBuf> buf, PacketInfo pinfo) override;
  void CpuInit() override;
    
  const EthernetAddress& GetMacAddress() override;
  
 protected:
  static const constexpr uint16_t kIxgbeVendorId = 0x8086;
  static const constexpr uint16_t kIxgbeDeviceId = 0x10FB;

  /* FreeBSD:
   * RxDescriptors Valid Range: 64-4096 Default Value: 256 This value is the
   * number of receive descriptors allocated for each RX queue. Increasing this
   * value allows the driver to buffer more incoming packets. Each descriptor
   * is 16 bytes.  A receive buffer is also allocated for each descriptor.
   *
   * Note: with 8 rings and a dual port card, it is possible to bump up
   *	against the system mbuf pool limit, you can tune nmbclusters
   *	to adjust for this.
   */
  static const constexpr uint32_t NTXDESCS = 256;
  static const constexpr uint32_t NRXDESCS = 256;
  //static const constexpr uint32_t NTXDESCS = 4096;
  //static const constexpr uint32_t NRXDESCS = 4096;
  static const constexpr uint32_t RXBUFSZ = 2048;
  //static const constexpr uint32_t RXBUFSZ = 16384;

  class e10Kq {
  public:    
  e10Kq(size_t idx, Nid nid) : rx_head_(0), rx_tail_(0), rx_size_(NRXDESCS),
      tx_tail_(0), tx_last_tail_(0), tx_size_(NTXDESCS), idx_(idx) {
      
      circ_buffer_.reserve(NRXDESCS);
      for(uint32_t k=0; k < NRXDESCS; k ++)
      {
	circ_buffer_.emplace_back(MakeUniqueIOBuf(RXBUFSZ, true));
      }
	
      // RX
      auto sz = align::Up(sizeof(rdesc_legacy_t) * NRXDESCS, 4096);
      auto order = Fls(sz - 1) - pmem::kPageShift + 1;
      auto page = page_allocator->Alloc(order, nid);
      kbugon(page == Pfn::None(), "ixgbe: page allocation failed in %s", __FUNCTION__);
      auto addr = reinterpret_cast<void*>(page.ToAddr());
      memset(addr, 0, sz);
      rx_ring_ = static_cast<rdesc_legacy_t*>(addr);
      
      // TX
      sz = align::Up(sizeof(tdesc_legacy_t) * NTXDESCS, 4096);
      order = Fls(sz - 1) - pmem::kPageShift + 1;
      page = page_allocator->Alloc(order, nid);
      kbugon(page == Pfn::None(), "ixgbe: page allocation failed in %s", __FUNCTION__);
      addr = reinterpret_cast<void*>(page.ToAddr());
      memset(addr, 0, sz);
      tx_ring_ = static_cast<tdesc_legacy_t*>(addr);

      // context
      sz = align::Up(sizeof(bool) * NTXDESCS, 4096);
      order = Fls(sz - 1) - pmem::kPageShift + 1;
      page = page_allocator->Alloc(order, nid);
      kbugon(page == Pfn::None(), "ixgbe: page allocation failed in %s", __FUNCTION__);
      addr = reinterpret_cast<void*>(page.ToAddr());
      memset(addr, 0, sz);
      tx_isctx_ = static_cast<bool*>(addr);

#ifdef TX_HEAD_WB
      // TODO: not sure how much exactly to allocate for head wb addr
      tx_head_ = (uint32_t*) malloc (4 * sizeof(uint32_t));
      memset(tx_head_, 0, 4 * sizeof(uint32_t));
      txhwbaddr_ = reinterpret_cast<uint64_t>(tx_head_);
      // txhwbaddr must be byte aligned
      ebbrt::kbugon((txhwbaddr_ & 0x3) != 0, "txhwbaddr not byte aligned\n");
      kassert((txhwbaddr_ & 0x3) == 0);
#else
      tx_head_ = 0;
#endif
      
      rxaddr_ = reinterpret_cast<uint64_t>(rx_ring_);
      txaddr_ = reinterpret_cast<uint64_t>(tx_ring_);
      rx_size_bytes_ = sizeof(rdesc_legacy_t) * NRXDESCS;
      tx_size_bytes_ = sizeof(tdesc_legacy_t) * NTXDESCS;

      // must be 128 byte aligned
      ebbrt::kbugon((rxaddr_ & 0x7F) != 0, "rx_addr_ not 128 byte aligned\n");
      ebbrt::kbugon((txaddr_ & 0x7F) != 0, "tx_addr_ not 128 byte aligned\n");
      ebbrt::kbugon((rx_size_bytes_ & 0x7F) != 0, "rx_size_bytes_ not 128 byte aligned\n");
      ebbrt::kbugon((tx_size_bytes_ & 0x7F) != 0, "tx_size_bytes_ not 128 byte aligned\n");
      
      //ebbrt::kprintf("%s Core: %d: rx_addr = %p, tx_addr = %p, txhwbaddr = %p, rx_size_bytes_ = %p, tx_size_bytes_ = %p\n", __FUNCTION__, idx_, rxaddr_, txaddr_, txhwbaddr_, rx_size_bytes_, tx_size_bytes_);
    }

    size_t rx_head_;
    size_t rx_tail_;
    size_t rx_size_;
    size_t tx_tail_;
    size_t tx_last_tail_;
    size_t tx_size_;
    size_t idx_;
    size_t rx_size_bytes_;
    size_t tx_size_bytes_;
    uint64_t rxaddr_;
    uint64_t txaddr_;
    uint64_t txhwbaddr_;
    
    std::vector<std::unique_ptr<MutIOBuf>> circ_buffer_;
    
    rdesc_legacy_t* rx_ring_;
    tdesc_legacy_t* tx_ring_;
    bool* tx_isctx_;
#ifdef TX_HEAD_WB
    uint32_t *tx_head_;
#else
    size_t tx_head_;
#endif
  };

  SpinLock lock;
    
 private:
  EbbRef<IxgbeDriverRep> ebb_;
  NetworkManager::Interface& itf_;
  EthernetAddress mac_addr_;
  
  void InitStruct();
  void DeviceInfo();
  void Init();
  void PhyInit();
  void StopDevice();
  void GlobalReset();
  //void SetupQueue(uint32_t i);
  

  bool SwsmSmbiRead();
  void SwsmSmbiClear();

  void SwsmSwesmbiSet();
  bool SwsmSwesmbiRead();
  void SwsmSwesmbiClear();

  uint32_t ReadSwfwSyncSmBits(uint32_t m);
  void WriteSwfwSyncSmBits(uint32_t m);
  void WriteSwfwSyncSmBits2(uint32_t m);

  bool SwfwLockPhy();
  void SwfwUnlockPhy();
  bool SwfwSemAcquire();
  void SwfwSemRelease();

  void WriteRxctrl(uint32_t m) const;
  void WriteDmatxctl(uint32_t m);
  void WriteDmatxctl_te(uint32_t m) const;

  void WriteEimc(uint32_t m);
  void WriteEitr(uint32_t n, uint32_t m) const;

  void WriteTxdctl(uint32_t n, uint32_t m) const;

  void WriteRxdctl_1(uint32_t n, uint32_t m);
  void WriteRxdctl_1_enable(uint32_t n, uint32_t m) const;

  void WriteRxdctl_2(uint32_t n, uint32_t m);
  void WriteCtrl(uint32_t m);
  void WriteCtrlExt(uint32_t m);
  void WriteFcttv(uint32_t n, uint32_t m);
  void WriteFcrtl(uint32_t n, uint32_t m);
  void WriteFcrth(uint32_t n, uint32_t m);
  void WriteFcrtv(uint32_t m);
  void WriteFccfg(uint32_t m);
  void WriteEerd(uint32_t m);

  void WriteCorectl(uint16_t m);

  void WriteAutoc(uint32_t m);

  void WriteEicr(uint32_t m) const;
  void WriteGpie(uint32_t m);
  void ReadGpie();
    
  void WriteEims(uint32_t m);

  void WriteRal(uint32_t n, uint32_t m);
  void WriteRah(uint32_t n, uint32_t m);

  void WriteMta(uint32_t n, uint32_t m);
  void WriteVfta(uint32_t n, uint32_t m);
  void WritePfvlvf(uint32_t n, uint32_t m);
  void WriteMpsar(uint32_t n, uint32_t m);
  void WriteFtqf(uint32_t n, uint32_t m);
  void WriteSaqf(uint32_t n, uint32_t m);
  void WriteDaqf(uint32_t n, uint32_t m);
  void WriteSdpqf(uint32_t n, uint32_t m);

  void WriteFctrl(uint32_t m);
  void WriteFhft_1(uint32_t n, uint32_t m);
  void WriteFhft_2(uint32_t n, uint32_t m);

  void WritePfuta(uint32_t n, uint32_t m);
  void WriteMcstctrl(uint32_t m);

  void WriteRttdqsel(uint32_t m);
  void WriteRttbcnrc(uint32_t m);

  void WriteDcaTxctrlTxdescWbro(uint32_t n, uint32_t m) const;
  void WriteDcaTxctrl(uint32_t n, uint32_t m) const;
  void WriteDcaRxctrl(uint32_t n, uint32_t m) const;
  void WriteDcaRxctrl_1(uint32_t n, uint32_t m);
  void WriteDcaRxctrl_2(uint32_t n, uint32_t m);

  void WriteRdbal_1(uint32_t n, uint32_t m) const;
  void WriteRdbal_2(uint32_t n, uint32_t m) const;

  void WriteRdbah_1(uint32_t n, uint32_t m) const;
  void WriteRdbah_2(uint32_t n, uint32_t m) const;

  void WriteRdlen_1(uint32_t n, uint32_t m) const;
  void WriteRdlen_2(uint32_t n, uint32_t m);

  void WriteSrrctl_1(uint32_t n, uint32_t m) const;
  // void WriteSrrctl_1_bsizepacket(uint32_t n, uint32_t m);
  void WriteSrrctl_1_desctype(uint32_t n, uint32_t m) const;

  void WriteRdt_1(uint32_t n, uint32_t m) const ;
  void WriteRdh_1(uint32_t n, uint32_t m) const ;
  void WriteRdt_2(uint32_t n, uint32_t m);

  void WriteIvarAlloc0(uint32_t n, uint32_t m) const;
  void WriteIvarAllocval0(uint32_t n, uint32_t m) const;
  void WriteIvarAlloc1(uint32_t n, uint32_t m);
  void WriteIvarAllocval1(uint32_t n, uint32_t m);
  void WriteIvarAlloc2(uint32_t n, uint32_t m) const;
  void WriteIvarAllocval2(uint32_t n, uint32_t m) const;
  void WriteIvarAlloc3(uint32_t n, uint32_t m);
  void WriteIvarAllocval3(uint32_t n, uint32_t m);

  void WriteSecrxctrl_Rx_Dis(uint32_t m) const;

  void WriteTdbal(uint32_t n, uint32_t m) const;
  void WriteTdbah(uint32_t n, uint32_t m) const;
  void WriteTdlen(uint32_t n, uint32_t m) const;

  void WriteTdh(uint32_t n, uint32_t m);
  void WriteTdt(uint32_t n, uint32_t m);

  void WriteTdwbal(uint32_t n, uint32_t m) const;
  void WriteTdwbah(uint32_t n, uint32_t m) const;
  
  void WriteHlreg0(uint32_t m);
  void WriteRdrxctl(uint32_t m);
  void WriteEiac(uint32_t m) const;
  void WriteEimsn(uint32_t n, uint32_t m) const;

  void WriteDcaCtrl(uint32_t m);

  void WriteMaxfrs(uint32_t m);
  
  uint8_t ReadRdrxctlDmaidone();

  void ReadEicr();
  void ReadEicrCause();
  bool ReadStatusPcieMes();
  uint8_t ReadStatusLanId();
  void ReadCtrl();
  bool ReadEerdDone();
  uint16_t ReadEerdData();
  uint16_t ReadEeprom(uint16_t offset);
  uint8_t ReadAnlp1();
  uint8_t ReadAutocRestartAn();
  uint8_t ReadEecAutoRd();
  uint32_t ReadEims();

  uint32_t ReadRal(uint32_t n);
  uint16_t ReadRah(uint32_t n);
  uint8_t ReadRahAv(uint32_t n);

  uint8_t ReadRxdctl_1_enable(uint32_t n) const;
  uint8_t ReadSecrxstat_Sr_Rdy() const;

  uint8_t ReadTxdctl_enable(uint32_t n) const;

  uint16_t ReadRdh_1(uint32_t n);
  uint16_t ReadTdh(uint32_t n);

  // statistics
  uint32_t ReadTpr();
  uint32_t ReadGprc();
  bool ReadLinksLinkUp();

  // Process
  void ProcessPacket(uint32_t n);
  uint32_t GetRxBuf(uint32_t* len, uint64_t* bAddr);
  void SendPacket(uint32_t n);

  pci::Device& dev_;
  pci::Bar& bar0_;
  
  struct IxgbeRegs {
    volatile uint32_t kIxgbeCtrl;
    volatile uint32_t kIxgbeCtrlBak;
    volatile uint32_t kIxgbeStatus;
  };

  friend class IxgbeDriverRep;
};  // class IxgbeDriver

class IxgbeDriverRep : public MulticoreEbb<IxgbeDriverRep, IxgbeDriver> {
 public:
  explicit IxgbeDriverRep(const IxgbeDriver& root);
  void Run();
  void ReceivePoll();
  void Send(std::unique_ptr<IOBuf> buf, PacketInfo pinfo);
  void AddContext(uint8_t idx, uint8_t maclen, 
		  uint16_t iplen, uint8_t l4len, enum l4_type l4type);
  void AddTx(const uint8_t *pa, uint64_t len, uint64_t totallen, 
	     bool first, bool last, 
	     uint8_t ctx, bool ip_cksum, bool tcpudp_cksum);
  void SetupMultiQueue(uint32_t i);
  
 private:
  uint16_t ReadRdh_1(uint32_t n);
  void WriteRdt_1(uint32_t n, uint32_t m);
  void WriteTdt_1(uint32_t n, uint32_t m);
  void WriteEimcn(uint32_t n, uint32_t m);
  void ReadEicr();
  void ReadEims();
  void ReclaimTx();
  uint32_t GetRxBuf(uint32_t* len, uint64_t* bAddr);

  const IxgbeDriver& root_;
  //e10k_queue_t& ixgq_;
  //IxgbeDriver::e10Kq& ixgmq_;
  IxgbeDriver::e10Kq ixgmq_;
  SpinLock& lock_;
  EventManager::IdleCallback receive_callback_;

};  // class IxgbeDriverRep

}  // namespace ebbrt

#endif  // BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_DRIVER_H_
