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
#include "Perf.h"
#include "Rapl.h"

// Receive Side Scaling (RSC) enabled
//#define RSC_EN
// Direct Cache Access (DCA) enabled
//#define DCA_ENABLE
// Transmit Header Writeback enabled
//#define TX_HEAD_WB
//#define JUMBO_EN

// Collect Statistics Flag
#define STATS_EN
//#define MAX_DESC


namespace ebbrt {
  
// Per-core receive and transmit queue
typedef struct {
  rdesc_legacy_t* rx_ring;
  uint32_t rx_head;
  uint32_t rx_tail;
  uint32_t rx_size;

  tdesc_legacy_t* tx_ring;
  uint32_t* tx_head;
  uint32_t tx_tail;
  uint32_t tx_last_tail;
  uint32_t tx_size;
  bool* tx_isctx;

  // buffers holding packet data
  std::vector<std::unique_ptr<MutIOBuf>> circ_buffer;
} e10k_queue_t;

class IxgbeDriverRep;

class IxgbeDriver : public EthernetDevice {
 public:
  explicit IxgbeDriver(pci::Device& dev)
      : itf_(network_manager->NewInterface(*this)), dev_(dev),
        bar0_(dev.GetBar(0)) {
    dev_.SetBusMaster(true);

    // set up interrupts, polling won't work after this
    auto msix = dev_.MsixEnable();
    kbugon(!msix, "Ixgbe without msix is unsupported\n");

    // each core gets a queue struct
    ixgmq.resize(Cpu::Count());
  }
  
  static void Create(pci::Device& dev);
  static bool Probe(pci::Device& dev) {
    if (dev.GetVendorId() == kIxgbeVendorId &&
        dev.GetDeviceId() == kIxgbeDeviceId && dev.GetFunc() == 0) {
      IxgbeDriver::Create(dev);
      return true;
    }
    return false;
  }
  
  //void Run();
  void Send(std::unique_ptr<IOBuf> buf, PacketInfo pinfo) override;
  //void SendUdp(std::unique_ptr<IOBuf> buf, uint64_t len) override;
  //void SendTCPUnchained(std::unique_ptr<IOBuf> buf, uint64_t len) override;
  //void SendTCPUnchained(std::unique_ptr<IOBuf> buf, uint64_t len) override;
  
  void Config(std::string s, uint32_t v) override;
  std::string ReadNic() override;
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

#ifdef MAX_DESC
  static const constexpr uint32_t NTXDESCS = 8192;
  static const constexpr uint32_t NRXDESCS = 8192;
#else
  static const constexpr uint32_t NTXDESCS = 512;
  static const constexpr uint32_t NRXDESCS = 512;
  //static const constexpr uint32_t NTXDESCS = 4096;
  //static const constexpr uint32_t NRXDESCS = 4096;
#endif

  // Linux Defaults
  static const constexpr uint32_t RXBUFSZ = 2048;
  //static const constexpr uint32_t RXBUFSZ = 8192;
  static const constexpr uint32_t BSIZEHEADER = 256;

  //static const constexpr uint32_t RXBUFSZ = 4096;
  //static const constexpr uint32_t RXBUFSZ = 8192;
  //static const constexpr uint32_t RXBUFSZ = 16384;

  // 8 bits (3 - 11) in          (ITR_INTERVAL * 2 us)
  //static const constexpr uint8_t ITR_INTERVAL = 32;
  static const constexpr uint8_t ITR_INTERVAL = 8;
  
  // 3 bits only (0 - 7) in      (RSC_DELAY + 1) * 4 us
  static const constexpr uint8_t RSC_DELAY = 7;
  
  // DMA Tx TCP Max Allow Size Requests — DTXMXSZRQ
  //static const constexpr uint16_t MAX_BYTES_NUM_REQ = 0x10;
  static const constexpr uint16_t MAX_BYTES_NUM_REQ = 0xFFF;

  // Class with per core queue data structures
  class e10Kq {
   public:
    e10Kq(uint32_t idx, Nid nid)
        : rx_head_(0), rx_tail_(0), rx_size_(NRXDESCS), tx_tail_(0),
          tx_last_tail_(0), tx_size_(NTXDESCS), idx_(idx), rxflag_(0),
          rsc_used(false), hanc{0} {

      circ_buffer_.reserve(NRXDESCS+1);
      for (uint32_t k = 0; k < NRXDESCS+1; k++) {
        circ_buffer_.emplace_back(MakeUniqueIOBuf(RXBUFSZ, true));
      }

      // rsc_chain_ is a map between receive descriptor number and
      // packet len, need packet len to extract out
      // packet data else code will read redundant
      // zeros if packet len does not use full buffer
      // TODO: should be optimized
      rsc_chain_.reserve(NRXDESCS+1);

      // keep a log of number of idle times
      idle_times_.reserve(NRXDESCS);

      // keep track of context descriptors
      tx_iseop.reserve(NRXDESCS);
      for (uint32_t k = 0; k < NRXDESCS; k++) {
        tx_iseop[k] = false;
      }
      
      // keeps a log of descriptors where eop == 1
      // used to coalesce reclaiming of tx descriptors
      // once the threshold of some limit is hit      
      //send_to_watch.reserve(NRXDESCS);

      // RX ring buffer allocation
      auto sz = align::Up(sizeof(rdesc_legacy_t) * NRXDESCS, 4096);
      auto order = Fls(sz - 1) - pmem::kPageShift + 1;
      auto page = page_allocator->Alloc(order, nid);
      kbugon(page == Pfn::None(), "ixgbe: page allocation failed in %s",
             __FUNCTION__);
      auto addr = reinterpret_cast<void*>(page.ToAddr());
      memset(addr, 0, sz);
      rx_ring_ = static_cast<rdesc_legacy_t*>(addr);

      // TX ring buffer allocation
      sz = align::Up(sizeof(tdesc_legacy_t) * NTXDESCS, 4096);
      order = Fls(sz - 1) - pmem::kPageShift + 1;
      page = page_allocator->Alloc(order, nid);
      kbugon(page == Pfn::None(), "ixgbe: page allocation failed in %s",
             __FUNCTION__);
      addr = reinterpret_cast<void*>(page.ToAddr());
      memset(addr, 0, sz);
      tx_ring_ = static_cast<tdesc_legacy_t*>(addr);

      // TX adv context buffer allocation
      /*sz = align::Up(sizeof(bool) * NTXDESCS, 4096);
      order = Fls(sz - 1) - pmem::kPageShift + 1;
      page = page_allocator->Alloc(order, nid);
      kbugon(page == Pfn::None(), "ixgbe: page allocation failed in %s",
             __FUNCTION__);
      addr = reinterpret_cast<void*>(page.ToAddr());
      memset(addr, 0, sz);
      tx_isctx_ = static_cast<bool*>(addr);*/

#ifdef TX_HEAD_WB
      // TODO: not sure how much exactly to allocate for head wb addr
      tx_head_ = (uint32_t*)malloc(4 * sizeof(uint32_t));
      memset(tx_head_, 0, 4 * sizeof(uint32_t));
      txhwbaddr_ = reinterpret_cast<uint64_t>(tx_head_);
      // txhwbaddr must be byte aligned
      ebbrt::kbugon((txhwbaddr_ & 0x3) != 0, "txhwbaddr not byte aligned\n");
      kassert((txhwbaddr_ & 0x3) == 0);
#else
      tx_head_ = 0;
#endif

      // get starting address, need to write to device registers
      rxaddr_ = reinterpret_cast<uint64_t>(rx_ring_);
      txaddr_ = reinterpret_cast<uint64_t>(tx_ring_);
      rx_size_bytes_ = sizeof(rdesc_legacy_t) * NRXDESCS;
      tx_size_bytes_ = sizeof(tdesc_legacy_t) * NTXDESCS;

      // must be 128 byte aligned
      ebbrt::kbugon((rxaddr_ & 0x7F) != 0, "rx_addr_ not 128 byte aligned\n");
      ebbrt::kbugon((txaddr_ & 0x7F) != 0, "tx_addr_ not 128 byte aligned\n");
      ebbrt::kbugon((rx_size_bytes_ & 0x7F) != 0,
                    "rx_size_bytes_ not 128 byte aligned\n");
      ebbrt::kbugon((tx_size_bytes_ & 0x7F) != 0,
                    "tx_size_bytes_ not 128 byte aligned\n");

      tx_desc_counts.reserve(100);
      rx_desc_counts.reserve(100);
      for(int i=0;i<100;i++) {
	tx_desc_counts.emplace_back(0);
	rx_desc_counts.emplace_back(0);
      }
    }
    
    uint32_t rx_head_;
    uint32_t rx_tail_;
    uint32_t rx_size_;
    uint32_t tx_tail_;
    uint32_t tx_last_tail_;
    uint32_t tx_size_;
    uint32_t idx_;
    uint32_t rx_size_bytes_;
    uint32_t tx_size_bytes_;
    uint64_t rxaddr_;
    uint64_t txaddr_;
    uint64_t txhwbaddr_;
    uint64_t rxflag_;
    uint64_t cleaned_count{0};
      
    std::vector<std::unique_ptr<MutIOBuf>> circ_buffer_;
    std::vector<std::pair<uint32_t, uint32_t>> rsc_chain_;
    std::unordered_map<uint32_t, uint32_t> idle_times_;
    std::vector<std::pair<uint32_t, uint32_t>> send_to_watch;
    std::vector<bool> tx_iseop;
    std::ostringstream str_stats;
    //std::vector<uint32_t> send_to_watch;
      
    rdesc_legacy_t* rx_ring_;
    tdesc_legacy_t* tx_ring_;
    
    //std::vector<bool> tx_isctx;
    //bool* tx_isctx_;
    bool rsc_used;
    int hanc;
#ifdef TX_HEAD_WB
    uint32_t* tx_head_;
#else
    uint32_t tx_head_;
#endif

    // stats
    uint64_t stat_num_recv{0};
    uint64_t stat_num_send{0};
    uint64_t stat_num_rx_bytes{0};
    uint64_t stat_num_tx_bytes{0};
    uint64_t time_us{0};
    uint64_t time_send{0};
    uint64_t time_idle_min{999999};
    uint64_t time_idle_max{0};
    uint64_t total_idle_time{0};
    uint64_t totalInterrupts{0};
    uint64_t totalCycles{0};
    uint64_t totalIns{0};
    uint64_t totalLLCmisses{0};
    uint64_t fireCount{0};
    uint32_t rapl_val{666};
    uint32_t itr_val{8};
    std::vector<uint32_t> tx_desc_counts;
    std::vector<uint32_t> rx_desc_counts;
    double totalNrg{0.0};
    double totalTime{0.0};
    double totalPower{0.0};
    
    bool stat_start{false};
    bool stat_init{false};
    ebbrt::perf::PerfCounter perfCycles;
    ebbrt::perf::PerfCounter perfInst;
    ebbrt::perf::PerfCounter perfLLC_ref;
    ebbrt::perf::PerfCounter perfLLC_miss;
    ebbrt::perf::PerfCounter perfTLB_store_miss;
    ebbrt::perf::PerfCounter perfTLB_load_miss;
    ebbrt::rapl::RaplCounter powerMeter;
    
  };

 private:
  EbbRef<IxgbeDriverRep> ebb_;
  NetworkManager::Interface& itf_;
  EthernetAddress mac_addr_;

  void Init();
  void PhyInit();
  void StopDevice();
  void GlobalReset();
  void SetupMultiQueue(uint32_t i);
  void FinishSetup();

  // device register writing code below
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

  void WriteRxctrl(uint32_t m);
  void WriteDmatxctl(uint32_t m);
  void WriteDmatxctl_te(uint32_t m);

  void WriteEimc(uint32_t m);
  void WriteEitr(uint32_t n, uint32_t m);

  void WriteTxdctl(uint32_t n, uint32_t m);

  void WriteRxdctl_1(uint32_t n, uint32_t m);
  void WriteRxdctl_1_enable(uint32_t n, uint32_t m);

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

  void WriteEicr(uint32_t m);
  void WriteGpie(uint32_t m);
  void WriteEiam(uint32_t n, uint32_t m);

  void WriteEims(uint32_t m);

  void WriteRal(uint32_t n, uint32_t m);
  void WriteRah(uint32_t n, uint32_t m);

  void WriteMta(uint32_t n, uint32_t m);
  void WriteVfta(uint32_t n, uint32_t m);
  void WritePfvlvf(uint32_t n, uint32_t m);
  void WritePfvlvfb(uint32_t n, uint32_t m);
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

  void WriteDcaTxctrlTxdescWbro(uint32_t n, uint32_t m);
  void WriteDcaTxctrl(uint32_t n, uint32_t m);
  void WriteDcaRxctrl(uint32_t n, uint32_t m);
  void WriteDcaRxctrlClear(uint32_t n, uint32_t m);
  void WriteDcaRxctrl_1(uint32_t n, uint32_t m);
  void WriteDcaRxctrl_2(uint32_t n, uint32_t m);
  void WriteDcaCtrl(uint32_t m);
  void ReadDcaTxctrl(uint32_t n);
  void ReadDcaRxctrl(uint32_t n);
  
  void WriteRdbal_1(uint32_t n, uint32_t m);
  void WriteRdbal_2(uint32_t n, uint32_t m);

  void WriteRdbah_1(uint32_t n, uint32_t m);
  void WriteRdbah_2(uint32_t n, uint32_t m);

  void WriteRdlen_1(uint32_t n, uint32_t m);
  void WriteRdlen_2(uint32_t n, uint32_t m);

  void WriteSrrctl_1(uint32_t n, uint32_t m);
  void WriteSrrctlZero(uint32_t n);
  void WriteSrrctl_1_desctype(uint32_t n, uint32_t m);
  void WriteRscdbu(uint32_t m);

  void WriteRdt_1(uint32_t n, uint32_t m);
  void WriteRdh_1(uint32_t n, uint32_t m);
  void WriteRdt_2(uint32_t n, uint32_t m);

  void WriteIvarAlloc0(uint32_t n, uint32_t m);
  void WriteIvarAllocval0(uint32_t n, uint32_t m);
  void WriteIvarAlloc1(uint32_t n, uint32_t m);
  void WriteIvarAllocval1(uint32_t n, uint32_t m);
  void WriteIvarAlloc2(uint32_t n, uint32_t m);
  void WriteIvarAllocval2(uint32_t n, uint32_t m);
  void WriteIvarAlloc3(uint32_t n, uint32_t m);
  void WriteIvarAllocval3(uint32_t n, uint32_t m);

  void WriteSecrxctrl_Rx_Dis(uint32_t m);

  void WriteTdbal(uint32_t n, uint32_t m);
  void WriteTdbah(uint32_t n, uint32_t m);
  void WriteTdlen(uint32_t n, uint32_t m);

  void WriteTdh(uint32_t n, uint32_t m);
  void WriteTdt(uint32_t n, uint32_t m);
  uint32_t ReadTdt(uint32_t n);

  void WriteTdwbal(uint32_t n, uint32_t m);
  void WriteTdwbah(uint32_t n, uint32_t m);

  void WriteHlreg0(uint32_t m);
  void WriteRdrxctl(uint32_t m);
  void WriteRdrxctlRSCFRSTSIZE(uint32_t m);

  void WriteEiac(uint32_t m);
  void WriteEimsn(uint32_t n, uint32_t m);

  void WriteRfctl(uint32_t m);

  void WriteRscctl(uint32_t n, uint32_t m);
  void WritePsrtype(uint32_t n, uint32_t m);

  void WriteRxcsum(uint32_t m);
  void WriteTxpbthresh(uint32_t n, uint32_t m);
  void WriteMrqc(uint32_t m);
  void WriteDtxmxszrq(uint32_t m);
  void WriteMflcn(uint32_t m);
  void WriteReta(uint32_t n, uint32_t m);
  void WriteRssrk(uint32_t n, uint32_t m) {
    kassert(n < 10);
    bar0_.Write32(0x0EB80 + 4 * n, m);
  }

  void WritePsrtypeZero(uint32_t n);

  void WriteRttdcs(uint32_t m);
  void WriteRttdcsArbdisEn(uint32_t m);
  void WriteRxpbsize(uint32_t n, uint32_t m);
  void WriteTxpbsize(uint32_t n, uint32_t m);
  void WriteTxpbThresh(uint32_t n, uint32_t m);
  void WriteMtqc(uint32_t m);
  void WritePfvtctl(uint32_t m);
  void WriteRtrup2tc(uint32_t m);
  void WriteRttup2tc(uint32_t m);
  void WritePfqde(uint32_t m);
  void WriteRttdt1c(uint32_t m);
  void WriteRttdt2c(uint32_t n, uint32_t m);
  void WriteRttpt2c(uint32_t n, uint32_t m);
  void WriteRtrpt4c(uint32_t n, uint32_t m);
  void WriteRttpcs(uint32_t m);
  void WriteRtrpcs(uint32_t m);
  void WritePfvml2flt(uint32_t n, uint32_t m);

  void WriteMngtxmap(uint32_t m);

  void WriteRxfeccerr0(uint32_t m);
  void WriteMaxfrs(uint32_t m);
  
  uint8_t ReadRdrxctlDmaidone();

  void ReadEicr();
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

  uint8_t ReadRxdctl_1_enable(uint32_t n);
  uint8_t ReadSecrxstat_Sr_Rdy();

  uint8_t ReadTxdctl_enable(uint32_t n);

  uint16_t ReadRdh_1(uint32_t n);
  uint16_t ReadTdh(uint32_t n);
  uint16_t ReadRdt_1(uint32_t n);

  // some statistics
  uint32_t ReadTpr();
  uint32_t ReadGprc();
  bool ReadLinksLinkUp();

  // Process packet functions
  void ProcessPacket(uint32_t n);
  uint32_t GetRxBuf(uint32_t* len, uint64_t* bAddr);
  void SendPacket(uint32_t n);

  // dump per core stats if STATS_EN
  void DumpStats();
  e10k_queue_t& GetQueue() const { return *ixgq; }

  e10Kq& GetMultiQueue(uint32_t index) const { return *ixgmq[index]; }

  pci::Device& dev_;
  pci::Bar& bar0_;

  struct IxgbeRegs {
    volatile uint32_t kIxgbeCtrl;
    volatile uint32_t kIxgbeCtrlBak;
    volatile uint32_t kIxgbeStatus;
  };

  e10k_queue_t* ixgq;
  uint8_t rcv_vector{0};

  std::vector<std::unique_ptr<e10Kq>> ixgmq;

  friend class IxgbeDriverRep;
};  // class IxgbeDriver

class IxgbeDriverRep : public MulticoreEbb<IxgbeDriverRep, IxgbeDriver>, Timer::Hook {
 public:
  explicit IxgbeDriverRep(const IxgbeDriver& root);
  void Run();
  void ReceivePoll();
  void ReclaimTx();
  void ReclaimRx();
  void Send(std::unique_ptr<IOBuf> buf, PacketInfo pinfo);
  void SendUdp(std::unique_ptr<IOBuf> buf, uint64_t len, PacketInfo pinfo);
  void SendTCPChained(std::unique_ptr<IOBuf> buf, uint64_t len, uint64_t num_chains, PacketInfo pinfo);
  void SendTCPUnchained(std::unique_ptr<IOBuf> buf, uint64_t len, PacketInfo pinfo);
  
  //void AddContext(uint8_t idx, uint8_t maclen, uint16_t iplen, uint8_t l4len,
  //               enum l4_type l4type);
  //void AddTx(uint64_t pa, uint64_t len, uint64_t totallen, bool first,
  //           bool last, uint8_t ctx, bool ip_cksum, bool tcpudp_cksum, bool tse, int hdr_len);
  void StartTimer();
  void StopTimer();
  
 private:
  uint16_t ReadRdh_1(uint32_t n);
  uint16_t ReadRdt_1(uint32_t n);
  void WriteRdt_1(uint32_t n, uint32_t m);
  void WriteRdh_1(uint32_t n, uint32_t m);
  void WriteTdt_1(uint32_t n, uint32_t m);
  void WriteEimcn(uint32_t n, uint32_t m);
  void WriteEimc(uint32_t m);
  void WriteEims(uint32_t m);
  uint32_t ReadTdh_1(uint32_t n);
  uint32_t ReadTdt_1(uint32_t n);
  uint32_t GetRxBuf(uint32_t* len, uint64_t* bAddr, uint64_t* rxflag,
                    bool* process_rsc, uint32_t* rnt, uint32_t* rxhead);
  void Fire() override;
  
  const IxgbeDriver& root_;
  e10k_queue_t& ixgq_;
  IxgbeDriver::e10Kq& ixgmq_;

  EventManager::IdleCallback receive_callback_;

};  // class IxgbeDriverRep

}  // namespace ebbrt

#endif  // BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_DRIVER_H_
