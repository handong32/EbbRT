//          Copyright Boston University SESA Group 2013 - 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#ifndef BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_DRIVER_H_
#define BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_DRIVER_H_

#include "../Align.h"
#include "../MulticoreEbb.h"
#include "../SpinLock.h"
#include "Debug.h"
#include "Fls.h"
#include "Ixgbe.h"
#include "Net.h"
#include "PageAllocator.h"
#include "Pci.h"
#include "Pfn.h"
#include "SlabAllocator.h"

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
  explicit IxgbeDriver(pci::Device& dev)
      : itf_(network_manager->NewInterface(*this)), dev_(dev),
        bar0_(dev.GetBar(0)) {
    dev_.SetBusMaster(true);

    // set up interrupts, polling won't work after this
    auto msix = dev_.MsixEnable();
    kbugon(!msix, "Ixgbe without msix is unsupported\n");

    ebbrt::kprintf("%s constructor\n", __FUNCTION__);
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

  void Run();
  void Send(std::unique_ptr<IOBuf> buf, PacketInfo pinfo) override;
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
  static const constexpr uint32_t RXBUFSZ = 2048;

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
  void SetupQueue(uint32_t i);

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

  void WriteDcaTxctrlTxdescWbro(uint32_t n, uint32_t m);
  void WriteDcaRxctrl_1(uint32_t n, uint32_t m);
  void WriteDcaRxctrl_2(uint32_t n, uint32_t m);

  void WriteRdbal_1(uint32_t n, uint32_t m);
  void WriteRdbal_2(uint32_t n, uint32_t m);

  void WriteRdbah_1(uint32_t n, uint32_t m);
  void WriteRdbah_2(uint32_t n, uint32_t m);

  void WriteRdlen_1(uint32_t n, uint32_t m);
  void WriteRdlen_2(uint32_t n, uint32_t m);

  void WriteSrrctl_1(uint32_t n, uint32_t m);
  // void WriteSrrctl_1_bsizepacket(uint32_t n, uint32_t m);
  void WriteSrrctl_1_desctype(uint32_t n, uint32_t m);

  void WriteRdt_1(uint32_t n, uint32_t m);
  void WriteRdh_1(uint32_t n, uint32_t m);
  void WriteRdt_2(uint32_t n, uint32_t m);

  void WriteIvarAlloc0(uint32_t n, uint32_t m);
  void WriteIvarAllocval0(uint32_t n, uint32_t m);
  void WriteIvarAlloc1(uint32_t n, uint32_t m);
  void WriteIvarAllocval1(uint32_t n, uint32_t m);

  void WriteSecrxctrl_Rx_Dis(uint32_t m);

  void WriteTdbal(uint32_t n, uint32_t m);
  void WriteTdbah(uint32_t n, uint32_t m);
  void WriteTdlen(uint32_t n, uint32_t m);

  void WriteTdh(uint32_t n, uint32_t m);
  void WriteTdt(uint32_t n, uint32_t m);

  void WriteTdwbal(uint32_t n, uint32_t m);
  void WriteTdwbah(uint32_t n, uint32_t m);
  
  void WriteHlreg0(uint32_t m);
  void WriteRdrxctl(uint32_t m);
  void WriteEiac(uint32_t m);
  void WriteEimsn(uint32_t n, uint32_t m);

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

  // statistics
  uint32_t ReadTpr();
  uint32_t ReadGprc();
  bool ReadLinksLinkUp();

  // Process
  void ProcessPacket(uint32_t n);
  uint32_t GetRxBuf(uint32_t* len, uint64_t* bAddr);
  void SendPacket(uint32_t n);

  e10k_queue_t& GetQueue() const { return *ixgq; }

  pci::Device& dev_;
  pci::Bar& bar0_;

  struct IxgbeRegs {
    volatile uint32_t kIxgbeCtrl;
    volatile uint32_t kIxgbeCtrlBak;
    volatile uint32_t kIxgbeStatus;
  };

  e10k_queue_t* ixgq;

  //void* rxbuf;

  friend class IxgbeDriverRep;
};  // class IxgbeDriver

class IxgbeDriverRep : public MulticoreEbb<IxgbeDriverRep, IxgbeDriver> {
 public:
  explicit IxgbeDriverRep(const IxgbeDriver& root);
  void Run();
  void ReceivePoll(uint32_t n);
  void Send(std::unique_ptr<IOBuf> buf, PacketInfo pinfo);
  void AddContext(uint8_t idx, uint8_t maclen, 
		  uint16_t iplen, uint8_t l4len, enum l4_type l4type);
  void AddTx(const unsigned char *pa, uint64_t len, 
	     bool first, bool last, 
	     uint8_t ctx, bool ip_cksum, bool tcpudp_cksum);

 private:
  uint16_t ReadRdh_1(uint32_t n);
  void WriteRdt_1(uint32_t n, uint32_t m);
  void WriteTdt_1(uint32_t n, uint32_t m);
  uint32_t GetRxBuf(uint32_t* len, uint64_t* bAddr);

  const IxgbeDriver& root_;
  e10k_queue_t& ixgq_;
  EventManager::IdleCallback receive_callback_;

};  // class IxgbeDriverRep

}  // namespace ebbrt

#endif  // BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_DRIVER_H_
