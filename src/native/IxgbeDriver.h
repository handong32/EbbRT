//          Copyright Boston University SESA Group 2013 - 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#ifndef BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_DRIVER_H_
#define BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_DRIVER_H_

#include "../Align.h"
#include "Debug.h"
#include "Fls.h"
#include "PageAllocator.h"
#include "Pci.h"
#include "Pfn.h"
#include "SlabAllocator.h"

namespace ebbrt {

class IxgbeDriver {
 public:
  static bool Probe(pci::Device& dev) {
    if (dev.GetVendorId() == kIxgbeVendorId &&
        dev.GetDeviceId() == kIxgbeDeviceId && dev.GetFunc() == 0) {
      IxgbeDriver::Create(dev);
      return true;
    }
    return false;
  }

  static void Create(pci::Device& dev);

 protected:
  static const constexpr uint16_t kIxgbeVendorId = 0x8086;
  static const constexpr uint16_t kIxgbeDeviceId = 0x10FB;

  explicit IxgbeDriver(pci::Device& dev) : dev_(dev), bar0_(dev.GetBar(0)) {
    dev_.SetBusMaster(true);
    ebbrt::kprintf("%s constructor\n", __FUNCTION__);
  }

  // 7.1.5 Legacy Receive Descriptor

  /*
    // 7.1.5 Legacy Receive Descriptor
  typedef struct {
    uint64_t buffer;
    uint16_t length;
    uint16_t checksum;

    // Status Field
    union {
      struct {
        uint8_t dd : 1;  // Descriptor DOne
        uint8_t eop : 1;  // End of Packet
        uint8_t rsrvd : 1;
        uint8_t vp : 1;  // Vlan Packet
        uint8_t udpcs : 1;  // Udp Checksum
        uint8_t l4cs : 1;  // L4 checksum
        uint8_t ipcs : 1;  // Ipv4 checksum
        uint8_t pif : 1;  // non unicast address
      } status_bits;
      uint8_t status;
    };

    // Receive Errors
    union {
      struct {
        uint8_t rxe : 1;  // mac error
        uint8_t rsrvd : 5;
        uint8_t tcpe : 1;  // tcp/udp checksum error
        uint8_t ipe : 1;  // ipv4 checksum
      } recv_bits;
      uint8_t recv;
    };

    uint16_t vlan_tag;
  } rdesc_legacy_t;

  // 7.1.6.2 Advanced Receive Descriptors — Write-Back Format
  typedef struct {

    union {
      struct {
        uint8_t rss_type : 4;

        // packet type
        uint8_t pt_ipv4 : 1;
        uint8_t pt_ipv4e : 1;
        uint8_t pt_ipv6 : 1;
        uint8_t pt_ipv6e : 1;
        uint8_t pt_tcp : 1;
        uint8_t pt_udp : 1;
        uint8_t pt_sctp : 1;
        uint8_t pt_nfs : 1;
        uint8_t pt_isesp : 1;
        uint8_t pt_isah : 1;
        uint8_t pt_linksec : 1;
        uint8_t pt_l2packet : 1;
        uint8_t pt_rsvd : 1;

        uint8_t rsccnt : 4;
        uint16_t header_len : 10;
        uint8_t sph : 1;
      } p_bits;
      uint32_t packed1;
    };

    uint32_t param;  // RSS Hash or FCOE_PARAM or Flow Director Filters ID
    // (32-bit offset 32, 1st line) Fragment Checksum (16-bit
    // offset 48, 1st line)

    union {
      struct {
          // extended status, nextp
          uint8_t dd : 1;
          uint8_t eop : 1;
          uint8_t flm : 1;
          uint8_t vp : 1;

          union {
              struct {
                  uint8_t udpcs : 1;
                  uint8_t l4i : 1;
              } fcoe_bits;
          };

      };
      uint32_t extended;
    };
  } rdesc_advance_wbf_t;
  */
 private:
  void InitStruct();
  void DeviceInfo();
  void Init();
  void PhyInit();
  void StopDevice();
  void GlobalReset();
  void SetupQueue();

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
  void WriteEimc(uint32_t m);
  void WriteTxdctl(uint32_t n, uint32_t m);
  void WriteRxdctl_1(uint32_t n, uint32_t m);
  void WriteRxdctl_2(uint32_t n, uint32_t m);
  void WriteCtrl(uint32_t m);
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
  uint32_t ReadTpr();

  bool ReadLinksLinkUp();

  pci::Device& dev_;
  pci::Bar& bar0_;

  struct IxgbeRegs {
    volatile uint32_t kIxgbeCtrl;
    volatile uint32_t kIxgbeCtrlBak;
    volatile uint32_t kIxgbeStatus;
  };

  // 7.1.5 Legacy Receive Descriptor, Table 7 - 11
  typedef union {

    uint64_t raw[2];

    struct {
      uint64_t buffer_address;

      union {
        uint64_t word2_raw;

        struct {
          uint64_t length : 16;
          uint64_t fragment_checksum : 16;

          // uint64_t status : 8;
          uint64_t dd : 1;
          uint64_t eop : 1;
          uint64_t rsvd : 1;
          uint64_t vp : 1;
          uint64_t udpcs : 1;
          uint64_t l4cs : 1;
          uint64_t ipcs : 1;
          uint64_t pif : 1;

          uint64_t errors : 8;
          uint64_t vlan_tag : 16;
        };  // struct

      };  // union

    } __attribute__((packed));  // struct

  } rdesc_legacy_t;  // typedef union

  // 7.1.6.1 Advanced Receive Descriptors Read Format
  typedef struct {
    uint64_t packet_buffer;
    uint64_t header_buffer;
  } rdesc_advance_rf_t;

  // 7.1.6.2 Advanced Receive Descriptors — Write-Back Format
  typedef union {
    uint64_t raw[2];
    struct {
      union {
        uint32_t raw32_1;
        struct {
          uint32_t rss_type : 4;

          // packet type
          uint32_t pt_ipv4 : 1;
          uint32_t pt_ipv4e : 1;
          uint32_t pt_ipv6 : 1;
          uint32_t pt_ipv6e : 1;
          uint32_t pt_tcp : 1;
          uint32_t pt_udp : 1;
          uint32_t pt_sctp : 1;
          uint32_t pt_nfs : 1;
          uint32_t pt_isesp : 1;
          uint32_t pt_isah : 1;
          uint32_t pt_linksec : 1;
          uint32_t pt_l2packet : 1;
          uint32_t pt_rsvd : 1;

          uint32_t rsccnt : 4;
          uint32_t hdr_len : 10;
          uint32_t sph : 1;
        };
      };  // union raw32_1

      union {
        uint32_t raw32_2;
        uint32_t rss_hash;
        uint32_t fragment_checksum;
        uint32_t rtt;
        uint32_t fcoe_param;
        uint32_t flow_directors_filters_id;  // may need more, page 317
      };  // union raw32_2

      union {
        uint32_t raw32_3;

        struct {
          // extended status
          uint32_t dd : 1;
          uint32_t eop : 1;
          uint32_t flm : 1;
          uint32_t vp : 1;

          // fcstat - 2 bits
          uint32_t udpcs : 1;
          uint32_t l4i : 1;

          uint32_t ipcs : 1;
          uint32_t pif : 1;
          uint32_t rsvd_1 : 1;
          uint32_t vext : 1;
          uint32_t udpv : 1;
          uint32_t llint : 1;
          uint32_t rsvd_2 : 4;
          uint32_t ts : 1;
          uint32_t secp : 1;
          uint32_t lb : 1;
          uint32_t rsvd_3 : 1;

          // extended error
          uint32_t fdierr : 3;
          uint32_t hbo : 1;
          uint32_t rsvd : 3;
          uint32_t secerr : 2;
          uint32_t rxe : 1;
          uint32_t l4e : 1;
          uint32_t ipe : 1;
        } status_last_descriptor;

        struct {
          // extended status
          uint32_t dd : 1;
          uint32_t eop : 1;
          uint32_t rsvd : 2;
          uint32_t next_descriptor_ptr : 16;

          // extended error
          uint32_t error : 12;
        } status_non_last_descriptor;
      };  // union raw32_3

      union {
        uint32_t raw32_4;
        struct {
          uint32_t pkt_len : 16;
          uint32_t vlan_tag : 16;
        };
      };  // union raw32_4

    } __attribute__((packed));  // struct
  } rdesc_advance_wbf_t;
};
}  // namespace ebbrt

#endif  // BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_DRIVER_H_
