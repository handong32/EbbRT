#ifndef BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_H_
#define BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_H_

// from https://github.com/cisco-open-source/ethtool/ixgbe.c

/* Register Bit Masks */
#define IXGBE_FCTRL_SBP 0x00000002
#define IXGBE_FCTRL_MPE 0x00000100
#define IXGBE_FCTRL_UPE 0x00000200
#define IXGBE_FCTRL_BAM 0x00000400
#define IXGBE_FCTRL_PMCF 0x00001000
#define IXGBE_FCTRL_DPF 0x00002000
#define IXGBE_FCTRL_RPFCE 0x00004000
#define IXGBE_FCTRL_RFCE 0x00008000
#define IXGBE_VLNCTRL_VET 0x0000FFFF
#define IXGBE_VLNCTRL_CFI 0x10000000
#define IXGBE_VLNCTRL_CFIEN 0x20000000
#define IXGBE_VLNCTRL_VFE 0x40000000
#define IXGBE_VLNCTRL_VME 0x80000000
#define IXGBE_LINKS_UP 0x40000000
#define IXGBE_LINKS_SPEED 0x20000000
#define IXGBE_SRRCTL_BSIZEPKT_MASK 0x0000007F
#define IXGBE_HLREG0_TXCRCEN 0x00000001
#define IXGBE_HLREG0_RXCRCSTRP 0x00000002
#define IXGBE_HLREG0_JUMBOEN 0x00000004
#define IXGBE_HLREG0_TXPADEN 0x00000400
#define IXGBE_HLREG0_LPBK 0x00008000
#define IXGBE_RMCS_TFCE_802_3X 0x00000008
#define IXGBE_RMCS_TFCE_PRIORITY 0x00000010
#define IXGBE_FCCFG_TFCE_802_3X 0x00000008
#define IXGBE_FCCFG_TFCE_PRIORITY 0x00000010
#define IXGBE_MFLCN_PMCF 0x00000001 /* Pass MAC Control Frames */
#define IXGBE_MFLCN_DPF 0x00000002 /* Discard Pause Frame */
#define IXGBE_MFLCN_RPFCE 0x00000004 /* Receive Priority FC Enable */
#define IXGBE_MFLCN_RFCE 0x00000008 /* Receive FC Enable */

/***********************
   * RX
   * Descriptors
   **********************/
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

/***********************
 * TX
 * Descriptors
 **********************/
// 7.2.3.2.2 Legacy Transmit Descriptor Format
typedef union {
  uint64_t raw[2];

  struct {
    uint64_t buffer_address;

    union {
      uint64_t word2_raw;

      struct {
        uint64_t length : 16;
        uint64_t cso : 8;

        // cmd
        uint64_t eop : 1;
        uint64_t ifcs : 1;
        uint64_t ic : 1;
        uint64_t rs : 1;
        uint64_t rsvd_1 : 1;
        uint64_t dext : 1;
        uint64_t vle : 1;
        uint64_t rsvd_2 : 1;

        // sta
        uint64_t dd : 1;
        uint64_t rsvd_3 : 3;

        uint64_t rsvd_4 : 4;
        uint64_t css : 8;
        uint64_t vlan : 16;
      };
    };

  } __attribute__((packed));
} tdesc_legacy_t;

// 7.2.3.2.3 Advanced Transmit Context Descriptor
typedef union {
  uint64_t raw[2];

  struct {
    union {
      uint64_t raw_1;

      struct {
        uint64_t iplen : 9;
        uint64_t maclen : 7;
        uint64_t vlan : 16;
        uint64_t ipsec_sa_index : 10;
        uint64_t fcoef : 6;
        uint64_t rsvd_1 : 16;
      };
    };

    union {
      uint64_t raw_2;

      struct {
        // tucmd
        uint64_t ipsec_esp_len : 9;
        uint64_t snap : 1;
        uint64_t ipv4 : 1;
        uint64_t l4t : 2;  // l4 packet type
        uint64_t ipsec_type : 1;
        uint64_t encyption : 1;
        uint64_t fcoe : 1;
        uint64_t rsvd_2 : 4;

        uint64_t dytp : 4;
        uint64_t rsvd_3 : 5;
        uint64_t dext : 1;

        uint64_t bcntlen : 6;
        uint64_t idx : 1;
        uint64_t rsvd_4 : 3;
        uint64_t l4len : 8;
        uint64_t mss : 16;
      };
    };

  } __attribute__((packed));

} tdesc_advance_ctxt_wb_t;

// 7.2.3.2.4 Advanced Transmit Data Descriptor  - Read Format
typedef union {
  uint64_t raw[2];

  struct {
    uint64_t address;

    union {
      uint64_t raw2;
      struct {
        uint64_t dtalen : 16;
        uint64_t rsvd_1 : 2;

        // mac
        uint64_t mac_ilsec : 1;
        uint64_t mac_1588 : 1;

        uint64_t dtyp : 4;

        // dcmd
        uint64_t eop : 1;
        uint64_t ifcs : 1;
        uint64_t rsvd_2 : 1;
        uint64_t rs : 1;
        uint64_t rsvd_3 : 1;
        uint64_t dext : 1;
        uint64_t vle : 1;
        uint64_t tse : 1;

        // status
        uint64_t dd : 1;
        uint64_t rsvd_4 : 3;

        // idx
        uint64_t idx : 1;
        uint64_t rsvd_5 : 2;
        uint64_t cc : 1;

        // popts
        uint64_t ixsm : 1;
        uint64_t txsm : 1;
        uint64_t ipsec : 1;
        uint64_t rsvd_6 : 3;

        uint64_t paylen : 18;
      };
    };
  };

} tdesc_advance_tx_rf_t;

// Advanced Transmit Data Descriptor  - Write-back Format
typedef union {
  uint64_t raw[2];

  struct {
    uint64_t rsvd_1;

    union {
      uint64_t raw2;

      struct {
        uint64_t rsvd_2 : 32;

        // status
        uint64_t dd : 1;
        uint64_t rsvd_3 : 3;

        uint64_t rsvd_4 : 28;
      };
    };
  };

} tdesc_advance_tx_wbf_t;

/**
 * Context structure for RX descriptors. This is needed to implement RSC,
 * since
 * we need to be able to chain buffers together. */
/*struct e10k_queue_rxctx {
  void* opaque;
  struct e10k_queue_rxctx* previous;
  bool used;
};

// Queue
typedef struct {
  tdesc_advance_tx_wbf_t* tx_ring;
  void** tx_opaque;
  bool* tx_isctx;
  size_t tx_head;
  size_t tx_tail;
  size_t tx_lasttail;
  size_t tx_size;
  // uint32_t* tx_hwb;

  rdesc_advance_wbf_t* rx_ring;
  struct e10k_queue_rxctx* rx_context;
  size_t rx_head;
  size_t rx_tail;
  size_t rx_size;

  // struct e10k_queue_ops           ops;
  // void*                           opaque;

  } e10k_queue_t;*/

// Queue
typedef struct {
  rdesc_legacy_t* rx_ring;
  size_t rx_head;
  size_t rx_tail;
  size_t rx_size;

  tdesc_legacy_t* tx_ring;
  size_t tx_head;
  size_t tx_tail;
  size_t tx_size;

} e10k_queue_t;

#endif  // BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_H_
