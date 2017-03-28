#ifndef BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_H_
#define BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_H_

#define udelay ebbrt::clock::SleepMicro
#define mdelay ebbrt::clock::SleepMilli
#define KPRINTF ebbrt::kprintf

#define IXGBE_82599_MAX_TX_QUEUES 1
#define IXGBE_82599_MAX_RX_QUEUES 1
#define IXGBE_82599_RAR_ENTRIES   1
#define IXGBE_82599_MC_TBL_SIZE   1
#define IXGBE_82599_VFT_TBL_SIZE  1
#define IXGBE_82599_RX_PB_SIZE	  512

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;


/* Error Codes */
#define IXGBE_ERR_EEPROM                        -1
#define IXGBE_ERR_EEPROM_CHECKSUM               -2
#define IXGBE_ERR_PHY                           -3
#define IXGBE_ERR_CONFIG                        -4
#define IXGBE_ERR_PARAM                         -5
#define IXGBE_ERR_MAC_TYPE                      -6
#define IXGBE_ERR_UNKNOWN_PHY                   -7
#define IXGBE_ERR_LINK_SETUP                    -8
#define IXGBE_ERR_ADAPTER_STOPPED               -9
#define IXGBE_ERR_INVALID_MAC_ADDR              -10
#define IXGBE_ERR_DEVICE_NOT_SUPPORTED          -11
#define IXGBE_ERR_MASTER_REQUESTS_PENDING       -12
#define IXGBE_ERR_INVALID_LINK_SETTINGS         -13
#define IXGBE_ERR_AUTONEG_NOT_COMPLETE          -14
#define IXGBE_ERR_RESET_FAILED                  -15
#define IXGBE_ERR_SWFW_SYNC                     -16
#define IXGBE_ERR_PHY_ADDR_INVALID              -17
#define IXGBE_ERR_I2C                           -18
#define IXGBE_ERR_SFP_NOT_SUPPORTED             -19
#define IXGBE_ERR_SFP_NOT_PRESENT               -20
#define IXGBE_ERR_SFP_NO_INIT_SEQ_PRESENT       -21
#define IXGBE_ERR_NO_SAN_ADDR_PTR               -22
#define IXGBE_ERR_FDIR_REINIT_FAILED            -23
#define IXGBE_ERR_EEPROM_VERSION                -24
#define IXGBE_ERR_NO_SPACE                      -25
#define IXGBE_ERR_OVERTEMP                      -26
#define IXGBE_ERR_FC_NOT_NEGOTIATED             -27
#define IXGBE_ERR_FC_NOT_SUPPORTED              -28
#define IXGBE_ERR_SFP_SETUP_NOT_COMPLETE        -30
#define IXGBE_ERR_PBA_SECTION                   -31
#define IXGBE_ERR_INVALID_ARGUMENT              -32
#define IXGBE_ERR_HOST_INTERFACE_COMMAND        -33
#define IXGBE_NOT_IMPLEMENTED                   0x7FFFFFFF

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

// from https://github.com/handong32/EbbRT/blob/linux_ixgbe/src/native/Ixgbe.h
#define IXGBE_EEC		0x10010
#define IXGBE_EEC_PRES		0x00000100 /* EEPROM Present */
#define IXGBE_EEC_SIZE		0x00007800 /* EEPROM Size */
#define IXGBE_EEC_SIZE_SHIFT		11
#define IXGBE_EEC_ADDR_SIZE	0x00000400
#define IXGBE_EEPROM_WORD_SIZE_SHIFT	6

/* General Registers */
#define IXGBE_CTRL      0x00000
#define IXGBE_STATUS    0x00008
#define IXGBE_CTRL_EXT  0x00018
#define IXGBE_ESDP      0x00020
#define IXGBE_EODSDP    0x00028
#define IXGBE_I2CCTL    0x00028
#define IXGBE_LEDCTL    0x00200
#define IXGBE_FRTIMER   0x00048
#define IXGBE_TCPTIMER  0x0004C
#define IXGBE_CORESPARE 0x00600
#define IXGBE_EXVET     0x05078

#define IXGBE_FWSM      0x10148
/* Firmware Semaphore Register */
#define IXGBE_FWSM_MODE_MASK	0xE
#define IXGBE_FWSM_FW_MODE_PT	0x4
#define IXGBE_MANC      0x05820
/* Management Bit Fields and Masks */
#define IXGBE_MANC_RCV_TCO_EN	0x00020000 /* Rcv TCO packet enable */
#define IXGBE_FACTPS    0x10150
#define IXGBE_FACTPS_MNGCG      0x20000000 /* Manageblility Clock Gated */

#define IXGBE_WUS       0x05810
#define IXGBE_RXCTRL    0x03000
/* Interrupt Registers */
#define IXGBE_EICR      0x00800
#define IXGBE_EICS      0x00808
#define IXGBE_EIMS      0x00880
#define IXGBE_EIMC      0x00888
#define IXGBE_EIAC      0x00810
#define IXGBE_EIAM      0x00890
/* Interrupt clear mask */
#define IXGBE_IRQ_CLEAR_MASK    0xFFFFFFFF

#define IXGBE_TXDCTL(_i) (0x06028 + ((_i) * 0x40))
/* Transmit Config masks */
#define IXGBE_TXDCTL_ENABLE     0x02000000 /* Enable specific Tx Queue */
#define IXGBE_TXDCTL_SWFLSH     0x04000000 /* Tx Desc. write-back flushing */

/* Receive Config masks */
#define IXGBE_RXCTRL_RXEN       0x00000001  /* Enable Receiver */
#define IXGBE_RXCTRL_DMBYPS     0x00000002  /* Descriptor Monitor Bypass */
#define IXGBE_RXDCTL_ENABLE     0x02000000  /* Enable specific Rx Queue */
#define IXGBE_RXDCTL_SWFLSH     0x04000000  /* Rx Desc. write-back flushing */
#define IXGBE_RXDCTL_RLPMLMASK  0x00003FFF  /* Only supported on the X540 */
#define IXGBE_RXDCTL_RLPML_EN   0x00008000
#define IXGBE_RXDCTL_VME        0x40000000  /* VLAN mode enable */

/* Receive DMA Registers */
#define IXGBE_RDBAL(_i) (((_i) < 64) ? (0x01000 + ((_i) * 0x40)) : \
			 (0x0D000 + (((_i) - 64) * 0x40)))
#define IXGBE_RDBAH(_i) (((_i) < 64) ? (0x01004 + ((_i) * 0x40)) : \
			 (0x0D004 + (((_i) - 64) * 0x40)))
#define IXGBE_RDLEN(_i) (((_i) < 64) ? (0x01008 + ((_i) * 0x40)) : \
			 (0x0D008 + (((_i) - 64) * 0x40)))
#define IXGBE_RDH(_i)   (((_i) < 64) ? (0x01010 + ((_i) * 0x40)) : \
			 (0x0D010 + (((_i) - 64) * 0x40)))
#define IXGBE_RDT(_i)   (((_i) < 64) ? (0x01018 + ((_i) * 0x40)) : \
			 (0x0D018 + (((_i) - 64) * 0x40)))
#define IXGBE_RXDCTL(_i) (((_i) < 64) ? (0x01028 + ((_i) * 0x40)) : \
			 (0x0D028 + (((_i) - 64) * 0x40)))
#define IXGBE_RSCCTL(_i) (((_i) < 64) ? (0x0102C + ((_i) * 0x40)) : \
			 (0x0D02C + (((_i) - 64) * 0x40)))

/* CTRL Bit Masks */
#define IXGBE_CTRL_GIO_DIS      0x00000004 /* Global IO Master Disable bit */
#define IXGBE_CTRL_LNK_RST      0x00000008 /* Link Reset. Resets everything. */
#define IXGBE_CTRL_RST          0x04000000 /* Reset (SW) */
#define IXGBE_CTRL_RST_MASK     (IXGBE_CTRL_LNK_RST | IXGBE_CTRL_RST)
#define IXGBE_STATUS_GIO            0x00080000 /* GIO Master Enable Status */

/* Number of 100 microseconds we wait for PCI Express master disable */
#define IXGBE_PCI_MASTER_DISABLE_TIMEOUT 800

/* MAC Registers */
#define IXGBE_MACS      0x0429C
#define IXGBE_AUTOC     0x042A0
#define IXGBE_LINKS     0x042A4
#define IXGBE_LINKS2    0x04324
#define IXGBE_AUTOC2    0x042A8
#define IXGBE_AUTOC3    0x042AC

/* LED modes */
#define IXGBE_LED_LINK_UP       0x0
#define IXGBE_LED_LINK_10G      0x1
#define IXGBE_LED_MAC           0x2
#define IXGBE_LED_FILTER        0x3
#define IXGBE_LED_LINK_ACTIVE   0x4
#define IXGBE_LED_LINK_1G       0x5
#define IXGBE_LED_ON            0xE
#define IXGBE_LED_OFF           0xF

#define IXGBE_LINK_UP_TIME      90 /* 9.0 Seconds */
#define IXGBE_AUTO_NEG_TIME     45 /* 4.5 Seconds */
#define IXGBE_LINKS_SPEED_82599     0x30000000
#define IXGBE_LINKS_SPEED_10G_82599 0x30000000
#define IXGBE_LINKS_SPEED_1G_82599  0x20000000
#define IXGBE_LINKS_SPEED_100_82599 0x10000000
#define IXGBE_AUTOC2_LINK_DISABLE_MASK        0x70000000

#define IXGBE_WRITE_FLUSH() IXGBE_READ_REG(IXGBE_STATUS)

/* Multicast Table Array - 128 entries */
#define IXGBE_MTA(_i)   (0x05200 + ((_i) * 4))
#define IXGBE_RAL(_i)   (((_i) <= 15) ? (0x05400 + ((_i) * 8)) : \
                         (0x0A200 + ((_i) * 8)))
#define IXGBE_RAH(_i)   (((_i) <= 15) ? (0x05404 + ((_i) * 8)) : \
                         (0x0A204 + ((_i) * 8)))
#define IXGBE_MPSAR_LO(_i) (0x0A600 + ((_i) * 8))
#define IXGBE_MPSAR_HI(_i) (0x0A604 + ((_i) * 8))

#define IXGBE_UTA(_i)        (0x0F400 + ((_i) * 4))

/* Link speed */
typedef u32 ixgbe_link_speed;
#define IXGBE_LINK_SPEED_UNKNOWN   0
#define IXGBE_LINK_SPEED_100_FULL  0x0008
#define IXGBE_LINK_SPEED_1GB_FULL  0x0020
#define IXGBE_LINK_SPEED_10GB_FULL 0x0080
#define IXGBE_LINK_SPEED_82598_AUTONEG (IXGBE_LINK_SPEED_1GB_FULL | \
                                        IXGBE_LINK_SPEED_10GB_FULL)
#define IXGBE_LINK_SPEED_82599_AUTONEG (IXGBE_LINK_SPEED_100_FULL | \
                                        IXGBE_LINK_SPEED_1GB_FULL | \
                                        IXGBE_LINK_SPEED_10GB_FULL)


#define IXGBE_FCTRL     0x05080
#define IXGBE_VLNCTRL   0x05088
#define IXGBE_MCSTCTRL  0x05090
#define IXGBE_MRQC      0x05818

/* Autonegotiation advertised speeds */
typedef u32 ixgbe_autoneg_advertised;

enum ixgbe_eeprom_type {
	ixgbe_eeprom_uninitialized = 0,
	ixgbe_eeprom_spi,
	ixgbe_flash,
	ixgbe_eeprom_none /* No NVM support */
};

enum ixgbe_mac_type {
	ixgbe_mac_unknown = 0,
	ixgbe_mac_82598EB,
	ixgbe_mac_82599EB,
	ixgbe_mac_X540,
	ixgbe_mac_X550,
	ixgbe_mac_X550EM_x,
	ixgbe_num_macs
};

enum ixgbe_phy_type {
	ixgbe_phy_unknown = 0,
	ixgbe_phy_none,
	ixgbe_phy_tn,
	ixgbe_phy_aq,
	ixgbe_phy_cu_unknown,
	ixgbe_phy_qt,
	ixgbe_phy_xaui,
	ixgbe_phy_nl,
	ixgbe_phy_sfp_passive_tyco,
	ixgbe_phy_sfp_passive_unknown,
	ixgbe_phy_sfp_active_unknown,
	ixgbe_phy_sfp_avago,
	ixgbe_phy_sfp_ftl,
	ixgbe_phy_sfp_ftl_active,
	ixgbe_phy_sfp_unknown,
	ixgbe_phy_sfp_intel,
	ixgbe_phy_qsfp_passive_unknown,
	ixgbe_phy_qsfp_active_unknown,
	ixgbe_phy_qsfp_intel,
	ixgbe_phy_qsfp_unknown,
	ixgbe_phy_sfp_unsupported,
	ixgbe_phy_generic
};

/*
 * SFP+ module type IDs:
 *
 * ID   Module Type
 * =============
 * 0    SFP_DA_CU
 * 1    SFP_SR
 * 2    SFP_LR
 * 3    SFP_DA_CU_CORE0 - 82599-specific
 * 4    SFP_DA_CU_CORE1 - 82599-specific
 * 5    SFP_SR/LR_CORE0 - 82599-specific
 * 6    SFP_SR/LR_CORE1 - 82599-specific
 */
enum ixgbe_sfp_type {
	ixgbe_sfp_type_da_cu = 0,
	ixgbe_sfp_type_sr = 1,
	ixgbe_sfp_type_lr = 2,
	ixgbe_sfp_type_da_cu_core0 = 3,
	ixgbe_sfp_type_da_cu_core1 = 4,
	ixgbe_sfp_type_srlr_core0 = 5,
	ixgbe_sfp_type_srlr_core1 = 6,
	ixgbe_sfp_type_da_act_lmt_core0 = 7,
	ixgbe_sfp_type_da_act_lmt_core1 = 8,
	ixgbe_sfp_type_1g_cu_core0 = 9,
	ixgbe_sfp_type_1g_cu_core1 = 10,
	ixgbe_sfp_type_1g_sx_core0 = 11,
	ixgbe_sfp_type_1g_sx_core1 = 12,
	ixgbe_sfp_type_1g_lx_core0 = 13,
	ixgbe_sfp_type_1g_lx_core1 = 14,
	ixgbe_sfp_type_not_present = 0xFFFE,
	ixgbe_sfp_type_unknown = 0xFFFF
};

enum ixgbe_media_type {
	ixgbe_media_type_unknown = 0,
	ixgbe_media_type_fiber,
	ixgbe_media_type_fiber_fixed,
	ixgbe_media_type_fiber_qsfp,
	ixgbe_media_type_fiber_lco,
	ixgbe_media_type_copper,
	ixgbe_media_type_backplane,
	ixgbe_media_type_cx4,
	ixgbe_media_type_virtual
};

/* Smart Speed Settings */
#define IXGBE_SMARTSPEED_MAX_RETRIES	3
enum ixgbe_smart_speed {
	ixgbe_smart_speed_auto = 0,
	ixgbe_smart_speed_on,
	ixgbe_smart_speed_off
};

// Other Structs
struct ixgbe_eeprom_info {
	enum ixgbe_eeprom_type type;
	u32 semaphore_delay;
	u16 word_size;
	u16 address_bits;
	u16 word_page_size;
	u16 ctrl_word_3;
};

#define ETH_ALEN 6
struct ixgbe_mac_info {
    //struct ixgbe_mac_operations     ops;
    enum ixgbe_mac_type             type;
    u8                              addr[ETH_ALEN];
    u8                              perm_addr[ETH_ALEN];
    u8                              san_addr[ETH_ALEN];
    /* prefix for World Wide Node Name (WWNN) */
    u16                             wwnn_prefix;
    /* prefix for World Wide Port Name (WWPN) */
    u16                             wwpn_prefix;
    u16				max_msix_vectors;
#define IXGBE_MAX_MTA			128
    u32				mta_shadow[IXGBE_MAX_MTA];
    s32                             mc_filter_type;
    u32                             mcft_size;
    u32                             vft_size;
    u32                             num_rar_entries;
    u32                             rar_highwater;
    u32				rx_pb_size;
    u32                             max_tx_queues;
    u32                             max_rx_queues;
    u32                             orig_autoc;
    u32                             cached_autoc;
    u32                             orig_autoc2;
    bool                            orig_link_settings_stored;
    bool                            autotry_restart;
    u8                              flags;
    u8				san_mac_rar_index;
    //struct ixgbe_thermal_sensor_data  thermal_sensor_data;
};

struct mdio_if_info {
    int prtad;
    u32 mmds;
};

struct ixgbe_phy_info {
//	struct ixgbe_phy_operations     ops;
    struct mdio_if_info		mdio;
	enum ixgbe_phy_type             type;
	u32                             id;
	enum ixgbe_sfp_type             sfp_type;
	bool                            sfp_setup_needed;
	u32                             revision;
	enum ixgbe_media_type           media_type;
	bool                            reset_disable;
	ixgbe_autoneg_advertised        autoneg_advertised;
	enum ixgbe_smart_speed          smart_speed;
	bool                            smart_speed_active;
	bool                            multispeed_fiber;
	bool                            reset_if_overtemp;
	bool                            qsfp_shared_i2c_bus;
};

struct ixgbe_addr_filter_info {
	u32 num_mc_addrs;
	u32 rar_used_count;
	u32 mta_in_use;
	u32 overflow_promisc;
	bool uc_set_promisc;
	bool user_set_promisc;
};

struct ixgbe_hw {
    u8			*hw_addr;
    void				*back;
//	struct ixgbe_mac_info		mac;
    struct ixgbe_addr_filter_info	addr_ctrl;
//	struct ixgbe_fc_info		fc;
//	struct ixgbe_phy_info		phy;
//	struct ixgbe_eeprom_info	eeprom;
//	struct ixgbe_bus_info		bus;
//	struct ixgbe_mbx_info		mbx;
    u16				device_id;
    u16				vendor_id;
    u16				subsystem_device_id;
    u16				subsystem_vendor_id;
    u8				revision_id;
    bool				adapter_stopped;
    bool				force_full_reset;
    bool				allow_unsupported_sfp;
    bool				mng_fw_enabled;
    bool				wol_enabled;
};

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

struct VirtioNetHeader {
    static const constexpr uint8_t kNeedsCsum = 1;
    static const constexpr uint8_t kGsoNone = 0;
    static const constexpr uint8_t kGsoTcpv4 = 1;
    static const constexpr uint8_t kGsoUdp = 3;
    static const constexpr uint8_t kGsoTcpv6 = 4;
    static const constexpr uint8_t kGsoEvn = 0x80;

    uint8_t flags;
    uint8_t gso_type;
    uint16_t hdr_len;
    uint16_t gso_size;
    uint16_t csum_start;
    uint16_t csum_offset;
    uint16_t num_buffers;
};

#endif  // BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_H_
