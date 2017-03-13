//          Copyright Boston University SESA Group 2013 - 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#ifndef BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_DRIVER_H_
#define BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_DRIVER_H_

#include "../Align.h"
#include "Debug.h"
#include "Fls.h"
#include "Ixgbe.h"
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
	dev.GetPcieDeviceInfo();
      IxgbeDriver::Create(dev);
      return true;
    }
    return false;
  }

  static void Create(pci::Device& dev);

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
  static const constexpr uint32_t NTXDESCS = 128;
  static const constexpr uint32_t NRXDESCS = 128;
  static const constexpr uint32_t RXBUFSZ = 2048;

  explicit IxgbeDriver(pci::Device& dev) : dev_(dev), bar0_(dev.GetBar(0)) {
    dev_.SetBusMaster(true);
    ebbrt::kprintf("%s constructor\n", __FUNCTION__);
  }

 private:
  void InitStruct();
  void DeviceInfo();
  void Init();
  void PhyInit();
  void StopDevice();
  void GlobalReset();
  void SetupQueue(uint32_t i);
  void ixgbe_probe(pci::Device& dev);
  s32 ixgbe_set_mac_type(struct ixgbe_hw* hw);
  s32 ixgbe_init_ops_82599(struct ixgbe_hw* hw);
  int ixgbe_sw_init(struct ixgbe_hw* hw, pci::Device& dev);
  s32 ixgbe_get_link_capabilities_82599(struct ixgbe_hw* hw,
                                        ixgbe_link_speed* speed, bool* autoneg);
  enum ixgbe_media_type ixgbe_get_media_type_82599(struct ixgbe_hw* hw);
  void ixgbe_disable_tx_laser_multispeed_fiber(struct ixgbe_hw* hw);
  void ixgbe_enable_tx_laser_multispeed_fiber(struct ixgbe_hw* hw);
  void ixgbe_flap_tx_laser_multispeed_fiber(struct ixgbe_hw* hw);
  void ixgbe_set_hard_rate_select_speed(struct ixgbe_hw* hw,
                                        ixgbe_link_speed speed);
  s32 ixgbe_setup_mac_link_smartspeed(struct ixgbe_hw* hw,
                                      ixgbe_link_speed speed,
                                      bool autoneg_wait_to_complete);
  s32 ixgbe_start_mac_link_82599(struct ixgbe_hw* hw,
                                 bool autoneg_wait_to_complete);
  s32 ixgbe_setup_mac_link_82599(struct ixgbe_hw* hw, ixgbe_link_speed speed,
                                 bool autoneg_wait_to_complete);
  s32 ixgbe_setup_sfp_modules_82599(struct ixgbe_hw* hw);
  void ixgbe_init_mac_link_ops_82599(struct ixgbe_hw* hw);
  s32 ixgbe_reset_hw_82599(struct ixgbe_hw* hw);
  s32 ixgbe_read_analog_reg8_82599(struct ixgbe_hw* hw, u32 reg, u8* val);
  s32 ixgbe_write_analog_reg8_82599(struct ixgbe_hw* hw, u32 reg, u8 val);
  s32 ixgbe_start_hw_82599(struct ixgbe_hw* hw);
  s32 ixgbe_identify_phy_82599(struct ixgbe_hw* hw);
  s32 ixgbe_init_phy_ops_82599(struct ixgbe_hw* hw);
  u32 ixgbe_get_supported_physical_layer_82599(struct ixgbe_hw* hw);
  s32 ixgbe_enable_rx_dma_82599(struct ixgbe_hw* hw, u32 regval);
  s32 prot_autoc_read_82599(struct ixgbe_hw* hw, bool* locked, u32* reg_val);
  s32 prot_autoc_write_82599(struct ixgbe_hw* hw, u32 reg_val, bool locked);
  s32 ixgbe_init_phy_ops_generic(struct ixgbe_hw* hw);
  s32 ixgbe_init_ops_generic(struct ixgbe_hw* hw);
  bool ixgbe_mng_enabled(struct ixgbe_hw* hw);
  u32 ixgbe_read_reg(struct ixgbe_hw* hw, u32 reg, bool quiet);
  void IXGBE_WRITE_REG(struct ixgbe_hw* hw, u32 reg, u32 value);
  s32 ixgbe_stop_adapter_generic(struct ixgbe_hw* hw);
  void ixgbe_disable_rx_generic(struct ixgbe_hw* hw);
  s32 ixgbe_disable_pcie_master(struct ixgbe_hw* hw);
  void ixgbe_clear_tx_pending(struct ixgbe_hw* hw);
  s32 ixgbe_identify_phy_generic(struct ixgbe_hw* hw);
  bool ixgbe_probe_phy(struct ixgbe_hw* hw, u16 phy_addr);
  bool ixgbe_validate_phy_addr(struct ixgbe_hw* hw, u32 phy_addr);
  s32 ixgbe_read_phy_reg_generic(struct ixgbe_hw* hw, u32 reg_addr,
                                 u32 device_type, u16* phy_data);
  s32 ixgbe_read_phy_reg_mdi(struct ixgbe_hw* hw, u32 reg_addr, u32 device_type,
                             u16* phy_data);
  s32 ixgbe_get_phy_id(struct ixgbe_hw* hw);
  enum ixgbe_phy_type ixgbe_get_phy_type_from_id(u32 phy_id);
  s32 ixgbe_identify_module_generic(struct ixgbe_hw* hw);
  s32 ixgbe_identify_sfp_module_generic(struct ixgbe_hw* hw);
  s32 ixgbe_read_i2c_eeprom_generic(struct ixgbe_hw* hw, u8 byte_offset,
                                    u8* eeprom_data);
  s32 ixgbe_read_i2c_byte_generic(struct ixgbe_hw* hw, u8 byte_offset,
                                  u8 dev_addr, u8* data);
  s32 ixgbe_read_i2c_byte_generic_int(struct ixgbe_hw* hw, u8 byte_offset,
                                      u8 dev_addr, u8* data, bool lock);
  void ixgbe_i2c_bus_clear(struct ixgbe_hw* hw);
  void ixgbe_i2c_stop(struct ixgbe_hw* hw);
  s32 ixgbe_clock_in_i2c_byte(struct ixgbe_hw* hw, u8* data);
  s32 ixgbe_clock_in_i2c_bit(struct ixgbe_hw* hw, bool* data);
  s32 ixgbe_get_i2c_ack(struct ixgbe_hw* hw);
  s32 ixgbe_clock_out_i2c_byte(struct ixgbe_hw* hw, u8 data);
  s32 ixgbe_clock_out_i2c_bit(struct ixgbe_hw* hw, bool data);
  void ixgbe_i2c_start(struct ixgbe_hw* hw);
  void ixgbe_lower_i2c_clk(struct ixgbe_hw* hw, u32* i2cctl);
  s32 ixgbe_set_i2c_data(struct ixgbe_hw* hw, u32* i2cctl, bool data);
  void ixgbe_raise_i2c_clk(struct ixgbe_hw* hw, u32* i2cctl);
  bool ixgbe_get_i2c_data(struct ixgbe_hw* hw, u32* i2cctl);
  bool ixgbe_is_sfp_probe(struct ixgbe_hw* hw, u8 offset, u8 addr);
  void ixgbe_set_lan_id_multi_port_pcie(struct ixgbe_hw* hw);
  s32 ixgbe_get_device_caps_generic(struct ixgbe_hw* hw, u16* device_caps);
  s32 ixgbe_read_eeprom_82599(struct ixgbe_hw* hw, u16 offset, u16* data);
  s32 ixgbe_read_eerd_generic(struct ixgbe_hw* hw, u16 offset, u16* data);
  s32 ixgbe_read_eerd_buffer_generic(struct ixgbe_hw* hw, u16 offset, u16 words,
                                     u16* data);
  s32 ixgbe_poll_eerd_eewr_done(struct ixgbe_hw* hw, u32 ee_reg);
  s32 ixgbe_init_eeprom_params_generic(struct ixgbe_hw* hw);

  void ixgbe_release_eeprom_semaphore(struct ixgbe_hw* hw);
  s32 ixgbe_get_eeprom_semaphore(struct ixgbe_hw* hw);
  void ixgbe_release_swfw_sync(struct ixgbe_hw* hw, u32 mask);
  s32 ixgbe_acquire_swfw_sync(struct ixgbe_hw* hw, u32 mask);

  s32 ixgbe_get_mac_addr_generic(struct ixgbe_hw* hw, u8* mac_addr);
  s32 ixgbe_reset_pipeline_82599(struct ixgbe_hw* hw);
  bool ixgbe_verify_lesm_fw_enabled_82599(struct ixgbe_hw* hw);
  s32 ixgbe_check_reset_blocked(struct ixgbe_hw* hw);
  s32 ixgbe_check_mac_link_generic(struct ixgbe_hw* hw, ixgbe_link_speed* speed,
                                   bool* link_up,
                                   bool link_up_wait_to_complete);

  s32 ixgbe_init_rx_addrs_generic(struct ixgbe_hw* hw);
  s32 ixgbe_init_uta_tables_generic(struct ixgbe_hw* hw);
  s32 ixgbe_clear_vmdq_generic(struct ixgbe_hw* hw, u32 rar, u32 vmdq);
  s32 ixgbe_clear_rar_generic(struct ixgbe_hw* hw, u32 index);
  s32 ixgbe_set_rar_generic(struct ixgbe_hw* hw, u32 index, u8* addr, u32 vmdq,
                            u32 enable_addr);
  s32 ixgbe_set_vmdq_generic(struct ixgbe_hw* hw, u32 rar, u32 vmdq);
  s32 ixgbe_validate_mac_addr(u8* mac_addr);

  s32 ixgbe_get_san_mac_addr_generic(struct ixgbe_hw *hw, u8 *san_mac_addr);
  s32 ixgbe_get_san_mac_addr_offset(struct ixgbe_hw *hw,
				    u16 *san_mac_offset);
  s32 ixgbe_get_wwn_prefix_generic(struct ixgbe_hw *hw, u16 *wwnn_prefix,
				   u16 *wwpn_prefix);
  s32 ixgbe_get_sfp_init_sequence_offsets(struct ixgbe_hw *hw,
					u16 *list_offset,
					  u16 *data_offset);
  s32 ixgbe_read_eeprom_bit_bang_generic(struct ixgbe_hw *hw, u16 offset,
					 u16 *data);
  s32 ixgbe_read_eeprom_buffer_bit_bang(struct ixgbe_hw *hw, u16 offset,
					u16 words, u16 *data);
  void ixgbe_release_eeprom(struct ixgbe_hw *hw);
  s32 ixgbe_ready_eeprom(struct ixgbe_hw *hw);
  void ixgbe_standby_eeprom(struct ixgbe_hw *hw);
  u16 ixgbe_shift_in_eeprom_bits(struct ixgbe_hw *hw, u16 count);
  void ixgbe_shift_out_eeprom_bits(struct ixgbe_hw *hw, u16 data,
				   u16 count);
  void ixgbe_lower_eeprom_clk(struct ixgbe_hw *hw, u32 *eec);
  void ixgbe_raise_eeprom_clk(struct ixgbe_hw *hw, u32 *eec);
  s32 ixgbe_acquire_eeprom(struct ixgbe_hw *hw);
  
  s32 ixgbe_start_hw_generic(struct ixgbe_hw *hw);
  s32 ixgbe_setup_fc_generic(struct ixgbe_hw *hw);
  s32 ixgbe_clear_hw_cntrs_generic(struct ixgbe_hw *hw);
  s32 ixgbe_clear_vfta_generic(struct ixgbe_hw *hw);
  s32 ixgbe_start_hw_gen2(struct ixgbe_hw *hw);

  s32 ixgbe_setup_mac_link_multispeed_fiber(struct ixgbe_hw *hw,
					  ixgbe_link_speed speed,
					    bool autoneg_wait_to_complete);
					
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
  e10k_queue_t* ixgq;

  void* rxbuf;

};  // class IxgbeDriver
}  // namespace ebbrt

#endif  // BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_DRIVER_H_
