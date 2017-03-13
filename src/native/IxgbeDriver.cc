//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#include "IxgbeDriver.h"

#include "../StaticIOBuf.h"
#include "../UniqueIOBuf.h"
#include "Clock.h"
#include "Debug.h"
#include "EventManager.h"

#include <atomic>

#define IXGBE_READ_REG(h, r) ixgbe_read_reg(h, r, false)
#define IXGBE_R32_Q(h, r) ixgbe_read_reg(h, r, true)

#define IXGBE_READ_REG_ARRAY(a, reg, offset)                                   \
  (IXGBE_READ_REG((a), (reg) + ((offset) << 2)))

#define IXGBE_WRITE_FLUSH(a) IXGBE_READ_REG(a, IXGBE_STATUS)

void ebbrt::IxgbeDriver::Create(pci::Device& dev) {
  auto ixgbe_dev = new IxgbeDriver(dev);
  //ebbrt::clock::SleepMilli(200);
  //ixgbe_dev->ixgbe_probe(dev);
  ixgbe_dev->Init();
  ixgbe_dev->SetupQueue(0);
  ebbrt::clock::SleepMilli(200);
  ebbrt::kprintf("intel 82599 card initialzed\n");
  
  // Send test packet
  //ixgbe_dev->SendPacket(0);

  while (1) {
  // ebbrt::clock::SleepMilli(10000);
    // ebbrt::kprintf("Slept 10s: ");
    // ixgbe_dev->ReadGprc();
    ixgbe_dev->ProcessPacket(0);
  }
}

void ebbrt::IxgbeDriver::IXGBE_WRITE_REG(struct ixgbe_hw* hw, u32 reg,
                                         u32 value) {
  bar0_.Write32(reg, value);
}

u32 ebbrt::IxgbeDriver::ixgbe_read_reg(struct ixgbe_hw* hw, u32 reg,
                                       bool quiet) {
  u32 value;

  value = bar0_.Read32(reg);
  if (value == IXGBE_FAILED_READ_REG || value == IXGBE_DEAD_READ_REG) {
    ebbrt::kabort("%s failed\n", __PRETTY_FUNCTION__);
  }
  return value;
}

/**
 *  ixgbe_disable_pcie_master - Disable PCI-express master access
 *  @hw: pointer to hardware structure
 *
 *  Disables PCI-Express master access and verifies there are no pending
 *  requests. IXGBE_ERR_MASTER_REQUESTS_PENDING is returned if master disable
 *  bit hasn't caused the master requests to be disabled, else IXGBE_SUCCESS
 *  is returned signifying master requests disabled.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_disable_pcie_master(struct ixgbe_hw *hw)
{
	s32 status = IXGBE_SUCCESS;
	u32 i, poll;
	u16 value;

	DEBUGFUNC("ixgbe_disable_pcie_master\n");

	/* Always set this bit to ensure any future transactions are blocked */
	IXGBE_WRITE_REG(hw, IXGBE_CTRL, IXGBE_CTRL_GIO_DIS);
	
	//ebbrt::clock::SleepMilli(2);
	
	/* Exit if master requests are blocked */
	/*if (!(IXGBE_READ_REG(hw, IXGBE_STATUS) & IXGBE_STATUS_GIO))
	{
	    ebbrt::kabort("master requests blocked\n");
	    //IXGBE_REMOVED(hw->hw_addr))
	    goto out;
	    }*/

	/* Poll for master request bit to clear */
	for (i = 0; i < IXGBE_PCI_MASTER_DISABLE_TIMEOUT; i++) {
	    //ebbrt::clock::SleepMilli(1);
	    usec_delay(100);
	    if (!(IXGBE_READ_REG(hw, IXGBE_STATUS) & IXGBE_STATUS_GIO))
		goto out;
	}

	ebbrt::kabort("master request not cleared\n");
	/*
	 * Two consecutive resets are required via CTRL.RST per datasheet
	 * 5.2.5.3.2 Master Disable.  We set a flag to inform the reset routine
	 * of this need.  The first reset prevents new master requests from
	 * being issued by our device.  We then must wait 1usec or more for any
	 * remaining completions from the PCIe bus to trickle in, and then reset
	 * again to clear out any effects they may have had on our device.
	 */
	//DEBUGOUT("GIO Master Disable bit didn't clear - requesting resets\n");
	// hw->mac.flags |= IXGBE_FLAGS_DOUBLE_RESET_REQUIRED;

	// if (hw->mac.type >= ixgbe_mac_X550)
	// 	goto out;

	// /*
	//  * Before proceeding, make sure that the PCIe block does not have
	//  * transactions pending.
	//  */
	// poll = ixgbe_pcie_timeout_poll(hw);
	// for (i = 0; i < poll; i++) {
	//     //usec_delay(100);
	//     ebbrt::clock::SleepMilli(1);
	// 	value = IXGBE_READ_PCIE_WORD(hw, IXGBE_PCI_DEVICE_STATUS);
	// 	//if (IXGBE_REMOVED(hw->hw_addr))
	// 	//	goto out;
	// 	if (!(value & IXGBE_PCI_DEVICE_STATUS_TRANSACTION_PENDING))
	// 		goto out;
	// }

	// ERROR_REPORT1(IXGBE_ERROR_POLLING,
	// 	     "PCIe transaction pending bit also did not clear.\n");
	// status = IXGBE_ERR_MASTER_REQUESTS_PENDING;

out:
	return status;
}

/**
 * ixgbe_mng_enabled - Is the manageability engine enabled?
 * @hw: pointer to hardware structure
 *
 * Returns true if the manageability engine is enabled.
 **/
bool ebbrt::IxgbeDriver::ixgbe_mng_enabled(struct ixgbe_hw* hw) {
  u32 fwsm, manc, factps;
      DEBUGFUNC("%s\n", __PRETTY_FUNCTION__);
      
  fwsm = IXGBE_READ_REG(hw, IXGBE_FWSM_BY_MAC(hw));
  if ((fwsm & IXGBE_FWSM_MODE_MASK) != IXGBE_FWSM_FW_MODE_PT)
    return false;

  manc = IXGBE_READ_REG(hw, IXGBE_MANC);
  if (!(manc & IXGBE_MANC_RCV_TCO_EN))
    return false;

  if (hw->mac.type <= ixgbe_mac_X540) {
    factps = IXGBE_READ_REG(hw, IXGBE_FACTPS_BY_MAC(hw));
    if (factps & IXGBE_FACTPS_MNGCG)
      return false;
  }

  return true;
}

/**
 *  ixgbe_set_mac_type - Sets MAC type
 *  @hw: pointer to the HW structure
 *
 *  This function sets the mac type of the adapter based on the
 *  vendor ID and device ID stored in the hw structure.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_set_mac_type(struct ixgbe_hw* hw) {
  s32 ret_val = IXGBE_SUCCESS;

  DEBUGFUNC("ixgbe_set_mac_type\n");

  if (hw->vendor_id != IXGBE_INTEL_VENDOR_ID) {
    DEBUGOUT2("Unsupported vendor id: %x", hw->vendor_id);
    return IXGBE_ERR_DEVICE_NOT_SUPPORTED;
  }

  hw->mvals = ixgbe_mvals_base;

  switch (hw->device_id) {
  case IXGBE_DEV_ID_82598:
  case IXGBE_DEV_ID_82598_BX:
  case IXGBE_DEV_ID_82598AF_SINGLE_PORT:
  case IXGBE_DEV_ID_82598AF_DUAL_PORT:
  case IXGBE_DEV_ID_82598AT:
  case IXGBE_DEV_ID_82598AT2:
  case IXGBE_DEV_ID_82598EB_CX4:
  case IXGBE_DEV_ID_82598_CX4_DUAL_PORT:
  case IXGBE_DEV_ID_82598_DA_DUAL_PORT:
  case IXGBE_DEV_ID_82598_SR_DUAL_PORT_EM:
  case IXGBE_DEV_ID_82598EB_XF_LR:
  case IXGBE_DEV_ID_82598EB_SFP_LOM:
    hw->mac.type = ixgbe_mac_82598EB;
    break;
  case IXGBE_DEV_ID_82599_KX4:
  case IXGBE_DEV_ID_82599_KX4_MEZZ:
  case IXGBE_DEV_ID_82599_XAUI_LOM:
  case IXGBE_DEV_ID_82599_COMBO_BACKPLANE:
  case IXGBE_DEV_ID_82599_KR:
  case IXGBE_DEV_ID_82599_SFP:
  case IXGBE_DEV_ID_82599_BACKPLANE_FCOE:
  case IXGBE_DEV_ID_82599_SFP_FCOE:
  case IXGBE_DEV_ID_82599_SFP_EM:
  case IXGBE_DEV_ID_82599_SFP_SF2:
  case IXGBE_DEV_ID_82599_SFP_SF_QP:
  case IXGBE_DEV_ID_82599_QSFP_SF_QP:
  case IXGBE_DEV_ID_82599EN_SFP:
  case IXGBE_DEV_ID_82599_CX4:
  case IXGBE_DEV_ID_82599_LS:
  case IXGBE_DEV_ID_82599_T3_LOM:
    hw->mac.type = ixgbe_mac_82599EB;
    break;
  default:
    ret_val = IXGBE_ERR_DEVICE_NOT_SUPPORTED;
    DEBUGOUT2("Unsupported device id: %x", hw->device_id);
    break;
  }

  DEBUGOUT2("ixgbe_set_mac_type found mac: %d, returns: %d\n", hw->mac.type,
            ret_val);
  return ret_val;
}

/**
 *  ixgbe_get_media_type_82599 - Get media type
 *  @hw: pointer to hardware structure
 *
 *  Returns the media type (fiber, copper, backplane)
 **/
enum ixgbe_media_type
ebbrt::IxgbeDriver::ixgbe_get_media_type_82599(struct ixgbe_hw* hw) {
  enum ixgbe_media_type media_type;

  DEBUGFUNC("ixgbe_get_media_type_82599\n");

  /* Detect if there is a copper PHY attached. */
  switch (hw->phy.type) {
  case ixgbe_phy_cu_unknown:
  case ixgbe_phy_tn:
    media_type = ixgbe_media_type_copper;
    goto out;
  default:
    break;
  }

  switch (hw->device_id) {
  case IXGBE_DEV_ID_82599_KX4:
  case IXGBE_DEV_ID_82599_KX4_MEZZ:
  case IXGBE_DEV_ID_82599_COMBO_BACKPLANE:
  case IXGBE_DEV_ID_82599_KR:
  case IXGBE_DEV_ID_82599_BACKPLANE_FCOE:
  case IXGBE_DEV_ID_82599_XAUI_LOM:
    /* Default device ID is mezzanine card KX/KX4 */
    media_type = ixgbe_media_type_backplane;
    break;
  case IXGBE_DEV_ID_82599_SFP:
  case IXGBE_DEV_ID_82599_SFP_FCOE:
  case IXGBE_DEV_ID_82599_SFP_EM:
  case IXGBE_DEV_ID_82599_SFP_SF2:
  case IXGBE_DEV_ID_82599_SFP_SF_QP:
  case IXGBE_DEV_ID_82599EN_SFP:
    media_type = ixgbe_media_type_fiber;
    break;
  case IXGBE_DEV_ID_82599_CX4:
    media_type = ixgbe_media_type_cx4;
    break;
  case IXGBE_DEV_ID_82599_T3_LOM:
    media_type = ixgbe_media_type_copper;
    break;
  case IXGBE_DEV_ID_82599_LS:
    media_type = ixgbe_media_type_fiber_lco;
    break;
  case IXGBE_DEV_ID_82599_QSFP_SF_QP:
    media_type = ixgbe_media_type_fiber_qsfp;
    break;
  default:
    media_type = ixgbe_media_type_unknown;
    break;
  }
out:
  //ebbrt::kprintf("media type = %d\n", media_type);
  return media_type;
}


void ebbrt::IxgbeDriver::ixgbe_init_mac_link_ops_82599(struct ixgbe_hw* hw) {
  struct ixgbe_mac_info* mac = &hw->mac;

  DEBUGFUNC("ixgbe_init_mac_link_ops_82599\n");

  /*
   * enable the laser control functions for SFP+ fiber
   * and MNG not enabled
   */
  // if ((mac->ops.get_media_type(hw) == ixgbe_media_type_fiber) &&
  if ((ixgbe_get_media_type_82599(hw) == ixgbe_media_type_fiber) &&
      !ixgbe_mng_enabled(hw)) {
      ebbrt::kprintf("ixgbe_media_type_fiber\n");
    // mac->ops.disable_tx_laser =
    //	ixgbe_disable_tx_laser_multispeed_fiber;
    /*mac->ops.enable_tx_laser =
        ixgbe_enable_tx_laser_multispeed_fiber;
        mac->ops.flap_tx_laser = ixgbe_flap_tx_laser_multispeed_fiber;*/

  } else {
    mac->ops.disable_tx_laser = NULL;
    mac->ops.enable_tx_laser = NULL;
    mac->ops.flap_tx_laser = NULL;
  }

  if (hw->phy.multispeed_fiber) {
    /* Set up dual speed SFP+ support */
    // mac->ops.setup_link = ixgbe_setup_mac_link_multispeed_fiber;
    // mac->ops.setup_mac_link = ixgbe_setup_mac_link_82599;
    // mac->ops.set_rate_select_speed =
    //	ixgbe_set_hard_rate_select_speed;
  }
  // else {
  // 	if ((ixgbe_get_media_type(hw) == ixgbe_media_type_backplane) &&
  // 	     (hw->phy.smart_speed == ixgbe_smart_speed_auto ||
  // 	      hw->phy.smart_speed == ixgbe_smart_speed_on) &&
  // 	      !ixgbe_verify_lesm_fw_enabled_82599(hw)) {
  // 		mac->ops.setup_link = ixgbe_setup_mac_link_smartspeed;
  // 	} else {
  // 		mac->ops.setup_link = ixgbe_setup_mac_link_82599;
  // 	}
  // }
}

/**
 *  ixgbe_init_ops_generic - Inits function ptrs
 *  @hw: pointer to the hardware structure
 *
 *  Initialize the function pointers.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_init_ops_generic(struct ixgbe_hw* hw) {
  struct ixgbe_eeprom_info* eeprom = &hw->eeprom;
  struct ixgbe_mac_info* mac = &hw->mac;
  u32 eec = IXGBE_READ_REG(hw, IXGBE_EEC_BY_MAC(hw));

  DEBUGFUNC("ixgbe_init_ops_generic\n");

  // /* EEPROM */
  // eeprom->ops.init_params = ixgbe_init_eeprom_params_generic;
  /* If EEPROM is valid (bit 8 = 1), use EERD otherwise use bit bang */
  if (eec & IXGBE_EEC_PRES) {
    ebbrt::kprintf("use EERD\n");
    //eeprom->ops.read = ixgbe_read_eerd_generic;
    //	eeprom->ops.read_buffer = ixgbe_read_eerd_buffer_generic;
  } else {
    ebbrt::kprintf("use EERD bit bang\n");
    //	eeprom->ops.read = ixgbe_read_eeprom_bit_bang_generic;
    //	eeprom->ops.read_buffer =
    //			 ixgbe_read_eeprom_buffer_bit_bang_generic;
  }
  // eeprom->ops.write = ixgbe_write_eeprom_generic;
  // eeprom->ops.write_buffer = ixgbe_write_eeprom_buffer_bit_bang_generic;
  // eeprom->ops.validate_checksum =
  // 			      ixgbe_validate_eeprom_checksum_generic;
  // eeprom->ops.update_checksum = ixgbe_update_eeprom_checksum_generic;
  // eeprom->ops.calc_checksum = ixgbe_calc_eeprom_checksum_generic;

  // /* MAC */
  // mac->ops.init_hw = ixgbe_init_hw_generic;
  //mac->ops.reset_hw = NULL;
  // mac->ops.start_hw = ixgbe_start_hw_generic;
  // mac->ops.clear_hw_cntrs = ixgbe_clear_hw_cntrs_generic;
     mac->ops.get_media_type = NULL;
     mac->ops.get_supported_physical_layer = NULL;
  // mac->ops.enable_rx_dma = ixgbe_enable_rx_dma_generic;
  // mac->ops.get_mac_addr = ixgbe_get_mac_addr_generic;
  // mac->ops.stop_adapter = ixgbe_stop_adapter_generic;
  // mac->ops.get_bus_info = ixgbe_get_bus_info_generic;
  // mac->ops.set_lan_id = ixgbe_set_lan_id_multi_port_pcie;
  // mac->ops.acquire_swfw_sync = ixgbe_acquire_swfw_sync;
  // mac->ops.release_swfw_sync = ixgbe_release_swfw_sync;
  // mac->ops.prot_autoc_read = prot_autoc_read_generic;
  // mac->ops.prot_autoc_write = prot_autoc_write_generic;

  // /* LEDs */
  // mac->ops.led_on = ixgbe_led_on_generic;
  // mac->ops.led_off = ixgbe_led_off_generic;
  // mac->ops.blink_led_start = ixgbe_blink_led_start_generic;
  // mac->ops.blink_led_stop = ixgbe_blink_led_stop_generic;

  // /* RAR, Multicast, VLAN */
  // mac->ops.set_rar = ixgbe_set_rar_generic;
  // mac->ops.clear_rar = ixgbe_clear_rar_generic;
   mac->ops.insert_mac_addr = NULL;
   mac->ops.set_vmdq = NULL;
   mac->ops.clear_vmdq = NULL;
  // mac->ops.init_rx_addrs = ixgbe_init_rx_addrs_generic;
  // mac->ops.update_uc_addr_list = ixgbe_update_uc_addr_list_generic;
  // mac->ops.update_mc_addr_list = ixgbe_update_mc_addr_list_generic;
  // mac->ops.enable_mc = ixgbe_enable_mc_generic;
  // mac->ops.disable_mc = ixgbe_disable_mc_generic;
   mac->ops.clear_vfta = NULL;
   mac->ops.set_vfta = NULL;
   mac->ops.set_vlvf = NULL;
   mac->ops.init_uta_tables = NULL;
  // mac->ops.enable_rx = ixgbe_enable_rx_generic;
  // mac->ops.disable_rx = ixgbe_disable_rx_generic;

  // /* Flow Control */
  // mac->ops.fc_enable = ixgbe_fc_enable_generic;
  // mac->ops.setup_fc = ixgbe_setup_fc_generic;
  // mac->ops.fc_autoneg = ixgbe_fc_autoneg;

  // /* Link */
   mac->ops.get_link_capabilities = NULL;
   mac->ops.setup_link = NULL;
   mac->ops.check_link = NULL;
   mac->ops.dmac_config = NULL;
   mac->ops.dmac_update_tcs = NULL;
   mac->ops.dmac_config_tcs = NULL;

  return IXGBE_SUCCESS;
}

/**
 *  ixgbe_init_phy_ops_generic - Inits PHY function ptrs
 *  @hw: pointer to the hardware structure
 *
 *  Initialize the function pointers.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_init_phy_ops_generic(struct ixgbe_hw* hw) {
  struct ixgbe_phy_info* phy = &hw->phy;

  DEBUGFUNC("ixgbe_init_phy_ops_generic\n");

  /* PHY */
     // 	phy->ops.identify = ixgbe_identify_phy_generic;
     //      phy->ops.reset = ixgbe_reset_phy_generic;
     //      phy->ops.read_reg = ixgbe_read_phy_reg_generic;
     //      phy->ops.write_reg = ixgbe_write_phy_reg_generic;
     //      phy->ops.read_reg_mdi = ixgbe_read_phy_reg_mdi;
     //      phy->ops.write_reg_mdi = ixgbe_write_phy_reg_mdi;
     //      phy->ops.setup_link = ixgbe_setup_phy_link_generic;
     //      phy->ops.setup_link_speed = ixgbe_setup_phy_link_speed_generic;
           phy->ops.check_link = NULL;
     //      phy->ops.get_firmware_version =
     // ixgbe_get_phy_firmware_version_generic;
     //      phy->ops.read_i2c_byte = ixgbe_read_i2c_byte_generic;
     //      phy->ops.write_i2c_byte = ixgbe_write_i2c_byte_generic;
     //      phy->ops.read_i2c_sff8472 = ixgbe_read_i2c_sff8472_generic;
     //      phy->ops.read_i2c_eeprom = ixgbe_read_i2c_eeprom_generic;
     //      phy->ops.write_i2c_eeprom = ixgbe_write_i2c_eeprom_generic;
     //      phy->ops.i2c_bus_clear = ixgbe_i2c_bus_clear;
     //      phy->ops.identify_sfp = ixgbe_identify_module_generic;
           phy->sfp_type = ixgbe_sfp_type_unknown;
     //      phy->ops.read_i2c_byte_unlocked =
     // ixgbe_read_i2c_byte_generic_unlocked;
     //      phy->ops.write_i2c_byte_unlocked =
     //                              ixgbe_write_i2c_byte_generic_unlocked;
     //      phy->ops.check_overtemp = ixgbe_tn_check_overtemp;
  return IXGBE_SUCCESS;
}

/**
 *  ixgbe_init_ops_82599 - Inits func ptrs and MAC type
 *  @hw: pointer to hardware structure
 *
 *  Initialize the function pointers and assign the MAC type for 82599.
 *  Does not touch the hardware.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_init_ops_82599(struct ixgbe_hw* hw) {
  struct ixgbe_mac_info* mac = &hw->mac;
  struct ixgbe_phy_info* phy = &hw->phy;
  struct ixgbe_eeprom_info* eeprom = &hw->eeprom;
  s32 ret_val;

  DEBUGFUNC("ixgbe_init_ops_82599\n");

  ixgbe_init_phy_ops_generic(hw);
  ret_val = ixgbe_init_ops_generic(hw);

  // /* PHY */
  // phy->ops.identify = ixgbe_identify_phy_82599;
  // phy->ops.init = ixgbe_init_phy_ops_82599;

  // /* MAC */
  // mac->ops.reset_hw = ixgbe_reset_hw_82599;
  // mac->ops.get_media_type = ixgbe_get_media_type_82599;
  // mac->ops.get_supported_physical_layer =
  // 			    ixgbe_get_supported_physical_layer_82599;
  // mac->ops.disable_sec_rx_path = ixgbe_disable_sec_rx_path_generic;
  // mac->ops.enable_sec_rx_path = ixgbe_enable_sec_rx_path_generic;
  // mac->ops.enable_rx_dma = ixgbe_enable_rx_dma_82599;
  // mac->ops.read_analog_reg8 = ixgbe_read_analog_reg8_82599;
  // mac->ops.write_analog_reg8 = ixgbe_write_analog_reg8_82599;
  // mac->ops.start_hw = ixgbe_start_hw_82599;
  // mac->ops.get_san_mac_addr = ixgbe_get_san_mac_addr_generic;
  // mac->ops.set_san_mac_addr = ixgbe_set_san_mac_addr_generic;
  // mac->ops.get_device_caps = ixgbe_get_device_caps_generic;
  // mac->ops.get_wwn_prefix = ixgbe_get_wwn_prefix_generic;
  // mac->ops.get_fcoe_boot_status = ixgbe_get_fcoe_boot_status_generic;
  // mac->ops.prot_autoc_read = prot_autoc_read_82599;
  // mac->ops.prot_autoc_write = prot_autoc_write_82599;

  // /* RAR, Multicast, VLAN */
  // mac->ops.set_vmdq = ixgbe_set_vmdq_generic;
  // mac->ops.set_vmdq_san_mac = ixgbe_set_vmdq_san_mac_generic;
  // mac->ops.clear_vmdq = ixgbe_clear_vmdq_generic;
  // mac->ops.insert_mac_addr = ixgbe_insert_mac_addr_generic;
  mac->rar_highwater = 1;
  // mac->ops.set_vfta = ixgbe_set_vfta_generic;
  // mac->ops.set_vlvf = ixgbe_set_vlvf_generic;
  // mac->ops.clear_vfta = ixgbe_clear_vfta_generic;
  // mac->ops.init_uta_tables = ixgbe_init_uta_tables_generic;
  // mac->ops.setup_sfp = ixgbe_setup_sfp_modules_82599;
  // mac->ops.set_mac_anti_spoofing = ixgbe_set_mac_anti_spoofing;
  // mac->ops.set_vlan_anti_spoofing = ixgbe_set_vlan_anti_spoofing;

  // /* Link */
  // mac->ops.get_link_capabilities = ixgbe_get_link_capabilities_82599;
  // mac->ops.check_link = ixgbe_check_mac_link_generic;
  // mac->ops.setup_rxpba = ixgbe_set_rxpba_generic;
  ixgbe_init_mac_link_ops_82599(hw);

   mac->mcft_size		= IXGBE_82599_MC_TBL_SIZE;
   mac->vft_size		= IXGBE_82599_VFT_TBL_SIZE;
   mac->num_rar_entries	= IXGBE_82599_RAR_ENTRIES;
   mac->rx_pb_size		= IXGBE_82599_RX_PB_SIZE;
   mac->max_rx_queues	= IXGBE_82599_MAX_RX_QUEUES;
   mac->max_tx_queues	= IXGBE_82599_MAX_TX_QUEUES;
  // mac->max_msix_vectors	= ixgbe_get_pcie_msix_count_generic(hw);

  // mac->arc_subsystem_valid = !!(IXGBE_READ_REG(hw, IXGBE_FWSM_BY_MAC(hw))
  // 			      & IXGBE_FWSM_MODE_MASK);

  // hw->mbx.ops.init_params = ixgbe_init_mbx_params_pf;

  // /* EEPROM */
  // eeprom->ops.read = ixgbe_read_eeprom_82599;
  // eeprom->ops.read_buffer = ixgbe_read_eeprom_buffer_82599;

  // /* Manageability interface */
  // mac->ops.set_fw_drv_ver = ixgbe_set_fw_drv_ver_generic;

  // mac->ops.get_thermal_sensor_data =
  // 				 ixgbe_get_thermal_sensor_data_generic;
  // mac->ops.init_thermal_sensor_thresh =
  // 			      ixgbe_init_thermal_sensor_thresh_generic;

  // mac->ops.get_rtrup2tc = ixgbe_dcb_get_rtrup2tc_generic;

  return ret_val;
}

/**
 * ixgbe_sw_init - Initialize general software structures (struct ixgbe_adapter)
 * @adapter: board private structure to initialize
 *
 * ixgbe_sw_init initializes the Adapter private data structure.
 * Fields are initialized based on PCI device information and
 * OS network device settings (MTU size).
 **/
int ebbrt::IxgbeDriver::ixgbe_sw_init(struct ixgbe_hw* hw, pci::Device& dev) {
  int err;
  unsigned int fdir;
  u32 fwsm;
  u16 device_caps;
      DEBUGFUNC("%s\n", __PRETTY_FUNCTION__);

  hw->revision_id = dev.GetVendorId();
  if (hw->revision_id == IXGBE_FAILED_READ_CFG_BYTE) {
    err = -1;
    goto out;
  }

  hw->subsystem_vendor_id = dev.GetSubsystemVendorId();
  hw->subsystem_device_id = dev.GetSubsystemDeviceId();
  ebbrt::kprintf(
      "revision_id = %x, subsystem_vendor_id = %x, subsystem_device_id = %x\n",
      hw->revision_id, hw->subsystem_vendor_id, hw->subsystem_device_id);

  // ixgbe_init_shared_code
  err = ixgbe_init_ops_82599(hw);
  hw->mac.max_link_up_time = IXGBE_LINK_UP_TIME;
  if (err) {
    DEBUGFUNC("init_ops failed: %d\n", err);
    goto out;
  }

  
out:
  return err;
}

void ebbrt::IxgbeDriver::ixgbe_disable_rx_generic(struct ixgbe_hw *hw)
{
    DEBUGFUNC("%s\n", __PRETTY_FUNCTION__);
    
	u32 pfdtxgswc;
	u32 rxctrl;

	rxctrl = IXGBE_READ_REG(hw, IXGBE_RXCTRL);
	if (rxctrl & IXGBE_RXCTRL_RXEN) {
		if (hw->mac.type != ixgbe_mac_82598EB) {
			pfdtxgswc = IXGBE_READ_REG(hw, IXGBE_PFDTXGSWC);
			if (pfdtxgswc & IXGBE_PFDTXGSWC_VT_LBEN) {
				pfdtxgswc &= ~IXGBE_PFDTXGSWC_VT_LBEN;
				IXGBE_WRITE_REG(hw, IXGBE_PFDTXGSWC, pfdtxgswc);
				hw->mac.set_lben = true;
			} else {
				hw->mac.set_lben = false;
			}
		}
		rxctrl &= ~IXGBE_RXCTRL_RXEN;
		IXGBE_WRITE_REG(hw, IXGBE_RXCTRL, rxctrl);
	}
}

/**
 *  ixgbe_stop_adapter_generic - Generic stop Tx/Rx units
 *  @hw: pointer to hardware structure
 *
 *  Sets the adapter_stopped flag within ixgbe_hw struct. Clears interrupts,
 *  disables transmit and receive units. The adapter_stopped flag is used by
 *  the shared code and drivers to determine if the adapter is in a stopped
 *  state and should not touch the hardware.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_stop_adapter_generic(struct ixgbe_hw *hw)
{
	u32 reg_val;
	u16 i;

	DEBUGFUNC("ixgbe_stop_adapter_generic\n");

	/*
	 * Set the adapter_stopped flag so other driver functions stop touching
	 * the hardware
	 */
	hw->adapter_stopped = true;

	/* Disable the receive unit */
	ixgbe_disable_rx_generic(hw);

	/* Clear interrupt mask to stop interrupts from being generated */
	IXGBE_WRITE_REG(hw, IXGBE_EIMC, IXGBE_IRQ_CLEAR_MASK);

	/* Clear any pending interrupts, flush previous writes */
	IXGBE_READ_REG(hw, IXGBE_EICR);

	/* Disable the transmit unit.  Each queue must be disabled. */
	for (i = 0; i < hw->mac.max_tx_queues; i++)
		IXGBE_WRITE_REG(hw, IXGBE_TXDCTL(i), IXGBE_TXDCTL_SWFLSH);

	/* Disable the receive unit by stopping each queue */
	for (i = 0; i < hw->mac.max_rx_queues; i++) {
		reg_val = IXGBE_READ_REG(hw, IXGBE_RXDCTL(i));
		reg_val &= ~IXGBE_RXDCTL_ENABLE;
		reg_val |= IXGBE_RXDCTL_SWFLSH;
		IXGBE_WRITE_REG(hw, IXGBE_RXDCTL(i), reg_val);
	}

	/* flush all queues disables */
	IXGBE_WRITE_FLUSH(hw);
	msec_delay(2);

	/*
	 * Prevent the PCI-E bus from hanging by disabling PCI-E master
	 * access and verify no pending requests
	 */
	return ixgbe_disable_pcie_master(hw);
}

/**
 * ixgbe_clear_tx_pending - Clear pending TX work from the PCIe fifo
 * @hw: pointer to the hardware structure
 *
 * The 82599 and x540 MACs can experience issues if TX work is still pending
 * when a reset occurs.  This function prevents this by flushing the PCIe
 * buffers on the system.
 **/
void ebbrt::IxgbeDriver::ixgbe_clear_tx_pending(struct ixgbe_hw *hw)
{
	u32 gcr_ext, hlreg0, i, poll;
	u16 value;

	/*
	 * If double reset is not requested then all transactions should
	 * already be clear and as such there is no work to do
	 */
	if (!(hw->mac.flags & IXGBE_FLAGS_DOUBLE_RESET_REQUIRED))
		return;

	/*
	 * Set loopback enable to prevent any transmits from being sent
	 * should the link come up.  This assumes that the RXCTRL.RXEN bit
	 * has already been cleared.
	 */
	hlreg0 = IXGBE_READ_REG(hw, IXGBE_HLREG0);
	IXGBE_WRITE_REG(hw, IXGBE_HLREG0, hlreg0 | IXGBE_HLREG0_LPBK);

	/* Wait for a last completion before clearing buffers */
	IXGBE_WRITE_FLUSH(hw);
	msec_delay(3);

	/*
	 * Before proceeding, make sure that the PCIe block does not have
	 * transactions pending.
	 */
	// poll = ixgbe_pcie_timeout_poll(hw);
	// for (i = 0; i < poll; i++) {
	// 	usec_delay(100);
	// 	value = IXGBE_READ_PCIE_WORD(hw, IXGBE_PCI_DEVICE_STATUS);
	// 	if (IXGBE_REMOVED(hw->hw_addr))
	// 		goto out;
	// 	if (!(value & IXGBE_PCI_DEVICE_STATUS_TRANSACTION_PENDING))
	// 		goto out;
	// }

out:
	/* initiate cleaning flow for buffers in the PCIe transaction layer */
	gcr_ext = IXGBE_READ_REG(hw, IXGBE_GCR_EXT);
	IXGBE_WRITE_REG(hw, IXGBE_GCR_EXT,
			gcr_ext | IXGBE_GCR_EXT_BUFFERS_CLEAR);

	/* Flush all writes and allow 20usec for all transactions to clear */
	IXGBE_WRITE_FLUSH(hw);
	usec_delay(20);

	/* restore previous register values */
	IXGBE_WRITE_REG(hw, IXGBE_GCR_EXT, gcr_ext);
	IXGBE_WRITE_REG(hw, IXGBE_HLREG0, hlreg0);
}

/**
 *  ixgbe_read_phy_mdi - Reads a value from a specified PHY register without
 *  the SWFW lock
 *  @hw: pointer to hardware structure
 *  @reg_addr: 32 bit address of PHY register to read
 *  @phy_data: Pointer to read data from PHY register
 **/
s32 ebbrt::IxgbeDriver::ixgbe_read_phy_reg_mdi(struct ixgbe_hw *hw, u32 reg_addr, u32 device_type,
					       u16 *phy_data)
{
	u32 i, data, command;

	DEBUGFUNC("%s\n", __PRETTY_FUNCTION__);

	/* Setup and write the address cycle command */
	command = ((reg_addr << IXGBE_MSCA_NP_ADDR_SHIFT)  |
		   (device_type << IXGBE_MSCA_DEV_TYPE_SHIFT) |
		   (hw->phy.addr << IXGBE_MSCA_PHY_ADDR_SHIFT) |
		   (IXGBE_MSCA_ADDR_CYCLE | IXGBE_MSCA_MDI_COMMAND));

	IXGBE_WRITE_REG(hw, IXGBE_MSCA, command);

	/*
	 * Check every 10 usec to see if the address cycle completed.
	 * The MDI Command bit will clear when the operation is
	 * complete
	 */
	for (i = 0; i < IXGBE_MDIO_COMMAND_TIMEOUT; i++) {
	    usec_delay(10);
		command = IXGBE_READ_REG(hw, IXGBE_MSCA);
		if ((command & IXGBE_MSCA_MDI_COMMAND) == 0)
				break;
	}


	if ((command & IXGBE_MSCA_MDI_COMMAND) != 0) {
	    ebbrt::kabort("PHY address command did not complete.\n");
	    return IXGBE_ERR_PHY;
	}

	/*
	 * Address cycle complete, setup and write the read
	 * command
	 */
	command = ((reg_addr << IXGBE_MSCA_NP_ADDR_SHIFT)  |
		   (device_type << IXGBE_MSCA_DEV_TYPE_SHIFT) |
		   (hw->phy.addr << IXGBE_MSCA_PHY_ADDR_SHIFT) |
		   (IXGBE_MSCA_READ | IXGBE_MSCA_MDI_COMMAND));

	IXGBE_WRITE_REG(hw, IXGBE_MSCA, command);

	/*
	 * Check every 10 usec to see if the address cycle
	 * completed. The MDI Command bit will clear when the
	 * operation is complete
	 */
	for (i = 0; i < IXGBE_MDIO_COMMAND_TIMEOUT; i++) {
	    usec_delay(10);

		command = IXGBE_READ_REG(hw, IXGBE_MSCA);
		if ((command & IXGBE_MSCA_MDI_COMMAND) == 0)
			break;
	}

	if ((command & IXGBE_MSCA_MDI_COMMAND) != 0) {
	    ebbrt::kabort("PHY read command didn't complete\n");
		return IXGBE_ERR_PHY;
	}

	/*
	 * Read operation is complete.  Get the data
	 * from MSRWD
	 */
	data = IXGBE_READ_REG(hw, IXGBE_MSRWD);
	data >>= IXGBE_MSRWD_READ_DATA_SHIFT;
	*phy_data = (u16)(data);

	return IXGBE_SUCCESS;
}

/**
 *  ixgbe_release_eeprom_semaphore - Release hardware semaphore
 *  @hw: pointer to hardware structure
 *
 *  This function clears hardware semaphore bits.
 **/
void ebbrt::IxgbeDriver::ixgbe_release_eeprom_semaphore(struct ixgbe_hw *hw)
{
	u32 swsm;

	DEBUGFUNC("ixgbe_release_eeprom_semaphore\n");

	swsm = IXGBE_READ_REG(hw, IXGBE_SWSM);

	/* Release both semaphores by writing 0 to the bits SWESMBI and SMBI */
	swsm &= ~(IXGBE_SWSM_SWESMBI | IXGBE_SWSM_SMBI);
	IXGBE_WRITE_REG(hw, IXGBE_SWSM, swsm);
	IXGBE_WRITE_FLUSH(hw);
	ebbrt::kprintf("returning\n");
	return;
}

/**
 *  ixgbe_get_eeprom_semaphore - Get hardware semaphore
 *  @hw: pointer to hardware structure
 *
 *  Sets the hardware semaphores so EEPROM access can occur for bit-bang method
 **/
s32 ebbrt::IxgbeDriver::ixgbe_get_eeprom_semaphore(struct ixgbe_hw *hw)
{
	s32 status = IXGBE_ERR_EEPROM;
	u32 timeout = 2000;
	u32 i;
	u32 swsm;

	DEBUGFUNC("ixgbe_get_eeprom_semaphore\n");

	/* Get SMBI software semaphore between device drivers first */
	for (i = 0; i < timeout; i++) {
		/*
		 * If the SMBI bit is 0 when we read it, then the bit will be
		 * set and we have the semaphore
		 */
		swsm = IXGBE_READ_REG(hw, IXGBE_SWSM_BY_MAC(hw));
		if (!(swsm & IXGBE_SWSM_SMBI)) {
			status = IXGBE_SUCCESS;
			ebbrt::kprintf("%s have sempahore\n", __PRETTY_FUNCTION__);
			break;
		}
		usec_delay(50);
	}

	if (i == timeout) {
		DEBUGOUT("Driver can't access the Eeprom - SMBI Semaphore "
			 "not granted.\n");
		/*
		 * this release is particularly important because our attempts
		 * above to get the semaphore may have succeeded, and if there
		 * was a timeout, we should unconditionally clear the semaphore
		 * bits to free the driver to make progress
		 */
		ixgbe_release_eeprom_semaphore(hw);

		usec_delay(50);
		/*
		 * one last try
		 * If the SMBI bit is 0 when we read it, then the bit will be
		 * set and we have the semaphore
		 */
		swsm = IXGBE_READ_REG(hw, IXGBE_SWSM_BY_MAC(hw));
		if (!(swsm & IXGBE_SWSM_SMBI))
			status = IXGBE_SUCCESS;
	}

	/* Now get the semaphore between SW/FW through the SWESMBI bit */
	if (status == IXGBE_SUCCESS) {
		for (i = 0; i < timeout; i++) {
			swsm = IXGBE_READ_REG(hw, IXGBE_SWSM_BY_MAC(hw));

			/* Set the SW EEPROM semaphore bit to request access */
			swsm |= IXGBE_SWSM_SWESMBI;
			IXGBE_WRITE_REG(hw, IXGBE_SWSM_BY_MAC(hw), swsm);

			/*
			 * If we set the bit successfully then we got the
			 * semaphore.
			 */
			swsm = IXGBE_READ_REG(hw, IXGBE_SWSM_BY_MAC(hw));
			if (swsm & IXGBE_SWSM_SWESMBI)
				break;

			usec_delay(50);
		}

		/*
		 * Release semaphores and return error if SW EEPROM semaphore
		 * was not granted because we don't have access to the EEPROM
		 */
		if (i >= timeout) {
		    ebbrt::kprintf("SWESMBI Software EEPROM semaphore not granted.\n");
			ixgbe_release_eeprom_semaphore(hw);
			ebbrt::kprintf("SWESMBI Software EEPROM semaphore not granted b.\n");
			status = IXGBE_ERR_EEPROM;
		}
	} else {
	    //ebbrt::kabort("Software semaphore SMBI between device drivers "
	    //		     "not granted.\n");
	}
	ebbrt::kprintf("%s done\n", __FUNCTION__);
	return status;
}

/**
 *  ixgbe_release_swfw_sync - Release SWFW semaphore
 *  @hw: pointer to hardware structure
 *  @mask: Mask to specify which semaphore to release
 *
 *  Releases the SWFW semaphore through the GSSR register for the specified
 *  function (CSR, PHY0, PHY1, EEPROM, Flash)
 **/
void ebbrt::IxgbeDriver::ixgbe_release_swfw_sync(struct ixgbe_hw *hw, u32 mask)
{
	u32 gssr;
	u32 swmask = mask;

	DEBUGFUNC("ixgbe_release_swfw_sync\n");

	ixgbe_get_eeprom_semaphore(hw);

	gssr = IXGBE_READ_REG(hw, IXGBE_GSSR);
	gssr &= ~swmask;
	IXGBE_WRITE_REG(hw, IXGBE_GSSR, gssr);

	ixgbe_release_eeprom_semaphore(hw);
}

/**
 *  ixgbe_acquire_swfw_sync - Acquire SWFW semaphore
 *  @hw: pointer to hardware structure
 *  @mask: Mask to specify which semaphore to acquire
 *
 *  Acquires the SWFW semaphore through the GSSR register for the specified
 *  function (CSR, PHY0, PHY1, EEPROM, Flash)
 **/
s32 ebbrt::IxgbeDriver::ixgbe_acquire_swfw_sync(struct ixgbe_hw *hw, u32 mask)
{
	u32 gssr = 0;
	u32 swmask = mask;
	u32 fwmask = mask << 5;
	u32 timeout = 200;
	u32 i;

	DEBUGFUNC("ixgbe_acquire_swfw_sync\n");

	for (i = 0; i < timeout; i++) {
		/*
		 * SW NVM semaphore bit is used for access to all
		 * SW_FW_SYNC bits (not just NVM)
		 */
		if (ixgbe_get_eeprom_semaphore(hw))
			return IXGBE_ERR_SWFW_SYNC;

		gssr = IXGBE_READ_REG(hw, IXGBE_GSSR);
		if (!(gssr & (fwmask | swmask))) {
			gssr |= swmask;
			IXGBE_WRITE_REG(hw, IXGBE_GSSR, gssr);
			ixgbe_release_eeprom_semaphore(hw);
			return IXGBE_SUCCESS;
		} else {
			/* Resource is currently in use by FW or SW */
			ixgbe_release_eeprom_semaphore(hw);
			msec_delay(5);
		}
	}

	/* If time expired clear the bits holding the lock and retry */
	if (gssr & (fwmask | swmask))
		ixgbe_release_swfw_sync(hw, gssr & (fwmask | swmask));

	msec_delay(5);
	return IXGBE_ERR_SWFW_SYNC;
}

/**
 *  ixgbe_read_phy_reg_generic - Reads a value from a specified PHY register
 *  using the SWFW lock - this function is needed in most cases
 *  @hw: pointer to hardware structure
 *  @reg_addr: 32 bit address of PHY register to read
 *  @phy_data: Pointer to read data from PHY register
 **/
s32 ebbrt::IxgbeDriver::ixgbe_read_phy_reg_generic(struct ixgbe_hw *hw, u32 reg_addr,
			       u32 device_type, u16 *phy_data)
{
	s32 status;
	u32 gssr = hw->phy.phy_semaphore_mask;

	DEBUGFUNC("ixgbe_read_phy_reg_generic\n");

	//if (hw->mac.ops.acquire_swfw_sync(hw, gssr))
	//	return IXGBE_ERR_SWFW_SYNC;
	if(ixgbe_acquire_swfw_sync(hw, gssr))
	    return IXGBE_ERR_SWFW_SYNC;
	
	status = ixgbe_read_phy_reg_mdi(hw, reg_addr, device_type, phy_data);
	//status = hw->phy.ops.read_reg_mdi(hw, reg_addr, device_type, phy_data);

	ixgbe_release_swfw_sync(hw, gssr);
	//hw->mac.ops.release_swfw_sync(hw, gssr);

	return status;
}

/**
 *  ixgbe_validate_phy_addr - Determines phy address is valid
 *  @hw: pointer to hardware structure
 *
 **/
bool ebbrt::IxgbeDriver::ixgbe_validate_phy_addr(struct ixgbe_hw *hw, u32 phy_addr)
{
	u16 phy_id = 0;
	bool valid = false;

	DEBUGFUNC("ixgbe_validate_phy_addr\n");

	hw->phy.addr = phy_addr;
	//hw->phy.ops.read_reg(hw, IXGBE_MDIO_PHY_ID_HIGH,
	//		     IXGBE_MDIO_PMA_PMD_DEV_TYPE, &phy_id);
	ixgbe_read_phy_reg_generic(hw, IXGBE_MDIO_PHY_ID_HIGH,
				   IXGBE_MDIO_PMA_PMD_DEV_TYPE, &phy_id);

	if (phy_id != 0xFFFF && phy_id != 0x0)
		valid = true;

	return valid;
}

/**
 *  ixgbe_get_phy_id - Get the phy type
 *  @hw: pointer to hardware structure
 *
 **/
s32 ebbrt::IxgbeDriver::ixgbe_get_phy_id(struct ixgbe_hw *hw)
{
	u32 status;
	u16 phy_id_high = 0;
	u16 phy_id_low = 0;

	DEBUGFUNC("ixgbe_get_phy_id");

	//status = hw->phy.ops.read_reg(hw, IXGBE_MDIO_PHY_ID_HIGH,
	status = ixgbe_read_phy_reg_generic(hw, IXGBE_MDIO_PHY_ID_HIGH,
				      IXGBE_MDIO_PMA_PMD_DEV_TYPE,
				      &phy_id_high);

	if (status == IXGBE_SUCCESS) {
		hw->phy.id = (u32)(phy_id_high << 16);
		//status = hw->phy.ops.read_reg(hw, IXGBE_MDIO_PHY_ID_LOW,
		status = ixgbe_read_phy_reg_generic(hw, IXGBE_MDIO_PHY_ID_LOW,
					      IXGBE_MDIO_PMA_PMD_DEV_TYPE,
					      &phy_id_low);
		hw->phy.id |= (u32)(phy_id_low & IXGBE_PHY_REVISION_MASK);
		hw->phy.revision = (u32)(phy_id_low & ~IXGBE_PHY_REVISION_MASK);
	}
	return status;
}

/**
 *  ixgbe_get_phy_type_from_id - Get the phy type
 *  @phy_id: PHY ID information
 *
 **/
enum ixgbe_phy_type ebbrt::IxgbeDriver::ixgbe_get_phy_type_from_id(u32 phy_id)
{
	enum ixgbe_phy_type phy_type;

	DEBUGFUNC("ixgbe_get_phy_type_from_id\n");

	switch (phy_id) {
	case TN1010_PHY_ID:
		phy_type = ixgbe_phy_tn;
		break;
	case X550_PHY_ID1:
	case X550_PHY_ID2:
	case X550_PHY_ID3:
	case X540_PHY_ID:
		phy_type = ixgbe_phy_aq;
		break;
	case QT2022_PHY_ID:
		phy_type = ixgbe_phy_qt;
		break;
	case ATH_PHY_ID:
		phy_type = ixgbe_phy_nl;
		break;
	case X557_PHY_ID:
		phy_type = ixgbe_phy_x550em_ext_t;
		break;
	default:
		phy_type = ixgbe_phy_unknown;
		break;
	}
	return phy_type;
}

/**
 * ixgbe_probe_phy - Probe a single address for a PHY
 * @hw: pointer to hardware structure
 * @phy_addr: PHY address to probe
 *
 * Returns true if PHY found
 */
bool ebbrt::IxgbeDriver::ixgbe_probe_phy(struct ixgbe_hw *hw, u16 phy_addr)
{
	u16 ext_ability = 0;
	DEBUGFUNC("%s\n", __PRETTY_FUNCTION__);

	if (!ixgbe_validate_phy_addr(hw, phy_addr))
		return false;

	if (ixgbe_get_phy_id(hw))
	    return false;

	hw->phy.type = ixgbe_get_phy_type_from_id(hw->phy.id);

	 if (hw->phy.type == ixgbe_phy_unknown) {
	     //hw->phy.ops.read_reg(hw, IXGBE_MDIO_PHY_EXT_ABILITY,
	     ixgbe_read_phy_reg_generic(hw, IXGBE_MDIO_PHY_EXT_ABILITY,
					IXGBE_MDIO_PMA_PMD_DEV_TYPE, &ext_ability);
	 	if (ext_ability &
	 	    (IXGBE_MDIO_PHY_10GBASET_ABILITY |
	 	     IXGBE_MDIO_PHY_1000BASET_ABILITY))
	 		hw->phy.type = ixgbe_phy_cu_unknown;
	 	else
	 		hw->phy.type = ixgbe_phy_generic;
	 }

	return true;
}

/**
 *  ixgbe_identify_phy_generic - Get physical layer module
 *  @hw: pointer to hardware structure
 *
 *  Determines the physical layer module found on the current adapter.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_identify_phy_generic(struct ixgbe_hw *hw)
{
	s32 status = IXGBE_ERR_PHY_ADDR_INVALID;
	u16 phy_addr;

	DEBUGFUNC("ixgbe_identify_phy_generic\n");

	if (!hw->phy.phy_semaphore_mask) {
		if (hw->bus.lan_id)
			hw->phy.phy_semaphore_mask = IXGBE_GSSR_PHY1_SM;
		else
			hw->phy.phy_semaphore_mask = IXGBE_GSSR_PHY0_SM;
	}

	if (hw->phy.type != ixgbe_phy_unknown)
		return IXGBE_SUCCESS;

	for (phy_addr = 0; phy_addr < IXGBE_MAX_PHY_ADDR; phy_addr++) {
		if (ixgbe_probe_phy(hw, phy_addr)) {
			status = IXGBE_SUCCESS;
			break;
		}
	}

	/* Certain media types do not have a phy so an address will not
	 * be found and the code will take this path.  Caller has to
	 * decide if it is an error or not.
	 */
	if (status != IXGBE_SUCCESS)
		hw->phy.addr = 0;

	DEBUGFUNC("ixgbe_identify_phy_generic DONE\n");
	return status;
}

/**
 *  ixgbe_set_lan_id_multi_port_pcie - Set LAN id for PCIe multiple port devices
 *  @hw: pointer to the HW structure
 *
 *  Determines the LAN function id by reading memory-mapped registers
 *  and swaps the port value if requested.
 **/
void ebbrt::IxgbeDriver::ixgbe_set_lan_id_multi_port_pcie(struct ixgbe_hw *hw)
{
	struct ixgbe_bus_info *bus = &hw->bus;
	u32 reg;

	DEBUGFUNC("ixgbe_set_lan_id_multi_port_pcie\n");

	reg = IXGBE_READ_REG(hw, IXGBE_STATUS);
	bus->func = (reg & IXGBE_STATUS_LAN_ID) >> IXGBE_STATUS_LAN_ID_SHIFT;
	bus->lan_id = (u8)bus->func;

	/* check for a port swap */
	reg = IXGBE_READ_REG(hw, IXGBE_FACTPS_BY_MAC(hw));
	if (reg & IXGBE_FACTPS_LFS)
		bus->func ^= 0x1;

}

/**
 * ixgbe_is_sfp_probe - Returns true if SFP is being detected
 * @hw: pointer to hardware structure
 * @offset: eeprom offset to be read
 * @addr: I2C address to be read
 */
bool ebbrt::IxgbeDriver::ixgbe_is_sfp_probe(struct ixgbe_hw *hw, u8 offset, u8 addr)
{
    DEBUGFUNC("%s\n", __PRETTY_FUNCTION__);
	if (addr == IXGBE_I2C_EEPROM_DEV_ADDR &&
	    offset == IXGBE_SFF_IDENTIFIER &&
	    hw->phy.sfp_type == ixgbe_sfp_type_not_present)
		return true;
	return false;
}

/**
 *  ixgbe_get_i2c_data - Reads the I2C SDA data bit
 *  @hw: pointer to hardware structure
 *  @i2cctl: Current value of I2CCTL register
 *
 *  Returns the I2C data bit value
 *  Negates the I2C data output enable on X550 hardware.
 **/
bool ebbrt::IxgbeDriver::ixgbe_get_i2c_data(struct ixgbe_hw *hw, u32 *i2cctl)
{
	u32 data_oe_bit = IXGBE_I2C_DATA_OE_N_EN_BY_MAC(hw);
	bool data;

	DEBUGFUNC("ixgbe_get_i2c_data\n");

	if (data_oe_bit) {
		*i2cctl |= data_oe_bit;
		IXGBE_WRITE_REG(hw, IXGBE_I2CCTL_BY_MAC(hw), *i2cctl);
		IXGBE_WRITE_FLUSH(hw);
		usec_delay(IXGBE_I2C_T_FALL);
	}

	if (*i2cctl & IXGBE_I2C_DATA_IN_BY_MAC(hw))
		data = 1;
	else
		data = 0;

	return data;
}

/**
 *  ixgbe_raise_i2c_clk - Raises the I2C SCL clock
 *  @hw: pointer to hardware structure
 *  @i2cctl: Current value of I2CCTL register
 *
 *  Raises the I2C clock line '0'->'1'
 *  Negates the I2C clock output enable on X550 hardware.
 **/
void ebbrt::IxgbeDriver::ixgbe_raise_i2c_clk(struct ixgbe_hw *hw, u32 *i2cctl)
{
	u32 clk_oe_bit = IXGBE_I2C_CLK_OE_N_EN_BY_MAC(hw);
	u32 i = 0;
	u32 timeout = IXGBE_I2C_CLOCK_STRETCHING_TIMEOUT;
	u32 i2cctl_r = 0;

	DEBUGFUNC("ixgbe_raise_i2c_clk\n");

	if (clk_oe_bit) {
		*i2cctl |= clk_oe_bit;
		IXGBE_WRITE_REG(hw, IXGBE_I2CCTL_BY_MAC(hw), *i2cctl);
	}

	for (i = 0; i < timeout; i++) {
		*i2cctl |= IXGBE_I2C_CLK_OUT_BY_MAC(hw);

		IXGBE_WRITE_REG(hw, IXGBE_I2CCTL_BY_MAC(hw), *i2cctl);
		IXGBE_WRITE_FLUSH(hw);
		/* SCL rise time (1000ns) */
		usec_delay(IXGBE_I2C_T_RISE);

		i2cctl_r = IXGBE_READ_REG(hw, IXGBE_I2CCTL_BY_MAC(hw));
		if (i2cctl_r & IXGBE_I2C_CLK_IN_BY_MAC(hw))
			break;
	}
}

/**
 *  ixgbe_set_i2c_data - Sets the I2C data bit
 *  @hw: pointer to hardware structure
 *  @i2cctl: Current value of I2CCTL register
 *  @data: I2C data value (0 or 1) to set
 *
 *  Sets the I2C data bit
 *  Asserts the I2C data output enable on X550 hardware.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_set_i2c_data(struct ixgbe_hw *hw, u32 *i2cctl, bool data)
{
	u32 data_oe_bit = IXGBE_I2C_DATA_OE_N_EN_BY_MAC(hw);
	s32 status = IXGBE_SUCCESS;

	DEBUGFUNC("ixgbe_set_i2c_data\n");

	if (data)
		*i2cctl |= IXGBE_I2C_DATA_OUT_BY_MAC(hw);
	else
		*i2cctl &= ~(IXGBE_I2C_DATA_OUT_BY_MAC(hw));
	*i2cctl &= ~data_oe_bit;

	IXGBE_WRITE_REG(hw, IXGBE_I2CCTL_BY_MAC(hw), *i2cctl);
	IXGBE_WRITE_FLUSH(hw);

	/* Data rise/fall (1000ns/300ns) and set-up time (250ns) */
	usec_delay(IXGBE_I2C_T_RISE + IXGBE_I2C_T_FALL + IXGBE_I2C_T_SU_DATA);

	if (!data)	/* Can't verify data in this case */
		return IXGBE_SUCCESS;
	if (data_oe_bit) {
		*i2cctl |= data_oe_bit;
		IXGBE_WRITE_REG(hw, IXGBE_I2CCTL_BY_MAC(hw), *i2cctl);
		IXGBE_WRITE_FLUSH(hw);
	}

	/* Verify data was set correctly */
	*i2cctl = IXGBE_READ_REG(hw, IXGBE_I2CCTL_BY_MAC(hw));
	if (data != ixgbe_get_i2c_data(hw, i2cctl)) {
		status = IXGBE_ERR_I2C;
		ebbrt::kabort(
			     "Error - I2C data was not set to %X.\n",
			     data);
	}

	return status;
}

/**
 *  ixgbe_lower_i2c_clk - Lowers the I2C SCL clock
 *  @hw: pointer to hardware structure
 *  @i2cctl: Current value of I2CCTL register
 *
 *  Lowers the I2C clock line '1'->'0'
 *  Asserts the I2C clock output enable on X550 hardware.
 **/
void ebbrt::IxgbeDriver::ixgbe_lower_i2c_clk(struct ixgbe_hw *hw, u32 *i2cctl)
{
	DEBUGFUNC("ixgbe_lower_i2c_clk\n");

	*i2cctl &= ~(IXGBE_I2C_CLK_OUT_BY_MAC(hw));
	*i2cctl &= ~IXGBE_I2C_CLK_OE_N_EN_BY_MAC(hw);

	IXGBE_WRITE_REG(hw, IXGBE_I2CCTL_BY_MAC(hw), *i2cctl);
	IXGBE_WRITE_FLUSH(hw);

	/* SCL fall time (300ns) */
	usec_delay(IXGBE_I2C_T_FALL);
}

/**
 *  ixgbe_i2c_start - Sets I2C start condition
 *  @hw: pointer to hardware structure
 *
 *  Sets I2C start condition (High -> Low on SDA while SCL is High)
 *  Set bit-bang mode on X550 hardware.
 **/
void ebbrt::IxgbeDriver::ixgbe_i2c_start(struct ixgbe_hw *hw)
{
	u32 i2cctl = IXGBE_READ_REG(hw, IXGBE_I2CCTL_BY_MAC(hw));

	DEBUGFUNC("ixgbe_i2c_start\n");

	i2cctl |= IXGBE_I2C_BB_EN_BY_MAC(hw);

	/* Start condition must begin with data and clock high */
	ixgbe_set_i2c_data(hw, &i2cctl, 1);
	ixgbe_raise_i2c_clk(hw, &i2cctl);

	/* Setup time for start condition (4.7us) */
	usec_delay(IXGBE_I2C_T_SU_STA);

	ixgbe_set_i2c_data(hw, &i2cctl, 0);

	/* Hold time for start condition (4us) */
	usec_delay(IXGBE_I2C_T_HD_STA);

	ixgbe_lower_i2c_clk(hw, &i2cctl);

	/* Minimum low period of clock is 4.7 us */
	usec_delay(IXGBE_I2C_T_LOW);

}

/**
 *  ixgbe_clock_out_i2c_bit - Clocks in/out one bit via I2C data/clock
 *  @hw: pointer to hardware structure
 *  @data: data value to write
 *
 *  Clocks out one bit via I2C data/clock
 **/
s32 ebbrt::IxgbeDriver::ixgbe_clock_out_i2c_bit(struct ixgbe_hw *hw, bool data)
{
	s32 status;
	u32 i2cctl = IXGBE_READ_REG(hw, IXGBE_I2CCTL_BY_MAC(hw));

	DEBUGFUNC("ixgbe_clock_out_i2c_bit\n");

	status = ixgbe_set_i2c_data(hw, &i2cctl, data);
	if (status == IXGBE_SUCCESS) {
		ixgbe_raise_i2c_clk(hw, &i2cctl);

		/* Minimum high period of clock is 4us */
		usec_delay(IXGBE_I2C_T_HIGH);

		ixgbe_lower_i2c_clk(hw, &i2cctl);

		/* Minimum low period of clock is 4.7 us.
		 * This also takes care of the data hold time.
		 */
		usec_delay(IXGBE_I2C_T_LOW);
	} else {
		status = IXGBE_ERR_I2C;
		ebbrt::kabort(
			     "I2C data was not set to %X\n", data);
	}

	return status;
}

/**
 *  ixgbe_clock_out_i2c_byte - Clocks out one byte via I2C
 *  @hw: pointer to hardware structure
 *  @data: data byte clocked out
 *
 *  Clocks out one byte data via I2C data/clock
 **/
s32 ebbrt::IxgbeDriver::ixgbe_clock_out_i2c_byte(struct ixgbe_hw *hw, u8 data)
{
	s32 status = IXGBE_SUCCESS;
	s32 i;
	u32 i2cctl;
	bool bit;

	DEBUGFUNC("ixgbe_clock_out_i2c_byte\n");

	for (i = 7; i >= 0; i--) {
		bit = (data >> i) & 0x1;
		status = ixgbe_clock_out_i2c_bit(hw, bit);

		if (status != IXGBE_SUCCESS)
			break;
	}

	/* Release SDA line (set high) */
	i2cctl = IXGBE_READ_REG(hw, IXGBE_I2CCTL_BY_MAC(hw));
	i2cctl |= IXGBE_I2C_DATA_OUT_BY_MAC(hw);
	i2cctl |= IXGBE_I2C_DATA_OE_N_EN_BY_MAC(hw);
	IXGBE_WRITE_REG(hw, IXGBE_I2CCTL_BY_MAC(hw), i2cctl);
	IXGBE_WRITE_FLUSH(hw);

	return status;
}


/**
 *  ixgbe_get_i2c_ack - Polls for I2C ACK
 *  @hw: pointer to hardware structure
 *
 *  Clocks in/out one bit via I2C data/clock
 **/
s32 ebbrt::IxgbeDriver::ixgbe_get_i2c_ack(struct ixgbe_hw *hw)
{
	u32 data_oe_bit = IXGBE_I2C_DATA_OE_N_EN_BY_MAC(hw);
	s32 status = IXGBE_SUCCESS;
	u32 i = 0;
	u32 i2cctl = IXGBE_READ_REG(hw, IXGBE_I2CCTL_BY_MAC(hw));
	u32 timeout = 10;
	bool ack = 1;

	DEBUGFUNC("ixgbe_get_i2c_ack\n");

	if (data_oe_bit) {
		i2cctl |= IXGBE_I2C_DATA_OUT_BY_MAC(hw);
		i2cctl |= data_oe_bit;
		IXGBE_WRITE_REG(hw, IXGBE_I2CCTL_BY_MAC(hw), i2cctl);
		IXGBE_WRITE_FLUSH(hw);
	}
	ixgbe_raise_i2c_clk(hw, &i2cctl);

	/* Minimum high period of clock is 4us */
	usec_delay(IXGBE_I2C_T_HIGH);

	/* Poll for ACK.  Note that ACK in I2C spec is
	 * transition from 1 to 0 */
	for (i = 0; i < timeout; i++) {
		i2cctl = IXGBE_READ_REG(hw, IXGBE_I2CCTL_BY_MAC(hw));
		ack = ixgbe_get_i2c_data(hw, &i2cctl);

		usec_delay(1);
		if (!ack)
			break;
	}

	if (ack) {
		DEBUGOUT("I2C ack was not received.\n");
		status = IXGBE_ERR_I2C;
	}

	ixgbe_lower_i2c_clk(hw, &i2cctl);

	/* Minimum low period of clock is 4.7 us */
	usec_delay(IXGBE_I2C_T_LOW);

	return status;
}

/**
 *  ixgbe_clock_in_i2c_bit - Clocks in one bit via I2C data/clock
 *  @hw: pointer to hardware structure
 *  @data: read data value
 *
 *  Clocks in one bit via I2C data/clock
 **/
s32 ebbrt::IxgbeDriver::ixgbe_clock_in_i2c_bit(struct ixgbe_hw *hw, bool *data)
{
	u32 i2cctl = IXGBE_READ_REG(hw, IXGBE_I2CCTL_BY_MAC(hw));
	u32 data_oe_bit = IXGBE_I2C_DATA_OE_N_EN_BY_MAC(hw);

	DEBUGFUNC("ixgbe_clock_in_i2c_bit\n");

	if (data_oe_bit) {
		i2cctl |= IXGBE_I2C_DATA_OUT_BY_MAC(hw);
		i2cctl |= data_oe_bit;
		IXGBE_WRITE_REG(hw, IXGBE_I2CCTL_BY_MAC(hw), i2cctl);
		IXGBE_WRITE_FLUSH(hw);
	}
	ixgbe_raise_i2c_clk(hw, &i2cctl);

	/* Minimum high period of clock is 4us */
	usec_delay(IXGBE_I2C_T_HIGH);

	i2cctl = IXGBE_READ_REG(hw, IXGBE_I2CCTL_BY_MAC(hw));
	*data = ixgbe_get_i2c_data(hw, &i2cctl);

	ixgbe_lower_i2c_clk(hw, &i2cctl);

	/* Minimum low period of clock is 4.7 us */
	usec_delay(IXGBE_I2C_T_LOW);

	return IXGBE_SUCCESS;
}

/**
 *  ixgbe_clock_in_i2c_byte - Clocks in one byte via I2C
 *  @hw: pointer to hardware structure
 *  @data: data byte to clock in
 *
 *  Clocks in one byte data via I2C data/clock
 **/
s32 ebbrt::IxgbeDriver::ixgbe_clock_in_i2c_byte(struct ixgbe_hw *hw, u8 *data)
{
	s32 i;
	bool bit = 0;

	DEBUGFUNC("ixgbe_clock_in_i2c_byte\n");

	*data = 0;
	for (i = 7; i >= 0; i--) {
		ixgbe_clock_in_i2c_bit(hw, &bit);
		*data |= bit << i;
	}

	return IXGBE_SUCCESS;
}

/**
 *  ixgbe_i2c_stop - Sets I2C stop condition
 *  @hw: pointer to hardware structure
 *
 *  Sets I2C stop condition (Low -> High on SDA while SCL is High)
 *  Disables bit-bang mode and negates data output enable on X550
 *  hardware.
 **/
void ebbrt::IxgbeDriver::ixgbe_i2c_stop(struct ixgbe_hw *hw)
{
	u32 i2cctl = IXGBE_READ_REG(hw, IXGBE_I2CCTL_BY_MAC(hw));
	u32 data_oe_bit = IXGBE_I2C_DATA_OE_N_EN_BY_MAC(hw);
	u32 clk_oe_bit = IXGBE_I2C_CLK_OE_N_EN_BY_MAC(hw);
	u32 bb_en_bit = IXGBE_I2C_BB_EN_BY_MAC(hw);

	DEBUGFUNC("ixgbe_i2c_stop\n");

	/* Stop condition must begin with data low and clock high */
	ixgbe_set_i2c_data(hw, &i2cctl, 0);
	ixgbe_raise_i2c_clk(hw, &i2cctl);

	/* Setup time for stop condition (4us) */
	usec_delay(IXGBE_I2C_T_SU_STO);

	ixgbe_set_i2c_data(hw, &i2cctl, 1);

	/* bus free time between stop and start (4.7us)*/
	usec_delay(IXGBE_I2C_T_BUF);

	if (bb_en_bit || data_oe_bit || clk_oe_bit) {
		i2cctl &= ~bb_en_bit;
		i2cctl |= data_oe_bit | clk_oe_bit;
		IXGBE_WRITE_REG(hw, IXGBE_I2CCTL_BY_MAC(hw), i2cctl);
		IXGBE_WRITE_FLUSH(hw);
	}
}

/**
 *  ixgbe_i2c_bus_clear - Clears the I2C bus
 *  @hw: pointer to hardware structure
 *
 *  Clears the I2C bus by sending nine clock pulses.
 *  Used when data line is stuck low.
 **/
void ebbrt::IxgbeDriver::ixgbe_i2c_bus_clear(struct ixgbe_hw *hw)
{
	u32 i2cctl;
	u32 i;

	DEBUGFUNC("ixgbe_i2c_bus_clear\n");

	ixgbe_i2c_start(hw);
	i2cctl = IXGBE_READ_REG(hw, IXGBE_I2CCTL_BY_MAC(hw));

	ixgbe_set_i2c_data(hw, &i2cctl, 1);

	for (i = 0; i < 9; i++) {
		ixgbe_raise_i2c_clk(hw, &i2cctl);

		/* Min high period of clock is 4us */
		usec_delay(IXGBE_I2C_T_HIGH);

		ixgbe_lower_i2c_clk(hw, &i2cctl);

		/* Min low period of clock is 4.7us*/
		usec_delay(IXGBE_I2C_T_LOW);
	}

	ixgbe_i2c_start(hw);

	/* Put the i2c bus back to default state */
	ixgbe_i2c_stop(hw);
}

/**
 *  ixgbe_read_i2c_byte_generic_int - Reads 8 bit word over I2C
 *  @hw: pointer to hardware structure
 *  @byte_offset: byte offset to read
 *  @data: value read
 *  @lock: true if to take and release semaphore
 *
 *  Performs byte read operation to SFP module's EEPROM over I2C interface at
 *  a specified device address.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_read_i2c_byte_generic_int(struct ixgbe_hw *hw, u8 byte_offset,
					   u8 dev_addr, u8 *data, bool lock)
{
	s32 status;
	u32 max_retry = 10;
	u32 retry = 0;
	u32 swfw_mask = hw->phy.phy_semaphore_mask;
	bool nack = 1;
	*data = 0;

	DEBUGFUNC("ixgbe_read_i2c_byte_generic\n");

	if (hw->mac.type >= ixgbe_mac_X550)
		max_retry = 3;
	if (ixgbe_is_sfp_probe(hw, byte_offset, dev_addr))
		max_retry = IXGBE_SFP_DETECT_RETRIES;

	do {
	    //if (lock && hw->mac.ops.acquire_swfw_sync(hw, swfw_mask))
	    if(lock && ixgbe_acquire_swfw_sync(hw, swfw_mask))
			return IXGBE_ERR_SWFW_SYNC;

		ixgbe_i2c_start(hw);

		/* Device Address and write indication */
		status = ixgbe_clock_out_i2c_byte(hw, dev_addr);
		if (status != IXGBE_SUCCESS)
			goto fail;

		status = ixgbe_get_i2c_ack(hw);
		if (status != IXGBE_SUCCESS)
			goto fail;

		status = ixgbe_clock_out_i2c_byte(hw, byte_offset);
		if (status != IXGBE_SUCCESS)
			goto fail;

		status = ixgbe_get_i2c_ack(hw);
		if (status != IXGBE_SUCCESS)
			goto fail;

		ixgbe_i2c_start(hw);

		/* Device Address and read indication */
		status = ixgbe_clock_out_i2c_byte(hw, (dev_addr | 0x1));
		if (status != IXGBE_SUCCESS)
			goto fail;

		status = ixgbe_get_i2c_ack(hw);
		if (status != IXGBE_SUCCESS)
			goto fail;

		status = ixgbe_clock_in_i2c_byte(hw, data);
		if (status != IXGBE_SUCCESS)
			goto fail;

		status = ixgbe_clock_out_i2c_bit(hw, nack);
		if (status != IXGBE_SUCCESS)
			goto fail;

		ixgbe_i2c_stop(hw);
		if (lock)
		    //hw->mac.ops.release_swfw_sync(hw, swfw_mask);
		    ixgbe_release_swfw_sync(hw, swfw_mask);
		return IXGBE_SUCCESS;

fail:
		ixgbe_i2c_bus_clear(hw);
		if (lock) {
		    //hw->mac.ops.release_swfw_sync(hw, swfw_mask);
		    ixgbe_release_swfw_sync(hw, swfw_mask);
		    msec_delay(100);
		}
		retry++;
		if (retry < max_retry)
			DEBUGOUT("I2C byte read error - Retrying.\n");
		else
			DEBUGOUT("I2C byte read error.\n");

	} while (retry < max_retry);

	return status;
}

/**
 *  ixgbe_read_i2c_byte_generic - Reads 8 bit word over I2C
 *  @hw: pointer to hardware structure
 *  @byte_offset: byte offset to read
 *  @data: value read
 *
 *  Performs byte read operation to SFP module's EEPROM over I2C interface at
 *  a specified device address.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_read_i2c_byte_generic(struct ixgbe_hw *hw, u8 byte_offset,
				u8 dev_addr, u8 *data)
{
        DEBUGFUNC("%s\n", __PRETTY_FUNCTION__);
	return ixgbe_read_i2c_byte_generic_int(hw, byte_offset, dev_addr,
					       data, true);
}

/**
 *  ixgbe_read_i2c_eeprom_generic - Reads 8 bit EEPROM word over I2C interface
 *  @hw: pointer to hardware structure
 *  @byte_offset: EEPROM byte offset to read
 *  @eeprom_data: value read
 *
 *  Performs byte read operation to SFP module's EEPROM over I2C interface.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_read_i2c_eeprom_generic(struct ixgbe_hw *hw, u8 byte_offset,
				  u8 *eeprom_data)
{
	DEBUGFUNC("ixgbe_read_i2c_eeprom_generic\n");

	//hw->phy.ops.read_i2c_byte(

	return ixgbe_read_i2c_byte_generic(hw, byte_offset,
					   IXGBE_I2C_EEPROM_DEV_ADDR,
					   eeprom_data);
}

/**
 *  ixgbe_init_eeprom_params_generic - Initialize EEPROM params
 *  @hw: pointer to hardware structure
 *
 *  Initializes the EEPROM parameters ixgbe_eeprom_info within the
 *  ixgbe_hw struct in order to set up EEPROM access.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_init_eeprom_params_generic(struct ixgbe_hw *hw)
{
	struct ixgbe_eeprom_info *eeprom = &hw->eeprom;
	u32 eec;
	u16 eeprom_size;

	DEBUGFUNC("ixgbe_init_eeprom_params_generic\n");

	if (eeprom->type == ixgbe_eeprom_uninitialized) {
		eeprom->type = ixgbe_eeprom_none;
		/* Set default semaphore delay to 10ms which is a well
		 * tested value */
		eeprom->semaphore_delay = 10;
		/* Clear EEPROM page size, it will be initialized as needed */
		eeprom->word_page_size = 0;

		/*
		 * Check for EEPROM present first.
		 * If not present leave as none
		 */
		eec = IXGBE_READ_REG(hw, IXGBE_EEC_BY_MAC(hw));
		if (eec & IXGBE_EEC_PRES) {
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

		if (eec & IXGBE_EEC_ADDR_SIZE)
			eeprom->address_bits = 16;
		else
			eeprom->address_bits = 8;
		DEBUGOUT3("Eeprom params: type = %d, size = %d, address bits: "
			  "%d\n", eeprom->type, eeprom->word_size,
			  eeprom->address_bits);
	}

	return IXGBE_SUCCESS;
}

/**
 *  ixgbe_poll_eerd_eewr_done - Poll EERD read or EEWR write status
 *  @hw: pointer to hardware structure
 *  @ee_reg: EEPROM flag for polling
 *
 *  Polls the status bit (bit 1) of the EERD or EEWR to determine when the
 *  read or write is done respectively.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_poll_eerd_eewr_done(struct ixgbe_hw *hw, u32 ee_reg)
{
	u32 i;
	u32 reg;
	s32 status = IXGBE_ERR_EEPROM;

	DEBUGFUNC("ixgbe_poll_eerd_eewr_done\n");

	for (i = 0; i < IXGBE_EERD_EEWR_ATTEMPTS; i++) {
		if (ee_reg == IXGBE_NVM_POLL_READ)
			reg = IXGBE_READ_REG(hw, IXGBE_EERD);
		else
			reg = IXGBE_READ_REG(hw, IXGBE_EEWR);

		if (reg & IXGBE_EEPROM_RW_REG_DONE) {
			status = IXGBE_SUCCESS;
			break;
		}
		usec_delay(5);
	}

	if (i == IXGBE_EERD_EEWR_ATTEMPTS)
	    ebbrt::kabort(
			     "EEPROM read/write done polling timed out");

	return status;
}

/**
 *  ixgbe_read_eerd_buffer_generic - Read EEPROM word(s) using EERD
 *  @hw: pointer to hardware structure
 *  @offset: offset of word in the EEPROM to read
 *  @words: number of word(s)
 *  @data: 16 bit word(s) from the EEPROM
 *
 *  Reads a 16 bit word(s) from the EEPROM using the EERD register.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_read_eerd_buffer_generic(struct ixgbe_hw *hw, u16 offset,
				   u16 words, u16 *data)
{
	u32 eerd;
	s32 status = IXGBE_SUCCESS;
	u32 i;

	DEBUGFUNC("ixgbe_read_eerd_buffer_generic\n");

	//hw->eeprom.ops.init_params(hw);
	ixgbe_init_eeprom_params_generic(hw);

	if (words == 0) {
		status = IXGBE_ERR_INVALID_ARGUMENT;
		//ebbrt::kabort("Invalid EEPROM words");
		goto out;
	}

	if (offset >= hw->eeprom.word_size) {
		status = IXGBE_ERR_EEPROM;
		//ebbrt::kabort("Invalid EEPROM offset");
		goto out;
	}

	for (i = 0; i < words; i++) {
		eerd = ((offset + i) << IXGBE_EEPROM_RW_ADDR_SHIFT) |
		       IXGBE_EEPROM_RW_REG_START;

		IXGBE_WRITE_REG(hw, IXGBE_EERD, eerd);
		status = ixgbe_poll_eerd_eewr_done(hw, IXGBE_NVM_POLL_READ);

		if (status == IXGBE_SUCCESS) {
			data[i] = (IXGBE_READ_REG(hw, IXGBE_EERD) >>
				   IXGBE_EEPROM_RW_REG_DATA);
		} else {
			DEBUGOUT("Eeprom read timed out\n");
			goto out;
		}
	}
out:
	return status;
}

/**
 *  ixgbe_read_eerd_generic - Read EEPROM word using EERD
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to read
 *  @data: word read from the EEPROM
 *
 *  Reads a 16 bit word from the EEPROM using the EERD register.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_read_eerd_generic(struct ixgbe_hw *hw, u16 offset, u16 *data)
{
    DEBUGFUNC("%s\n", __PRETTY_FUNCTION__);
	return ixgbe_read_eerd_buffer_generic(hw, offset, 1, data);
}

/**
 *  ixgbe_acquire_eeprom - Acquire EEPROM using bit-bang
 *  @hw: pointer to hardware structure
 *
 *  Prepares EEPROM for access using bit-bang method. This function should
 *  be called before issuing a command to the EEPROM.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_acquire_eeprom(struct ixgbe_hw *hw)
{
	s32 status = IXGBE_SUCCESS;
	u32 eec;
	u32 i;

	DEBUGFUNC("ixgbe_acquire_eeprom\n");

	if (ixgbe_acquire_swfw_sync(hw, IXGBE_GSSR_EEP_SM)
	    != IXGBE_SUCCESS)
		status = IXGBE_ERR_SWFW_SYNC;

	if (status == IXGBE_SUCCESS) {
		eec = IXGBE_READ_REG(hw, IXGBE_EEC_BY_MAC(hw));

		/* Request EEPROM Access */
		eec |= IXGBE_EEC_REQ;
		IXGBE_WRITE_REG(hw, IXGBE_EEC_BY_MAC(hw), eec);

		for (i = 0; i < IXGBE_EEPROM_GRANT_ATTEMPTS; i++) {
			eec = IXGBE_READ_REG(hw, IXGBE_EEC_BY_MAC(hw));
			if (eec & IXGBE_EEC_GNT)
				break;
			usec_delay(5);
		}

		/* Release if grant not acquired */
		if (!(eec & IXGBE_EEC_GNT)) {
			eec &= ~IXGBE_EEC_REQ;
			IXGBE_WRITE_REG(hw, IXGBE_EEC_BY_MAC(hw), eec);
			DEBUGOUT("Could not acquire EEPROM grant\n");

		        ixgbe_release_swfw_sync(hw, IXGBE_GSSR_EEP_SM);
			status = IXGBE_ERR_EEPROM;
		}

		/* Setup EEPROM for Read/Write */
		if (status == IXGBE_SUCCESS) {
			/* Clear CS and SK */
			eec &= ~(IXGBE_EEC_CS | IXGBE_EEC_SK);
			IXGBE_WRITE_REG(hw, IXGBE_EEC_BY_MAC(hw), eec);
			IXGBE_WRITE_FLUSH(hw);
			usec_delay(1);
		}
	}
	return status;
}

/**
 *  ixgbe_raise_eeprom_clk - Raises the EEPROM's clock input.
 *  @hw: pointer to hardware structure
 *  @eec: EEC register's current value
 **/
void ebbrt::IxgbeDriver::ixgbe_raise_eeprom_clk(struct ixgbe_hw *hw, u32 *eec)
{
	DEBUGFUNC("ixgbe_raise_eeprom_clk\n");

	/*
	 * Raise the clock input to the EEPROM
	 * (setting the SK bit), then delay
	 */
	*eec = *eec | IXGBE_EEC_SK;
	IXGBE_WRITE_REG(hw, IXGBE_EEC_BY_MAC(hw), *eec);
	IXGBE_WRITE_FLUSH(hw);
	usec_delay(1);
}

/**
 *  ixgbe_lower_eeprom_clk - Lowers the EEPROM's clock input.
 *  @hw: pointer to hardware structure
 *  @eecd: EECD's current value
 **/
void ebbrt::IxgbeDriver::ixgbe_lower_eeprom_clk(struct ixgbe_hw *hw, u32 *eec)
{
	DEBUGFUNC("ixgbe_lower_eeprom_clk\n");

	/*
	 * Lower the clock input to the EEPROM (clearing the SK bit), then
	 * delay
	 */
	*eec = *eec & ~IXGBE_EEC_SK;
	IXGBE_WRITE_REG(hw, IXGBE_EEC_BY_MAC(hw), *eec);
	IXGBE_WRITE_FLUSH(hw);
	usec_delay(1);
}

/**
 *  ixgbe_shift_out_eeprom_bits - Shift data bits out to the EEPROM.
 *  @hw: pointer to hardware structure
 *  @data: data to send to the EEPROM
 *  @count: number of bits to shift out
 **/
void ebbrt::IxgbeDriver::ixgbe_shift_out_eeprom_bits(struct ixgbe_hw *hw, u16 data,
					u16 count)
{
	u32 eec;
	u32 mask;
	u32 i;

	DEBUGFUNC("ixgbe_shift_out_eeprom_bits\n");

	eec = IXGBE_READ_REG(hw, IXGBE_EEC_BY_MAC(hw));

	/*
	 * Mask is used to shift "count" bits of "data" out to the EEPROM
	 * one bit at a time.  Determine the starting bit based on count
	 */
	mask = 0x01 << (count - 1);

	for (i = 0; i < count; i++) {
		/*
		 * A "1" is shifted out to the EEPROM by setting bit "DI" to a
		 * "1", and then raising and then lowering the clock (the SK
		 * bit controls the clock input to the EEPROM).  A "0" is
		 * shifted out to the EEPROM by setting "DI" to "0" and then
		 * raising and then lowering the clock.
		 */
		if (data & mask)
			eec |= IXGBE_EEC_DI;
		else
			eec &= ~IXGBE_EEC_DI;

		IXGBE_WRITE_REG(hw, IXGBE_EEC_BY_MAC(hw), eec);
		IXGBE_WRITE_FLUSH(hw);

		usec_delay(1);

		ixgbe_raise_eeprom_clk(hw, &eec);
		ixgbe_lower_eeprom_clk(hw, &eec);

		/*
		 * Shift mask to signify next bit of data to shift in to the
		 * EEPROM
		 */
		mask = mask >> 1;
	};

	/* We leave the "DI" bit set to "0" when we leave this routine. */
	eec &= ~IXGBE_EEC_DI;
	IXGBE_WRITE_REG(hw, IXGBE_EEC_BY_MAC(hw), eec);
	IXGBE_WRITE_FLUSH(hw);
}

/**
 *  ixgbe_shift_in_eeprom_bits - Shift data bits in from the EEPROM
 *  @hw: pointer to hardware structure
 **/
u16 ebbrt::IxgbeDriver::ixgbe_shift_in_eeprom_bits(struct ixgbe_hw *hw, u16 count)
{
	u32 eec;
	u32 i;
	u16 data = 0;

	DEBUGFUNC("ixgbe_shift_in_eeprom_bits\n");

	/*
	 * In order to read a register from the EEPROM, we need to shift
	 * 'count' bits in from the EEPROM. Bits are "shifted in" by raising
	 * the clock input to the EEPROM (setting the SK bit), and then reading
	 * the value of the "DO" bit.  During this "shifting in" process the
	 * "DI" bit should always be clear.
	 */
	eec = IXGBE_READ_REG(hw, IXGBE_EEC_BY_MAC(hw));

	eec &= ~(IXGBE_EEC_DO | IXGBE_EEC_DI);

	for (i = 0; i < count; i++) {
		data = data << 1;
		ixgbe_raise_eeprom_clk(hw, &eec);

		eec = IXGBE_READ_REG(hw, IXGBE_EEC_BY_MAC(hw));

		eec &= ~(IXGBE_EEC_DI);
		if (eec & IXGBE_EEC_DO)
			data |= 1;

		ixgbe_lower_eeprom_clk(hw, &eec);
	}

	return data;
}
/**
 *  ixgbe_standby_eeprom - Returns EEPROM to a "standby" state
 *  @hw: pointer to hardware structure
 **/
void ebbrt::IxgbeDriver::ixgbe_standby_eeprom(struct ixgbe_hw *hw)
{
	u32 eec;

	DEBUGFUNC("ixgbe_standby_eeprom\n");

	eec = IXGBE_READ_REG(hw, IXGBE_EEC_BY_MAC(hw));

	/* Toggle CS to flush commands */
	eec |= IXGBE_EEC_CS;
	IXGBE_WRITE_REG(hw, IXGBE_EEC_BY_MAC(hw), eec);
	IXGBE_WRITE_FLUSH(hw);
	usec_delay(1);
	eec &= ~IXGBE_EEC_CS;
	IXGBE_WRITE_REG(hw, IXGBE_EEC_BY_MAC(hw), eec);
	IXGBE_WRITE_FLUSH(hw);
	usec_delay(1);
}

/**
 *  ixgbe_ready_eeprom - Polls for EEPROM ready
 *  @hw: pointer to hardware structure
 **/
s32 ebbrt::IxgbeDriver::ixgbe_ready_eeprom(struct ixgbe_hw *hw)
{
	s32 status = IXGBE_SUCCESS;
	u16 i;
	u8 spi_stat_reg;

	DEBUGFUNC("ixgbe_ready_eeprom\n");

	/*
	 * Read "Status Register" repeatedly until the LSB is cleared.  The
	 * EEPROM will signal that the command has been completed by clearing
	 * bit 0 of the internal status register.  If it's not cleared within
	 * 5 milliseconds, then error out.
	 */
	for (i = 0; i < IXGBE_EEPROM_MAX_RETRY_SPI; i += 5) {
		ixgbe_shift_out_eeprom_bits(hw, IXGBE_EEPROM_RDSR_OPCODE_SPI,
					    IXGBE_EEPROM_OPCODE_BITS);
		spi_stat_reg = (u8)ixgbe_shift_in_eeprom_bits(hw, 8);
		if (!(spi_stat_reg & IXGBE_EEPROM_STATUS_RDY_SPI))
			break;

		usec_delay(5);
		ixgbe_standby_eeprom(hw);
	};

	/*
	 * On some parts, SPI write time could vary from 0-20mSec on 3.3V
	 * devices (and only 0-5mSec on 5V devices)
	 */
	if (i >= IXGBE_EEPROM_MAX_RETRY_SPI) {
		DEBUGOUT("SPI EEPROM Status error\n");
		status = IXGBE_ERR_EEPROM;
	}

	return status;
}

/**
 *  ixgbe_release_eeprom - Release EEPROM, release semaphores
 *  @hw: pointer to hardware structure
 **/
void ebbrt::IxgbeDriver::ixgbe_release_eeprom(struct ixgbe_hw *hw)
{
	u32 eec;

	DEBUGFUNC("ixgbe_release_eeprom\n");

	eec = IXGBE_READ_REG(hw, IXGBE_EEC_BY_MAC(hw));

	eec |= IXGBE_EEC_CS;  /* Pull CS high */
	eec &= ~IXGBE_EEC_SK; /* Lower SCK */

	IXGBE_WRITE_REG(hw, IXGBE_EEC_BY_MAC(hw), eec);
	IXGBE_WRITE_FLUSH(hw);

	usec_delay(1);

	/* Stop requesting EEPROM access */
	eec &= ~IXGBE_EEC_REQ;
	IXGBE_WRITE_REG(hw, IXGBE_EEC_BY_MAC(hw), eec);

        ixgbe_release_swfw_sync(hw, IXGBE_GSSR_EEP_SM);

	ebbrt::kprintf("%s %d\n", __FUNCTION__, hw->eeprom.semaphore_delay);
	/* Delay before attempt to obtain semaphore again to allow FW access */
	msec_delay(hw->eeprom.semaphore_delay);
}

/**
 *  ixgbe_read_eeprom_buffer_bit_bang - Read EEPROM using bit-bang
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to be read
 *  @words: number of word(s)
 *  @data: read 16 bit word(s) from EEPROM
 *
 *  Reads 16 bit word(s) from EEPROM through bit-bang method
 **/
s32 ebbrt::IxgbeDriver::ixgbe_read_eeprom_buffer_bit_bang(struct ixgbe_hw *hw, u16 offset,
					     u16 words, u16 *data)
{
	s32 status;
	u16 word_in;
	u8 read_opcode = IXGBE_EEPROM_READ_OPCODE_SPI;
	u16 i;

	DEBUGFUNC("ixgbe_read_eeprom_buffer_bit_bang\n");

	/* Prepare the EEPROM for reading  */
	status = ixgbe_acquire_eeprom(hw);

	if (status == IXGBE_SUCCESS) {
		if (ixgbe_ready_eeprom(hw) != IXGBE_SUCCESS) {
			ixgbe_release_eeprom(hw);
			status = IXGBE_ERR_EEPROM;
		}
	}

	if (status == IXGBE_SUCCESS) {
		for (i = 0; i < words; i++) {
			ixgbe_standby_eeprom(hw);
			/*
			 * Some SPI eeproms use the 8th address bit embedded
			 * in the opcode
			 */
			if ((hw->eeprom.address_bits == 8) &&
			    ((offset + i) >= 128))
				read_opcode |= IXGBE_EEPROM_A8_OPCODE_SPI;

			/* Send the READ command (opcode + addr) */
			ixgbe_shift_out_eeprom_bits(hw, read_opcode,
						    IXGBE_EEPROM_OPCODE_BITS);
			ixgbe_shift_out_eeprom_bits(hw, (u16)((offset + i) * 2),
						    hw->eeprom.address_bits);

			/* Read the data. */
			word_in = ixgbe_shift_in_eeprom_bits(hw, 16);
			data[i] = (word_in >> 8) | (word_in << 8);
		}

		/* End this read operation */
		ixgbe_release_eeprom(hw);
	}

	return status;
}

/**
 *  ixgbe_read_eeprom_bit_bang_generic - Read EEPROM word using bit-bang
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to be read
 *  @data: read 16 bit value from EEPROM
 *
 *  Reads 16 bit value from EEPROM through bit-bang method
 **/
s32 ebbrt::IxgbeDriver::ixgbe_read_eeprom_bit_bang_generic(struct ixgbe_hw *hw, u16 offset,
				       u16 *data)
{
	s32 status;

	DEBUGFUNC("ixgbe_read_eeprom_bit_bang_generic\n");

	//hw->eeprom.ops.init_params(hw);
	ixgbe_init_eeprom_params_generic(hw);
	
	if (offset >= hw->eeprom.word_size) {
		status = IXGBE_ERR_EEPROM;
		goto out;
	}

	status = ixgbe_read_eeprom_buffer_bit_bang(hw, offset, 1, data);

out:
	return status;
}

/**
 *  ixgbe_read_eeprom_82599 - Read EEPROM word using
 *  fastest available method
 *
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to read
 *  @data: word read from the EEPROM
 *
 *  Reads a 16 bit word from the EEPROM
 **/
s32 ebbrt::IxgbeDriver::ixgbe_read_eeprom_82599(struct ixgbe_hw *hw,
				   u16 offset, u16 *data)
{
	struct ixgbe_eeprom_info *eeprom = &hw->eeprom;
	s32 ret_val = IXGBE_ERR_CONFIG;

	DEBUGFUNC("ixgbe_read_eeprom_82599\n");

	/*
	 * If EEPROM is detected and can be addressed using 14 bits,
	 * use EERD otherwise use bit bang
	 */
	if ((eeprom->type == ixgbe_eeprom_spi) &&
	    (offset <= IXGBE_EERD_MAX_ADDR)) {
		ret_val = ixgbe_read_eerd_generic(hw, offset, data);
	}
	else{
	    //ebbrt::kabort("%s using eeprom bit bang generic\n", __PRETTY_FUNCTION__);
	    ret_val = ixgbe_read_eeprom_bit_bang_generic(hw, offset, data);
	}

	return ret_val;
}

/**
 *  ixgbe_get_device_caps_generic - Get additional device capabilities
 *  @hw: pointer to hardware structure
 *  @device_caps: the EEPROM word with the extra device capabilities
 *
 *  This function will read the EEPROM location for the device capabilities,
 *  and return the word through device_caps.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_get_device_caps_generic(struct ixgbe_hw *hw, u16 *device_caps)
{
	DEBUGFUNC("ixgbe_get_device_caps_generic\n");

	ixgbe_read_eeprom_82599(hw, IXGBE_DEVICE_CAPS, device_caps);
	    //hw->eeprom.ops.read(hw, IXGBE_DEVICE_CAPS, device_caps);

	return IXGBE_SUCCESS;
}

/**
 *  ixgbe_identify_sfp_module_generic - Identifies SFP modules
 *  @hw: pointer to hardware structure
 *
 *  Searches for and identifies the SFP module and assigns appropriate PHY type.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_identify_sfp_module_generic(struct ixgbe_hw *hw)
{
	s32 status = IXGBE_ERR_PHY_ADDR_INVALID;
	u32 vendor_oui = 0;
	enum ixgbe_sfp_type stored_sfp_type = hw->phy.sfp_type;
	u8 identifier = 0;
	u8 comp_codes_1g = 0;
	u8 comp_codes_10g = 0;
	u8 oui_bytes[3] = {0, 0, 0};
	u8 cable_tech = 0;
	u8 cable_spec = 0;
	u16 enforce_sfp = 0;

	DEBUGFUNC("ixgbe_identify_sfp_module_generic\n");

	//if (hw->mac.ops.get_media_type(hw) != ixgbe_media_type_fiber) {
	if(ixgbe_get_media_type_82599(hw) != ixgbe_media_type_fiber) {
		hw->phy.sfp_type = ixgbe_sfp_type_not_present;
		status = IXGBE_ERR_SFP_NOT_PRESENT;
		goto out;
	}

	/* LAN ID is needed for I2C access */
	//hw->mac.ops.set_lan_id(hw);
	ixgbe_set_lan_id_multi_port_pcie(hw);

	//status = hw->phy.ops.read_i2c_eeprom(hw,
	//				     IXGBE_SFF_IDENTIFIER,
	//				     &identifier);
	status = ixgbe_read_i2c_eeprom_generic(hw,
					     IXGBE_SFF_IDENTIFIER,
					     &identifier);

	if (status != IXGBE_SUCCESS)
		goto err_read_i2c_eeprom;

	if (identifier != IXGBE_SFF_IDENTIFIER_SFP) {
		hw->phy.type = ixgbe_phy_sfp_unsupported;
		status = IXGBE_ERR_SFP_NOT_SUPPORTED;
	} else {
	    status = ixgbe_read_i2c_eeprom_generic(hw,
						     IXGBE_SFF_1GBE_COMP_CODES,
						     &comp_codes_1g);

		if (status != IXGBE_SUCCESS)
			goto err_read_i2c_eeprom;

		status = ixgbe_read_i2c_eeprom_generic(hw,
						     IXGBE_SFF_10GBE_COMP_CODES,
						     &comp_codes_10g);

		if (status != IXGBE_SUCCESS)
			goto err_read_i2c_eeprom;
		status = ixgbe_read_i2c_eeprom_generic(hw,
						     IXGBE_SFF_CABLE_TECHNOLOGY,
						     &cable_tech);

		if (status != IXGBE_SUCCESS)
			goto err_read_i2c_eeprom;

		 /* ID Module
		  * =========
		  * 0   SFP_DA_CU
		  * 1   SFP_SR
		  * 2   SFP_LR
		  * 3   SFP_DA_CORE0 - 82599-specific
		  * 4   SFP_DA_CORE1 - 82599-specific
		  * 5   SFP_SR/LR_CORE0 - 82599-specific
		  * 6   SFP_SR/LR_CORE1 - 82599-specific
		  * 7   SFP_act_lmt_DA_CORE0 - 82599-specific
		  * 8   SFP_act_lmt_DA_CORE1 - 82599-specific
		  * 9   SFP_1g_cu_CORE0 - 82599-specific
		  * 10  SFP_1g_cu_CORE1 - 82599-specific
		  * 11  SFP_1g_sx_CORE0 - 82599-specific
		  * 12  SFP_1g_sx_CORE1 - 82599-specific
		  */
		if (hw->mac.type == ixgbe_mac_82598EB) {
			if (cable_tech & IXGBE_SFF_DA_PASSIVE_CABLE)
				hw->phy.sfp_type = ixgbe_sfp_type_da_cu;
			else if (comp_codes_10g & IXGBE_SFF_10GBASESR_CAPABLE)
				hw->phy.sfp_type = ixgbe_sfp_type_sr;
			else if (comp_codes_10g & IXGBE_SFF_10GBASELR_CAPABLE)
				hw->phy.sfp_type = ixgbe_sfp_type_lr;
			else
				hw->phy.sfp_type = ixgbe_sfp_type_unknown;
		} else {
			if (cable_tech & IXGBE_SFF_DA_PASSIVE_CABLE) {
				if (hw->bus.lan_id == 0)
					hw->phy.sfp_type =
						     ixgbe_sfp_type_da_cu_core0;
				else
					hw->phy.sfp_type =
						     ixgbe_sfp_type_da_cu_core1;
			} else if (cable_tech & IXGBE_SFF_DA_ACTIVE_CABLE) {
			    ixgbe_read_i2c_eeprom_generic(
						hw, IXGBE_SFF_CABLE_SPEC_COMP,
						&cable_spec);
				if (cable_spec &
				    IXGBE_SFF_DA_SPEC_ACTIVE_LIMITING) {
					if (hw->bus.lan_id == 0)
						hw->phy.sfp_type =
						ixgbe_sfp_type_da_act_lmt_core0;
					else
						hw->phy.sfp_type =
						ixgbe_sfp_type_da_act_lmt_core1;
				} else {
					hw->phy.sfp_type =
							ixgbe_sfp_type_unknown;
				}
			} else if (comp_codes_10g &
				   (IXGBE_SFF_10GBASESR_CAPABLE |
				    IXGBE_SFF_10GBASELR_CAPABLE)) {
				if (hw->bus.lan_id == 0)
					hw->phy.sfp_type =
						      ixgbe_sfp_type_srlr_core0;
				else
					hw->phy.sfp_type =
						      ixgbe_sfp_type_srlr_core1;
			} else if (comp_codes_1g & IXGBE_SFF_1GBASET_CAPABLE) {
				if (hw->bus.lan_id == 0)
					hw->phy.sfp_type =
						ixgbe_sfp_type_1g_cu_core0;
				else
					hw->phy.sfp_type =
						ixgbe_sfp_type_1g_cu_core1;
			} else if (comp_codes_1g & IXGBE_SFF_1GBASESX_CAPABLE) {
				if (hw->bus.lan_id == 0)
					hw->phy.sfp_type =
						ixgbe_sfp_type_1g_sx_core0;
				else
					hw->phy.sfp_type =
						ixgbe_sfp_type_1g_sx_core1;
			} else if (comp_codes_1g & IXGBE_SFF_1GBASELX_CAPABLE) {
				if (hw->bus.lan_id == 0)
					hw->phy.sfp_type =
						ixgbe_sfp_type_1g_lx_core0;
				else
					hw->phy.sfp_type =
						ixgbe_sfp_type_1g_lx_core1;
			} else {
				hw->phy.sfp_type = ixgbe_sfp_type_unknown;
			}
		}

		if (hw->phy.sfp_type != stored_sfp_type)
			hw->phy.sfp_setup_needed = true;

		/* Determine if the SFP+ PHY is dual speed or not. */
		hw->phy.multispeed_fiber = false;
		if (((comp_codes_1g & IXGBE_SFF_1GBASESX_CAPABLE) &&
		   (comp_codes_10g & IXGBE_SFF_10GBASESR_CAPABLE)) ||
		   ((comp_codes_1g & IXGBE_SFF_1GBASELX_CAPABLE) &&
		   (comp_codes_10g & IXGBE_SFF_10GBASELR_CAPABLE)))
			hw->phy.multispeed_fiber = true;

		/* Determine PHY vendor */
		if (hw->phy.type != ixgbe_phy_nl) {
			hw->phy.id = identifier;
			status = ixgbe_read_i2c_eeprom_generic(hw,
						    IXGBE_SFF_VENDOR_OUI_BYTE0,
						    &oui_bytes[0]);

			if (status != IXGBE_SUCCESS)
				goto err_read_i2c_eeprom;

			status = ixgbe_read_i2c_eeprom_generic(hw,
						    IXGBE_SFF_VENDOR_OUI_BYTE1,
						    &oui_bytes[1]);

			if (status != IXGBE_SUCCESS)
				goto err_read_i2c_eeprom;

			status = ixgbe_read_i2c_eeprom_generic(hw,
						    IXGBE_SFF_VENDOR_OUI_BYTE2,
						    &oui_bytes[2]);

			if (status != IXGBE_SUCCESS)
				goto err_read_i2c_eeprom;

			vendor_oui =
			  ((oui_bytes[0] << IXGBE_SFF_VENDOR_OUI_BYTE0_SHIFT) |
			   (oui_bytes[1] << IXGBE_SFF_VENDOR_OUI_BYTE1_SHIFT) |
			   (oui_bytes[2] << IXGBE_SFF_VENDOR_OUI_BYTE2_SHIFT));

			switch (vendor_oui) {
			case IXGBE_SFF_VENDOR_OUI_TYCO:
				if (cable_tech & IXGBE_SFF_DA_PASSIVE_CABLE)
					hw->phy.type =
						    ixgbe_phy_sfp_passive_tyco;
				break;
			case IXGBE_SFF_VENDOR_OUI_FTL:
				if (cable_tech & IXGBE_SFF_DA_ACTIVE_CABLE)
					hw->phy.type = ixgbe_phy_sfp_ftl_active;
				else
					hw->phy.type = ixgbe_phy_sfp_ftl;
				break;
			case IXGBE_SFF_VENDOR_OUI_AVAGO:
				hw->phy.type = ixgbe_phy_sfp_avago;
				break;
			case IXGBE_SFF_VENDOR_OUI_INTEL:
				hw->phy.type = ixgbe_phy_sfp_intel;
				break;
			default:
				if (cable_tech & IXGBE_SFF_DA_PASSIVE_CABLE)
					hw->phy.type =
						 ixgbe_phy_sfp_passive_unknown;
				else if (cable_tech & IXGBE_SFF_DA_ACTIVE_CABLE)
					hw->phy.type =
						ixgbe_phy_sfp_active_unknown;
				else
					hw->phy.type = ixgbe_phy_sfp_unknown;
				break;
			}
		}

		/* Allow any DA cable vendor */
		if (cable_tech & (IXGBE_SFF_DA_PASSIVE_CABLE |
		    IXGBE_SFF_DA_ACTIVE_CABLE)) {
			status = IXGBE_SUCCESS;
			goto out;
		}

		/* Verify supported 1G SFP modules */
		if (comp_codes_10g == 0 &&
		    !(hw->phy.sfp_type == ixgbe_sfp_type_1g_cu_core1 ||
		      hw->phy.sfp_type == ixgbe_sfp_type_1g_cu_core0 ||
		      hw->phy.sfp_type == ixgbe_sfp_type_1g_lx_core0 ||
		      hw->phy.sfp_type == ixgbe_sfp_type_1g_lx_core1 ||
		      hw->phy.sfp_type == ixgbe_sfp_type_1g_sx_core0 ||
		      hw->phy.sfp_type == ixgbe_sfp_type_1g_sx_core1)) {
			hw->phy.type = ixgbe_phy_sfp_unsupported;
			status = IXGBE_ERR_SFP_NOT_SUPPORTED;
			goto out;
		}

		/* Anything else 82598-based is supported */
		if (hw->mac.type == ixgbe_mac_82598EB) {
			status = IXGBE_SUCCESS;
			goto out;
		}

		//ixgbe_get_device_caps(hw, &enforce_sfp);
		ixgbe_get_device_caps_generic(hw, &enforce_sfp);
		if (!(enforce_sfp & IXGBE_DEVICE_CAPS_ALLOW_ANY_SFP) &&
		    !(hw->phy.sfp_type == ixgbe_sfp_type_1g_cu_core0 ||
		      hw->phy.sfp_type == ixgbe_sfp_type_1g_cu_core1 ||
		      hw->phy.sfp_type == ixgbe_sfp_type_1g_lx_core0 ||
		      hw->phy.sfp_type == ixgbe_sfp_type_1g_lx_core1 ||
		      hw->phy.sfp_type == ixgbe_sfp_type_1g_sx_core0 ||
		      hw->phy.sfp_type == ixgbe_sfp_type_1g_sx_core1)) {
			/* Make sure we're a supported PHY type */
			if (hw->phy.type == ixgbe_phy_sfp_intel) {
				status = IXGBE_SUCCESS;
			} else {
				if (hw->allow_unsupported_sfp == true) {
				    ebbrt::kprintf("WARNING: Intel (R) Network "
					      "Connections are quality tested "
					      "using Intel (R) Ethernet Optics."
					      " Using untested modules is not "
					      "supported and may cause unstable"
					      " operation or damage to the "
					      "module or the adapter. Intel "
					      "Corporation is not responsible "
					      "for any harm caused by using "
					      "untested modules.\n", status);
					status = IXGBE_SUCCESS;
				} else {
					DEBUGOUT("SFP+ module not supported\n");
					hw->phy.type =
						ixgbe_phy_sfp_unsupported;
					status = IXGBE_ERR_SFP_NOT_SUPPORTED;
				}
			}
		} else {
			status = IXGBE_SUCCESS;
		}
	}

out:
	return status;

err_read_i2c_eeprom:
	hw->phy.sfp_type = ixgbe_sfp_type_not_present;
	if (hw->phy.type != ixgbe_phy_nl) {
		hw->phy.id = 0;
		hw->phy.type = ixgbe_phy_unknown;
	}
	return IXGBE_ERR_SFP_NOT_PRESENT;
}

/**
 *  ixgbe_identify_module_generic - Identifies module type
 *  @hw: pointer to hardware structure
 *
 *  Determines HW type and calls appropriate function.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_identify_module_generic(struct ixgbe_hw *hw)
{
	s32 status = IXGBE_ERR_SFP_NOT_PRESENT;

	DEBUGFUNC("ixgbe_identify_module_generic");

	switch (ixgbe_get_media_type_82599(hw)) {
	case ixgbe_media_type_fiber:
		status = ixgbe_identify_sfp_module_generic(hw);
		break;
	default:
		hw->phy.sfp_type = ixgbe_sfp_type_not_present;
		status = IXGBE_ERR_SFP_NOT_PRESENT;
		ebbrt::kabort("%s\n", __PRETTY_FUNCTION__);
		break;
	}

	return status;
}

/**
 *  ixgbe_identify_phy_82599 - Get physical layer module
 *  @hw: pointer to hardware structure
 *
 *  Determines the physical layer module found on the current adapter.
 *  If PHY already detected, maintains current PHY type in hw struct,
 *  otherwise executes the PHY detection routine.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_identify_phy_82599(struct ixgbe_hw *hw)
{
	s32 status;

	DEBUGFUNC("ixgbe_identify_phy_82599\n");

	/* Detect PHY if not unknown - returns success if already detected. */
	status = ixgbe_identify_phy_generic(hw);
	if (status != IXGBE_SUCCESS) {
	    /* 82599 10GBASE-T requires an external PHY */
	    if (ixgbe_get_media_type_82599(hw) == ixgbe_media_type_copper)
		return status;
	    else
		status = ixgbe_identify_module_generic(hw);
	}
	
	 /* Set PHY type none if no PHY detected */
	if (hw->phy.type == ixgbe_phy_unknown) {
	 	hw->phy.type = ixgbe_phy_none;
	 	return IXGBE_SUCCESS;
	}
	
	/* Return error if SFP module has been detected but is not supported */
	if (hw->phy.type == ixgbe_phy_sfp_unsupported)
	    return IXGBE_ERR_SFP_NOT_SUPPORTED;

	DEBUGFUNC("ixgbe_identify_phy_82599 DONE\n");

	return status;
}

/**
 *  ixgbe_init_phy_ops_82599 - PHY/SFP specific init
 *  @hw: pointer to hardware structure
 *
 *  Initialize any function pointers that were not able to be
 *  set during init_shared_code because the PHY/SFP type was
 *  not known.  Perform the SFP init if necessary.
 *
 **/
s32 ebbrt::IxgbeDriver::ixgbe_init_phy_ops_82599(struct ixgbe_hw *hw)
{
	struct ixgbe_mac_info *mac = &hw->mac;
	struct ixgbe_phy_info *phy = &hw->phy;
	s32 ret_val = IXGBE_SUCCESS;
	u32 esdp;

	DEBUGFUNC("ixgbe_init_phy_ops_82599\n");

	if (hw->device_id == IXGBE_DEV_ID_82599_QSFP_SF_QP) {
	    ebbrt::kabort("hw->device_id == IXGBE_DEV_ID_82599_QSFP_SF_QP\n");
	}
	
	/* Identify the PHY or SFP module */
	ret_val = ixgbe_identify_phy_82599(hw);
	//ret_val = phy->ops.identify(hw);

	if (ret_val == IXGBE_ERR_SFP_NOT_SUPPORTED)
	    goto init_phy_ops_out;

        /* Setup function pointers based on detected SFP module and speeds */
	ixgbe_init_mac_link_ops_82599(hw);
	if (hw->phy.sfp_type != ixgbe_sfp_type_unknown)
	    hw->phy.ops.reset = NULL;

	/* If copper media, overwrite with copper function pointers */
	if (ixgbe_get_media_type_82599(hw) == ixgbe_media_type_copper) {
	    ebbrt::kabort("%s copper?\n", __PRETTY_FUNCTION__);
	    //mac->ops.setup_link = ixgbe_setup_copper_link_82599;
	    //	mac->ops.get_link_capabilities =
	    //			  ixgbe_get_copper_link_capabilities_generic;
	}
	
	/* Set necessary function pointers based on PHY type */
	switch (hw->phy.type) {
	 case ixgbe_phy_tn:
	     ebbrt::kprintf("%s ixgbe_phy_tn\n", __PRETTY_FUNCTION__);
	     //phy->ops.setup_link = ixgbe_setup_phy_link_tnx;
	     //phy->ops.check_link = ixgbe_check_phy_link_tnx;
	     //	phy->ops.get_firmware_version =
	     //	    ixgbe_get_phy_firmware_version_tnx;
	 	break;
	default:
	 	break;
	}
init_phy_ops_out:
	return ret_val;
}

/**
 *  ixgbe_check_mac_link_generic - Determine link and speed status
 *  @hw: pointer to hardware structure
 *  @speed: pointer to link speed
 *  @link_up: true when link is up
 *  @link_up_wait_to_complete: bool used to wait for link up or not
 *
 *  Reads the links register to determine if link is up and the current speed
 **/
s32 ebbrt::IxgbeDriver::ixgbe_check_mac_link_generic(struct ixgbe_hw *hw, ixgbe_link_speed *speed,
				 bool *link_up, bool link_up_wait_to_complete)
{
	u32 links_reg, links_orig;
	u32 i;

	DEBUGFUNC("ixgbe_check_mac_link_generic\n");

	/* clear the old state */
	links_orig = IXGBE_READ_REG(hw, IXGBE_LINKS);

	links_reg = IXGBE_READ_REG(hw, IXGBE_LINKS);

	if (links_orig != links_reg) {
		DEBUGOUT2("LINKS changed from %08X to %08X\n",
			  links_orig, links_reg);
	}

	if (link_up_wait_to_complete) {
		for (i = 0; i < hw->mac.max_link_up_time; i++) {
			if (links_reg & IXGBE_LINKS_UP) {
				*link_up = true;
				break;
			} else {
				*link_up = false;
			}
			msec_delay(100);
			links_reg = IXGBE_READ_REG(hw, IXGBE_LINKS);
		}
	} else {
		if (links_reg & IXGBE_LINKS_UP)
			*link_up = true;
		else
			*link_up = false;
	}

	switch (links_reg & IXGBE_LINKS_SPEED_82599) {
	case IXGBE_LINKS_SPEED_10G_82599:
		*speed = IXGBE_LINK_SPEED_10GB_FULL;
		if (hw->mac.type >= ixgbe_mac_X550) {
			if (links_reg & IXGBE_LINKS_SPEED_NON_STD)
				*speed = IXGBE_LINK_SPEED_2_5GB_FULL;
		}
		break;
	case IXGBE_LINKS_SPEED_1G_82599:
		*speed = IXGBE_LINK_SPEED_1GB_FULL;
		break;
	case IXGBE_LINKS_SPEED_100_82599:
		*speed = IXGBE_LINK_SPEED_100_FULL;
		if (hw->mac.type >= ixgbe_mac_X550) {
			if (links_reg & IXGBE_LINKS_SPEED_NON_STD)
				*speed = IXGBE_LINK_SPEED_5GB_FULL;
		}
		break;
	default:
		*speed = IXGBE_LINK_SPEED_UNKNOWN;
	}

	return IXGBE_SUCCESS;
}

/**
 * ixgbe_check_reset_blocked - check status of MNG FW veto bit
 * @hw: pointer to the hardware structure
 *
 * This function checks the MMNGC.MNG_VETO bit to see if there are
 * any constraints on link from manageability.  For MAC's that don't
 * have this bit just return faluse since the link can not be blocked
 * via this method.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_check_reset_blocked(struct ixgbe_hw *hw)
{
	u32 mmngc;

	DEBUGFUNC("ixgbe_check_reset_blocked\n");

	/* If we don't have this bit, it can't be blocking */
	if (hw->mac.type == ixgbe_mac_82598EB)
		return false;

	mmngc = IXGBE_READ_REG(hw, IXGBE_MMNGC);
	if (mmngc & IXGBE_MMNGC_MNG_VETO) {
	    ebbrt::kabort(
			      "MNG_VETO bit detected.\n");
		return true;
	}

	return false;
}

/**
 *  ixgbe_verify_lesm_fw_enabled_82599 - Checks LESM FW module state.
 *  @hw: pointer to hardware structure
 *
 *  Returns true if the LESM FW module is present and enabled. Otherwise
 *  returns false. Smart Speed must be disabled if LESM FW module is enabled.
 **/
bool ebbrt::IxgbeDriver::ixgbe_verify_lesm_fw_enabled_82599(struct ixgbe_hw *hw)
{
	bool lesm_enabled = false;
	u16 fw_offset, fw_lesm_param_offset, fw_lesm_state;
	s32 status;

	DEBUGFUNC("ixgbe_verify_lesm_fw_enabled_82599\n");

	/* get the offset to the Firmware Module block */
	//status = hw->eeprom.ops.read(hw, IXGBE_FW_PTR, &fw_offset);
	status = ixgbe_read_eeprom_82599(hw, IXGBE_FW_PTR, &fw_offset);

	if ((status != IXGBE_SUCCESS) ||
	    (fw_offset == 0) || (fw_offset == 0xFFFF))
		goto out;

	/* get the offset to the LESM Parameters block */
	//status = hw->eeprom.ops.read(hw, (fw_offset +
	status = ixgbe_read_eeprom_82599(hw, (fw_offset +
				     IXGBE_FW_LESM_PARAMETERS_PTR),
				     &fw_lesm_param_offset);

	if ((status != IXGBE_SUCCESS) ||
	    (fw_lesm_param_offset == 0) || (fw_lesm_param_offset == 0xFFFF))
		goto out;

	/* get the LESM state word */
	//status = hw->eeprom.ops.read(hw, (fw_lesm_param_offset +
	status = ixgbe_read_eeprom_82599(hw, (fw_lesm_param_offset +
				     IXGBE_FW_LESM_STATE_1),
				     &fw_lesm_state);

	if ((status == IXGBE_SUCCESS) &&
	    (fw_lesm_state & IXGBE_FW_LESM_STATE_ENABLED))
		lesm_enabled = true;

out:
	return lesm_enabled;
}

/**
 * ixgbe_reset_pipeline_82599 - perform pipeline reset
 *
 *  @hw: pointer to hardware structure
 *
 * Reset pipeline by asserting Restart_AN together with LMS change to ensure
 * full pipeline reset.  This function assumes the SW/FW lock is held.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_reset_pipeline_82599(struct ixgbe_hw *hw)
{
	s32 ret_val;
	u32 anlp1_reg = 0;
	u32 i, autoc_reg, autoc2_reg;

	/* Enable link if disabled in NVM */
	autoc2_reg = IXGBE_READ_REG(hw, IXGBE_AUTOC2);
	if (autoc2_reg & IXGBE_AUTOC2_LINK_DISABLE_MASK) {
		autoc2_reg &= ~IXGBE_AUTOC2_LINK_DISABLE_MASK;
		IXGBE_WRITE_REG(hw, IXGBE_AUTOC2, autoc2_reg);
		IXGBE_WRITE_FLUSH(hw);
	}

	autoc_reg = IXGBE_READ_REG(hw, IXGBE_AUTOC);
	autoc_reg |= IXGBE_AUTOC_AN_RESTART;
	/* Write AUTOC register with toggled LMS[2] bit and Restart_AN */
	IXGBE_WRITE_REG(hw, IXGBE_AUTOC,
			autoc_reg ^ (0x4 << IXGBE_AUTOC_LMS_SHIFT));
	/* Wait for AN to leave state 0 */
	for (i = 0; i < 10; i++) {
		msec_delay(4);
		anlp1_reg = IXGBE_READ_REG(hw, IXGBE_ANLP1);
		if (anlp1_reg & IXGBE_ANLP1_AN_STATE_MASK)
			break;
	}

	if (!(anlp1_reg & IXGBE_ANLP1_AN_STATE_MASK)) {
		DEBUGOUT("auto negotiation not completed\n");
		ret_val = IXGBE_ERR_RESET_FAILED;
		goto reset_pipeline_out;
	}

	ret_val = IXGBE_SUCCESS;

reset_pipeline_out:
	/* Write AUTOC register with original LMS field and Restart_AN */
	IXGBE_WRITE_REG(hw, IXGBE_AUTOC, autoc_reg);
	IXGBE_WRITE_FLUSH(hw);

	return ret_val;
}

/**
 *  prot_autoc_read_82599 - Hides MAC differences needed for AUTOC read
 *  @hw: pointer to hardware structure
 *  @locked: Return the if we locked for this read.
 *  @reg_val: Value we read from AUTOC
 *
 *  For this part (82599) we need to wrap read-modify-writes with a possible
 *  FW/SW lock.  It is assumed this lock will be freed with the next
 *  prot_autoc_write_82599().
 */
s32 ebbrt::IxgbeDriver::prot_autoc_read_82599(struct ixgbe_hw *hw, bool *locked, u32 *reg_val)
{
	s32 ret_val;

	*locked = false;
	 /* If LESM is on then we need to hold the SW/FW semaphore. */
	if (ixgbe_verify_lesm_fw_enabled_82599(hw)) {
		ret_val = ixgbe_acquire_swfw_sync(hw,
					IXGBE_GSSR_MAC_CSR_SM);
		if (ret_val != IXGBE_SUCCESS)
			return IXGBE_ERR_SWFW_SYNC;

		*locked = true;
	}

	*reg_val = IXGBE_READ_REG(hw, IXGBE_AUTOC);
	return IXGBE_SUCCESS;
}

/**
 * prot_autoc_write_82599 - Hides MAC differences needed for AUTOC write
 * @hw: pointer to hardware structure
 * @reg_val: value to write to AUTOC
 * @locked: bool to indicate whether the SW/FW lock was already taken by
 *           previous proc_autoc_read_82599.
 *
 * This part (82599) may need to hold the SW/FW lock around all writes to
 * AUTOC. Likewise after a write we need to do a pipeline reset.
 */
s32 ebbrt::IxgbeDriver::prot_autoc_write_82599(struct ixgbe_hw *hw, u32 autoc, bool locked)
{
	s32 ret_val = IXGBE_SUCCESS;

	/* Blocked by MNG FW so bail */
	if (ixgbe_check_reset_blocked(hw))
		goto out;

	/* We only need to get the lock if:
	 *  - We didn't do it already (in the read part of a read-modify-write)
	 *  - LESM is enabled.
	 */
	if (!locked && ixgbe_verify_lesm_fw_enabled_82599(hw)) {
		ret_val = ixgbe_acquire_swfw_sync(hw,
					IXGBE_GSSR_MAC_CSR_SM);
		if (ret_val != IXGBE_SUCCESS)
			return IXGBE_ERR_SWFW_SYNC;

		locked = true;
	}

	IXGBE_WRITE_REG(hw, IXGBE_AUTOC, autoc);
	ret_val = ixgbe_reset_pipeline_82599(hw);

out:
	/* Free the SW/FW semaphore as we either grabbed it here or
	 * already had it when this function was called.
	 */
	if (locked)
	        ixgbe_release_swfw_sync(hw, IXGBE_GSSR_MAC_CSR_SM);

	return ret_val;
}

/**
 *  ixgbe_get_mac_addr_generic - Generic get MAC address
 *  @hw: pointer to hardware structure
 *  @mac_addr: Adapter MAC address
 *
 *  Reads the adapter's MAC address from first Receive Address Register (RAR0)
 *  A reset of the adapter must be performed prior to calling this function
 *  in order for the MAC address to have been loaded from the EEPROM into RAR0
 **/
s32 ebbrt::IxgbeDriver::ixgbe_get_mac_addr_generic(struct ixgbe_hw *hw, u8 *mac_addr)
{
	u32 rar_high;
	u32 rar_low;
	u16 i;

	DEBUGFUNC("ixgbe_get_mac_addr_generic\n");

	rar_high = IXGBE_READ_REG(hw, IXGBE_RAH(0));
	rar_low = IXGBE_READ_REG(hw, IXGBE_RAL(0));
	ebbrt::kprintf("%x %x\n", rar_high, rar_low);
	
	for (i = 0; i < 4; i++)
		mac_addr[i] = (u8)(rar_low >> (i*8));

	for (i = 0; i < 2; i++)
		mac_addr[i+4] = (u8)(rar_high >> (i*8));

	return IXGBE_SUCCESS;
}

/**
 *  ixgbe_validate_mac_addr - Validate MAC address
 *  @mac_addr: pointer to MAC address.
 *
 *  Tests a MAC address to ensure it is a valid Individual Address.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_validate_mac_addr(u8 *mac_addr)
{
	s32 status = IXGBE_SUCCESS;

	DEBUGFUNC("ixgbe_validate_mac_addr\n");

	/* Make sure it is not a multicast address */
	if (IXGBE_IS_MULTICAST(mac_addr)) {
		status = IXGBE_ERR_INVALID_MAC_ADDR;
	/* Not a broadcast address */
	} else if (IXGBE_IS_BROADCAST(mac_addr)) {
		status = IXGBE_ERR_INVALID_MAC_ADDR;
	/* Reject the zero address */
	} else if (mac_addr[0] == 0 && mac_addr[1] == 0 && mac_addr[2] == 0 &&
		   mac_addr[3] == 0 && mac_addr[4] == 0 && mac_addr[5] == 0) {
		status = IXGBE_ERR_INVALID_MAC_ADDR;
	}
	return status;
}

/**
 *  ixgbe_set_vmdq_generic - Associate a VMDq pool index with a rx address
 *  @hw: pointer to hardware struct
 *  @rar: receive address register index to associate with a VMDq index
 *  @vmdq: VMDq pool index
 **/
s32 ebbrt::IxgbeDriver::ixgbe_set_vmdq_generic(struct ixgbe_hw *hw, u32 rar, u32 vmdq)
{
	u32 mpsar;
	u32 rar_entries = hw->mac.num_rar_entries;

	DEBUGFUNC("ixgbe_set_vmdq_generic\n");

	/* Make sure we are using a valid rar index range */
	if (rar >= rar_entries) {
	    ebbrt::kabort(
		"RAR index %d is out of range.\n", rar);
	    return IXGBE_ERR_INVALID_ARGUMENT;
	}

	if (vmdq < 32) {
		mpsar = IXGBE_READ_REG(hw, IXGBE_MPSAR_LO(rar));
		mpsar |= 1 << vmdq;
		IXGBE_WRITE_REG(hw, IXGBE_MPSAR_LO(rar), mpsar);
	} else {
		mpsar = IXGBE_READ_REG(hw, IXGBE_MPSAR_HI(rar));
		mpsar |= 1 << (vmdq - 32);
		IXGBE_WRITE_REG(hw, IXGBE_MPSAR_HI(rar), mpsar);
	}
	return IXGBE_SUCCESS;
}

/**
 *  ixgbe_set_rar_generic - Set Rx address register
 *  @hw: pointer to hardware structure
 *  @index: Receive address register to write
 *  @addr: Address to put into receive address register
 *  @vmdq: VMDq "set" or "pool" index
 *  @enable_addr: set flag that address is active
 *
 *  Puts an ethernet address into a receive address register.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_set_rar_generic(struct ixgbe_hw *hw, u32 index, u8 *addr, u32 vmdq,
			  u32 enable_addr)
{
	u32 rar_low, rar_high;
	u32 rar_entries = hw->mac.num_rar_entries;

	DEBUGFUNC("ixgbe_set_rar_generic\n");

	/* Make sure we are using a valid rar index range */
	if (index >= rar_entries) {
	ebbrt:kabort(
	    "RAR index %d is out of range.\n", index);
	    return IXGBE_ERR_INVALID_ARGUMENT;
	}

	/* setup VMDq pool selection before this RAR gets enabled */
	//hw->mac.ops.set_vmdq(hw, index, vmdq);
	ixgbe_set_vmdq_generic(hw, index, vmdq);

	/*
	 * HW expects these in little endian so we reverse the byte
	 * order from network order (big endian) to little endian
	 */
	rar_low = ((u32)addr[0] |
		   ((u32)addr[1] << 8) |
		   ((u32)addr[2] << 16) |
		   ((u32)addr[3] << 24));
	/*
	 * Some parts put the VMDq setting in the extra RAH bits,
	 * so save everything except the lower 16 bits that hold part
	 * of the address and the address valid bit.
	 */
	rar_high = IXGBE_READ_REG(hw, IXGBE_RAH(index));
	rar_high &= ~(0x0000FFFF | IXGBE_RAH_AV);
	rar_high |= ((u32)addr[4] | ((u32)addr[5] << 8));

	if (enable_addr != 0)
		rar_high |= IXGBE_RAH_AV;

	IXGBE_WRITE_REG(hw, IXGBE_RAL(index), rar_low);
	IXGBE_WRITE_REG(hw, IXGBE_RAH(index), rar_high);

	return IXGBE_SUCCESS;
}

/**
 *  ixgbe_clear_rar_generic - Remove Rx address register
 *  @hw: pointer to hardware structure
 *  @index: Receive address register to write
 *
 *  Clears an ethernet address from a receive address register.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_clear_rar_generic(struct ixgbe_hw *hw, u32 index)
{
	u32 rar_high;
	u32 rar_entries = hw->mac.num_rar_entries;

	DEBUGFUNC("ixgbe_clear_rar_generic\n");

	/* Make sure we are using a valid rar index range */
	if (index >= rar_entries) {
	    ebbrt::kabort(
			     "RAR index %d is out of range.\n", index);
		return IXGBE_ERR_INVALID_ARGUMENT;
	}

	/*
	 * Some parts put the VMDq setting in the extra RAH bits,
	 * so save everything except the lower 16 bits that hold part
	 * of the address and the address valid bit.
	 */
	rar_high = IXGBE_READ_REG(hw, IXGBE_RAH(index));
	rar_high &= ~(0x0000FFFF | IXGBE_RAH_AV);

	IXGBE_WRITE_REG(hw, IXGBE_RAL(index), 0);
	IXGBE_WRITE_REG(hw, IXGBE_RAH(index), rar_high);

	/* clear VMDq pool/queue selection for this RAR */
	//hw->mac.ops.clear_vmdq(hw, index, IXGBE_CLEAR_VMDQ_ALL);
	ixgbe_clear_vmdq_generic(hw, index, IXGBE_CLEAR_VMDQ_ALL);

	return IXGBE_SUCCESS;
}

/**
 *  ixgbe_clear_vmdq_generic - Disassociate a VMDq pool index from a rx address
 *  @hw: pointer to hardware struct
 *  @rar: receive address register index to disassociate
 *  @vmdq: VMDq pool index to remove from the rar
 **/
s32 ebbrt::IxgbeDriver::ixgbe_clear_vmdq_generic(struct ixgbe_hw *hw, u32 rar, u32 vmdq)
{
	u32 mpsar_lo, mpsar_hi;
	u32 rar_entries = hw->mac.num_rar_entries;

	DEBUGFUNC("ixgbe_clear_vmdq_generic\n");

	/* Make sure we are using a valid rar index range */
	if (rar >= rar_entries) {
	    ebbrt::kabort(
			     "RAR index %d is out of range.\n", rar);
		return IXGBE_ERR_INVALID_ARGUMENT;
	}

	mpsar_lo = IXGBE_READ_REG(hw, IXGBE_MPSAR_LO(rar));
	mpsar_hi = IXGBE_READ_REG(hw, IXGBE_MPSAR_HI(rar));

	if (IXGBE_REMOVED(hw->hw_addr))
		goto done;

	if (!mpsar_lo && !mpsar_hi)
		goto done;

	if (vmdq == IXGBE_CLEAR_VMDQ_ALL) {
		if (mpsar_lo) {
			IXGBE_WRITE_REG(hw, IXGBE_MPSAR_LO(rar), 0);
			mpsar_lo = 0;
		}
		if (mpsar_hi) {
			IXGBE_WRITE_REG(hw, IXGBE_MPSAR_HI(rar), 0);
			mpsar_hi = 0;
		}
	} else if (vmdq < 32) {
		mpsar_lo &= ~(1 << vmdq);
		IXGBE_WRITE_REG(hw, IXGBE_MPSAR_LO(rar), mpsar_lo);
	} else {
		mpsar_hi &= ~(1 << (vmdq - 32));
		IXGBE_WRITE_REG(hw, IXGBE_MPSAR_HI(rar), mpsar_hi);
	}

	/* was that the last pool using this rar? */
	if (mpsar_lo == 0 && mpsar_hi == 0 && rar != 0)
	    //hw->mac.ops.clear_rar(hw, rar);
	    ixgbe_clear_rar_generic(hw, rar);
done:
	return IXGBE_SUCCESS;
}

/**
 *  ixgbe_init_uta_tables_generic - Initialize the Unicast Table Array
 *  @hw: pointer to hardware structure
 **/
s32 ebbrt::IxgbeDriver::ixgbe_init_uta_tables_generic(struct ixgbe_hw *hw)
{
	int i;

	DEBUGFUNC("ixgbe_init_uta_tables_generic");
	DEBUGOUT(" Clearing UTA\n");

	for (i = 0; i < 128; i++)
		IXGBE_WRITE_REG(hw, IXGBE_UTA(i), 0);

	return IXGBE_SUCCESS;
}

/**
 *  ixgbe_init_rx_addrs_generic - Initializes receive address filters.
 *  @hw: pointer to hardware structure
 *
 *  Places the MAC address in receive address register 0 and clears the rest
 *  of the receive address registers. Clears the multicast table. Assumes
 *  the receiver is in reset when the routine is called.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_init_rx_addrs_generic(struct ixgbe_hw *hw)
{
	u32 i;
	u32 rar_entries = hw->mac.num_rar_entries;

	DEBUGFUNC("ixgbe_init_rx_addrs_generic\n");

	/*
	 * If the current mac address is valid, assume it is a software override
	 * to the permanent address.
	 * Otherwise, use the permanent address from the eeprom.
	 */
	//if (ixgbe_validate_mac_addr(hw->mac.addr) ==
	//  IXGBE_ERR_INVALID_MAC_ADDR) {
		/* Get the MAC address from the RAR0 for later reference */
	    //hw->mac.ops.get_mac_addr(hw, hw->mac.addr);
	ixgbe_get_mac_addr_generic(hw, hw->mac.addr);
	
	DEBUGOUT3(" Keeping Current RAR0 Addr =%.2X %.2X %.2X ",
		  hw->mac.addr[0], hw->mac.addr[1],
		  hw->mac.addr[2]);
	DEBUGOUT3("%.2X %.2X %.2X\n", hw->mac.addr[3],
		  hw->mac.addr[4], hw->mac.addr[5]);
	// } else {
	// 	/* Setup the receive address. */
	// 	DEBUGOUT("Overriding MAC Address in RAR[0]\n");
	// 	DEBUGOUT3(" New MAC Addr = %.2X %.2X %.2X ",
	// 		  hw->mac.addr[0], hw->mac.addr[1],
	// 		  hw->mac.addr[2]);
	// 	DEBUGOUT3("%.2X %.2X %.2X\n", hw->mac.addr[3],
	// 		  hw->mac.addr[4], hw->mac.addr[5]);

	// 	//hw->mac.ops.set_rar(hw, 0, hw->mac.addr, 0, IXGBE_RAH_AV);
	// 	ixgbe_set_rar_generic(hw, 0, hw->mac.addr, 0, IXGBE_RAH_AV);
	// }

	/* clear VMDq pool/queue selection for RAR 0 */
	//hw->mac.ops.clear_vmdq(hw, 0, IXGBE_CLEAR_VMDQ_ALL);
	ixgbe_clear_vmdq_generic(hw, 0, IXGBE_CLEAR_VMDQ_ALL);

	hw->addr_ctrl.overflow_promisc = 0;

	hw->addr_ctrl.rar_used_count = 1;

	/* Zero out the other receive addresses. */
	DEBUGOUT1("Clearing RAR[1-%d]\n", rar_entries - 1);
	for (i = 1; i < rar_entries; i++) {
		IXGBE_WRITE_REG(hw, IXGBE_RAL(i), 0);
		IXGBE_WRITE_REG(hw, IXGBE_RAH(i), 0);
	}

	/* Clear the MTA */
	hw->addr_ctrl.mta_in_use = 0;
	IXGBE_WRITE_REG(hw, IXGBE_MCSTCTRL, hw->mac.mc_filter_type);

	DEBUGOUT(" Clearing MTA\n");
	for (i = 0; i < hw->mac.mcft_size; i++)
		IXGBE_WRITE_REG(hw, IXGBE_MTA(i), 0);

	ixgbe_init_uta_tables_generic(hw);

	return IXGBE_SUCCESS;
}

/**
 *  ixgbe_get_san_mac_addr_offset - Get SAN MAC address offset from the EEPROM
 *  @hw: pointer to hardware structure
 *  @san_mac_offset: SAN MAC address offset
 *
 *  This function will read the EEPROM location for the SAN MAC address
 *  pointer, and returns the value at that location.  This is used in both
 *  get and set mac_addr routines.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_get_san_mac_addr_offset(struct ixgbe_hw *hw,
					 u16 *san_mac_offset)
{
	s32 ret_val;

	DEBUGFUNC("ixgbe_get_san_mac_addr_offset\n");

	/*
	 * First read the EEPROM pointer to see if the MAC addresses are
	 * available.
	 */
	//ret_val = hw->eeprom.ops.read(hw, IXGBE_SAN_MAC_ADDR_PTR,
	ret_val = ixgbe_read_eeprom_82599(hw, IXGBE_SAN_MAC_ADDR_PTR,
					  san_mac_offset);
	if (ret_val) {
	    //ebbrt::kabort(
	    //		      "eeprom at offset %d failed",
	    //		      IXGBE_SAN_MAC_ADDR_PTR);
	}

	return ret_val;
}

/**
 *  ixgbe_get_san_mac_addr_generic - SAN MAC address retrieval from the EEPROM
 *  @hw: pointer to hardware structure
 *  @san_mac_addr: SAN MAC address
 *
 *  Reads the SAN MAC address from the EEPROM, if it's available.  This is
 *  per-port, so set_lan_id() must be called before reading the addresses.
 *  set_lan_id() is called by identify_sfp(), but this cannot be relied
 *  upon for non-SFP connections, so we must call it here.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_get_san_mac_addr_generic(struct ixgbe_hw *hw, u8 *san_mac_addr)
{
	u16 san_mac_data, san_mac_offset;
	u8 i;
	s32 ret_val;

	DEBUGFUNC("ixgbe_get_san_mac_addr_generic\n");

	/*
	 * First read the EEPROM pointer to see if the MAC addresses are
	 * available.  If they're not, no point in calling set_lan_id() here.
	 */
	ret_val = ixgbe_get_san_mac_addr_offset(hw, &san_mac_offset);
	if (ret_val || san_mac_offset == 0 || san_mac_offset == 0xFFFF)
		goto san_mac_addr_out;

	/* make sure we know which port we need to program */
	//hw->mac.ops.set_lan_id(hw);
	ixgbe_set_lan_id_multi_port_pcie(hw);
	/* apply the port offset to the address offset */
	(hw->bus.func) ? (san_mac_offset += IXGBE_SAN_MAC_ADDR_PORT1_OFFSET) :
			 (san_mac_offset += IXGBE_SAN_MAC_ADDR_PORT0_OFFSET);
	for (i = 0; i < 3; i++) {
	    //ret_val = hw->eeprom.ops.read(hw, san_mac_offset,
	    ret_val = ixgbe_read_eeprom_82599(hw, san_mac_offset,
					      &san_mac_data);
		if (ret_val) {
		    //ebbrt::kabort(
		    //	"eeprom read at offset %d failed",
		    //		      san_mac_offset);
			goto san_mac_addr_out;
		}
		san_mac_addr[i * 2] = (u8)(san_mac_data);
		san_mac_addr[i * 2 + 1] = (u8)(san_mac_data >> 8);
		san_mac_offset++;
	}
	return IXGBE_SUCCESS;

san_mac_addr_out:
	/*
	 * No addresses available in this EEPROM.  It's not an
	 * error though, so just wipe the local address and return.
	 */
	for (i = 0; i < 6; i++)
		san_mac_addr[i] = 0xFF;
	return IXGBE_SUCCESS;
}

/**
 *  ixgbe_get_wwn_prefix_generic - Get alternative WWNN/WWPN prefix from
 *  the EEPROM
 *  @hw: pointer to hardware structure
 *  @wwnn_prefix: the alternative WWNN prefix
 *  @wwpn_prefix: the alternative WWPN prefix
 *
 *  This function will read the EEPROM from the alternative SAN MAC address
 *  block to check the support for the alternative WWNN/WWPN prefix support.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_get_wwn_prefix_generic(struct ixgbe_hw *hw, u16 *wwnn_prefix,
				 u16 *wwpn_prefix)
{
	u16 offset, caps;
	u16 alt_san_mac_blk_offset;

	DEBUGFUNC("ixgbe_get_wwn_prefix_generic\n");

	/* clear output first */
	*wwnn_prefix = 0xFFFF;
	*wwpn_prefix = 0xFFFF;

	/* check if alternative SAN MAC is supported */
	offset = IXGBE_ALT_SAN_MAC_ADDR_BLK_PTR;
	//if (hw->eeprom.ops.read(hw, offset, &alt_san_mac_blk_offset))
	if (ixgbe_read_eeprom_82599(hw, offset, &alt_san_mac_blk_offset))
	    goto wwn_prefix_err;

	if ((alt_san_mac_blk_offset == 0) ||
	    (alt_san_mac_blk_offset == 0xFFFF))
		goto wwn_prefix_out;

	/* check capability in alternative san mac address block */
	offset = alt_san_mac_blk_offset + IXGBE_ALT_SAN_MAC_ADDR_CAPS_OFFSET;
	//if (hw->eeprom.ops.read(hw, offset, &caps))
	if (ixgbe_read_eeprom_82599(hw, offset, &caps))
		goto wwn_prefix_err;
	
	if (!(caps & IXGBE_ALT_SAN_MAC_ADDR_CAPS_ALTWWN))
		goto wwn_prefix_out;

	/* get the corresponding prefix for WWNN/WWPN */
	offset = alt_san_mac_blk_offset + IXGBE_ALT_SAN_MAC_ADDR_WWNN_OFFSET;
	//if (hw->eeprom.ops.read(hw, offset, wwnn_prefix)) 
	if(ixgbe_read_eeprom_82599(hw, offset, wwnn_prefix)) 
	{
	    //ebbrt::kabort(
	    //		      "eeprom read at offset %d failed", offset);
	}

	offset = alt_san_mac_blk_offset + IXGBE_ALT_SAN_MAC_ADDR_WWPN_OFFSET;
	//if (hw->eeprom.ops.read(hw, offset, wwpn_prefix))
	if(ixgbe_read_eeprom_82599(hw, offset, wwpn_prefix))
	    goto wwn_prefix_err;

wwn_prefix_out:
	return IXGBE_SUCCESS;

wwn_prefix_err:
	//ebbrt::kabort(
	//	      "eeprom read at offset %d failed", offset);
	return IXGBE_SUCCESS;
}

/**
 *  ixgbe_get_sfp_init_sequence_offsets - Provides offset of PHY init sequence
 *  @hw: pointer to hardware structure
 *  @list_offset: offset to the SFP ID list
 *  @data_offset: offset to the SFP data block
 *
 *  Checks the MAC's EEPROM to see if it supports a given SFP+ module type, if
 *  so it returns the offsets to the phy init sequence block.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_get_sfp_init_sequence_offsets(struct ixgbe_hw *hw,
					u16 *list_offset,
					u16 *data_offset)
{
	u16 sfp_id;
	u16 sfp_type = hw->phy.sfp_type;

	DEBUGFUNC("ixgbe_get_sfp_init_sequence_offsets\n");

	if (hw->phy.sfp_type == ixgbe_sfp_type_unknown)
		return IXGBE_ERR_SFP_NOT_SUPPORTED;

	if (hw->phy.sfp_type == ixgbe_sfp_type_not_present)
		return IXGBE_ERR_SFP_NOT_PRESENT;

	if ((hw->device_id == IXGBE_DEV_ID_82598_SR_DUAL_PORT_EM) &&
	    (hw->phy.sfp_type == ixgbe_sfp_type_da_cu))
		return IXGBE_ERR_SFP_NOT_SUPPORTED;

	/*
	 * Limiting active cables and 1G Phys must be initialized as
	 * SR modules
	 */
	if (sfp_type == ixgbe_sfp_type_da_act_lmt_core0 ||
	    sfp_type == ixgbe_sfp_type_1g_lx_core0 ||
	    sfp_type == ixgbe_sfp_type_1g_cu_core0 ||
	    sfp_type == ixgbe_sfp_type_1g_sx_core0)
		sfp_type = ixgbe_sfp_type_srlr_core0;
	else if (sfp_type == ixgbe_sfp_type_da_act_lmt_core1 ||
		 sfp_type == ixgbe_sfp_type_1g_lx_core1 ||
		 sfp_type == ixgbe_sfp_type_1g_cu_core1 ||
		 sfp_type == ixgbe_sfp_type_1g_sx_core1)
		sfp_type = ixgbe_sfp_type_srlr_core1;

	/* Read offset to PHY init contents */
	//if (hw->eeprom.ops.read(hw, IXGBE_PHY_INIT_OFFSET_NL, list_offset)) {
	if(ixgbe_read_eeprom_82599(hw, IXGBE_PHY_INIT_OFFSET_NL, list_offset)) {
	    //ebbrt::kabort(
	    //	"eeprom read at offset %d failed",
	    //	IXGBE_PHY_INIT_OFFSET_NL);
	    return IXGBE_ERR_SFP_NO_INIT_SEQ_PRESENT;
	}

	if ((!*list_offset) || (*list_offset == 0xFFFF))
		return IXGBE_ERR_SFP_NO_INIT_SEQ_PRESENT;

	/* Shift offset to first ID word */
	(*list_offset)++;

	/*
	 * Find the matching SFP ID in the EEPROM
	 * and program the init sequence
	 */
	//if (hw->eeprom.ops.read(hw, *list_offset, &sfp_id))
	if(ixgbe_read_eeprom_82599(hw, *list_offset, &sfp_id))
		goto err_phy;

	while (sfp_id != IXGBE_PHY_INIT_END_NL) {
		if (sfp_id == sfp_type) {
			(*list_offset)++;
			//if (hw->eeprom.ops.read(hw, *list_offset, data_offset))
			if(ixgbe_read_eeprom_82599(hw, *list_offset, data_offset))
				goto err_phy;
			if ((!*data_offset) || (*data_offset == 0xFFFF)) {
				DEBUGOUT("SFP+ module not supported\n");
				return IXGBE_ERR_SFP_NOT_SUPPORTED;
			} else {
				break;
			}
		} else {
			(*list_offset) += 2;
			//if (hw->eeprom.ops.read(hw, *list_offset, &sfp_id))
			if(ixgbe_read_eeprom_82599(hw, *list_offset, &sfp_id))
				goto err_phy;
		}
	}

	if (sfp_id == IXGBE_PHY_INIT_END_NL) {
		DEBUGOUT("No matching SFP+ module found\n");
		return IXGBE_ERR_SFP_NOT_SUPPORTED;
	}

	return IXGBE_SUCCESS;

err_phy:
	//ebbrt::kabort(
	//	      "eeprom read at offset %d failed", *list_offset);
	return IXGBE_ERR_PHY;
}

s32 ebbrt::IxgbeDriver::ixgbe_setup_sfp_modules_82599(struct ixgbe_hw *hw)
{
	s32 ret_val = IXGBE_SUCCESS;
	u16 list_offset, data_offset, data_value;

	DEBUGFUNC("ixgbe_setup_sfp_modules_82599");

	if (hw->phy.sfp_type != ixgbe_sfp_type_unknown) {
		ixgbe_init_mac_link_ops_82599(hw);

		hw->phy.ops.reset = NULL;

		ret_val = ixgbe_get_sfp_init_sequence_offsets(hw, &list_offset,
							      &data_offset);
		if (ret_val != IXGBE_SUCCESS)
			goto setup_sfp_out;

		/* PHY config will finish before releasing the semaphore */
		ret_val = ixgbe_acquire_swfw_sync(hw,
							IXGBE_GSSR_MAC_CSR_SM);
		if (ret_val != IXGBE_SUCCESS) {
			ret_val = IXGBE_ERR_SWFW_SYNC;
			goto setup_sfp_out;
		}

		//if (hw->eeprom.ops.read(hw, ++data_offset, &data_value))
		if(ixgbe_read_eeprom_82599(hw, ++data_offset, &data_value))
		    goto setup_sfp_err;
		
		while (data_value != 0xffff) {
			IXGBE_WRITE_REG(hw, IXGBE_CORECTL, data_value);
			IXGBE_WRITE_FLUSH(hw);
			//if (hw->eeprom.ops.read(hw, ++data_offset, &data_value))
			if(ixgbe_read_eeprom_82599(hw, ++data_offset, &data_value))
			    goto setup_sfp_err;
		}

		/* Release the semaphore */
	        ixgbe_release_swfw_sync(hw, IXGBE_GSSR_MAC_CSR_SM);
		/* Delay obtaining semaphore again to allow FW access
		 * prot_autoc_write uses the semaphore too.
		 */
		msec_delay(hw->eeprom.semaphore_delay);

		/* Restart DSP and set SFI mode */
		//ret_val = hw->mac.ops.prot_autoc_write(hw,
		ret_val = prot_autoc_write_82599(hw,
						 hw->mac.orig_autoc | IXGBE_AUTOC_LMS_10G_SERIAL,
						 false);

		if (ret_val) {
			DEBUGOUT("sfp module setup not complete\n");
			ret_val = IXGBE_ERR_SFP_SETUP_NOT_COMPLETE;
			goto setup_sfp_out;
		}

	}

setup_sfp_out:
	return ret_val;

setup_sfp_err:
	/* Release the semaphore */
        ixgbe_release_swfw_sync(hw, IXGBE_GSSR_MAC_CSR_SM);
	/* Delay obtaining semaphore again to allow FW access */
	msec_delay(hw->eeprom.semaphore_delay);
	//ebbrt::kabort(
	//  "eeprom read at offset %d failed", data_offset);
	return IXGBE_ERR_PHY;
}

/**
 *  ixgbe_reset_hw_82599 - Perform hardware reset
 *  @hw: pointer to hardware structure
 *
 *  Resets the hardware by resetting the transmit and receive units, masks
 *  and clears all interrupts, perform a PHY reset, and perform a link (MAC)
 *  reset.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_reset_hw_82599(struct ixgbe_hw *hw)
{
	ixgbe_link_speed link_speed;
	s32 status;
	u32 ctrl = 0;
	u32 i, autoc, autoc2;
	u32 curr_lms;
	bool link_up = false;

	DEBUGFUNC("ixgbe_reset_hw_82599\n");

	/* Call adapter stop to disable tx/rx and clear interrupts */
	//status = hw->mac.ops.stop_adapter(hw);
	status = ixgbe_stop_adapter_generic(hw);
	if (status != IXGBE_SUCCESS) {
	    ebbrt::kabort("ixgbe_stop_adapter_generic failed\n");
		goto reset_hw_out;
	}

	// /* flush pending Tx transactions */
 	ixgbe_clear_tx_pending(hw);

 	/* PHY ops must be identified and initialized prior to reset */

 	/* Identify PHY and related function pointers */
	status = ixgbe_init_phy_ops_82599(hw);
	ebbrt::kprintf("ixgbe_init_phy_ops_82599(hw) finished\n");
 	//status = hw->phy.ops.init(hw);

 	if (status == IXGBE_ERR_SFP_NOT_SUPPORTED)
	    goto reset_hw_out;

 	/* Setup SFP module if there is one present. */
 	if (hw->phy.sfp_setup_needed) {
	    ebbrt::kprintf("sfp_setup_needed\n");
	    //status = hw->mac.ops.setup_sfp(hw);
	    status = ixgbe_setup_sfp_modules_82599(hw);
	    hw->phy.sfp_setup_needed = false;
 	}

 	if (status == IXGBE_ERR_SFP_NOT_SUPPORTED)
	    goto reset_hw_out;

 	/* Reset PHY */
 	if (hw->phy.reset_disable == false && hw->phy.ops.reset != NULL) {
	    GlobalReset();
	    //ixgbe_reset_phy_generic(hw);
	    //hw->phy.ops.reset(hw);
	}
	
 	/* remember AUTOC from before we reset */
 	curr_lms = IXGBE_READ_REG(hw, IXGBE_AUTOC) & IXGBE_AUTOC_LMS_MASK;

 mac_reset_top:
	/*
	 * Issue global reset to the MAC.  Needs to be SW reset if link is up.
	 * If link reset is used when link is up, it might reset the PHY when
	 * mng is using it.  If link is down or the flag to force full link
	 * reset is set, then perform link reset.
	 */
 	ctrl = IXGBE_CTRL_LNK_RST;
 	if (!hw->force_full_reset) {
	    //hw->mac.ops.check_link(hw, &link_speed, &link_up, false);
	    ixgbe_check_mac_link_generic(hw, &link_speed, &link_up, false);
	    if (link_up)
		ctrl = IXGBE_CTRL_RST;
 	}

 	ctrl |= IXGBE_READ_REG(hw, IXGBE_CTRL);
 	IXGBE_WRITE_REG(hw, IXGBE_CTRL, ctrl);
 	IXGBE_WRITE_FLUSH(hw);

 	/* Poll for reset bit to self-clear meaning reset is complete */
 	for (i = 0; i < 10; i++) {
 		usec_delay(1);
 		ctrl = IXGBE_READ_REG(hw, IXGBE_CTRL);
 		if (!(ctrl & IXGBE_CTRL_RST_MASK))
 			break;
 	}

 	if (ctrl & IXGBE_CTRL_RST_MASK) {
 		status = IXGBE_ERR_RESET_FAILED;
 		DEBUGOUT("Reset polling failed to complete.\n");
 	}

 	msec_delay(50);

 	/*
 	 * Double resets are required for recovery from certain error
 	 * conditions.  Between resets, it is necessary to stall to
 	 * allow time for any pending HW events to complete.
 	 */
 	if (hw->mac.flags & IXGBE_FLAGS_DOUBLE_RESET_REQUIRED) {
 		hw->mac.flags &= ~IXGBE_FLAGS_DOUBLE_RESET_REQUIRED;
 		goto mac_reset_top;
 	}

 	/*
 	 * Store the original AUTOC/AUTOC2 values if they have not been
 	 * stored off yet.  Otherwise restore the stored original
 	 * values since the reset operation sets back to defaults.
 	 */
 	autoc = IXGBE_READ_REG(hw, IXGBE_AUTOC);
 	autoc2 = IXGBE_READ_REG(hw, IXGBE_AUTOC2);

 	/* Enable link if disabled in NVM */
 	if (autoc2 & IXGBE_AUTOC2_LINK_DISABLE_MASK) {
 		autoc2 &= ~IXGBE_AUTOC2_LINK_DISABLE_MASK;
 		IXGBE_WRITE_REG(hw, IXGBE_AUTOC2, autoc2);
 		IXGBE_WRITE_FLUSH(hw);
 	}

 	if (hw->mac.orig_link_settings_stored == false) {
 		hw->mac.orig_autoc = autoc;
 		hw->mac.orig_autoc2 = autoc2;
 		hw->mac.orig_link_settings_stored = true;
 	} else {

		/* If MNG FW is running on a multi-speed device that
		 * doesn't autoneg with out driver support we need to
		 * leave LMS in the state it was before we MAC reset.
		 * Likewise if we support WoL we don't want change the
		 * LMS state.
		 */
 		if ((hw->phy.multispeed_fiber && ixgbe_mng_enabled(hw)) ||
 		    hw->wol_enabled)
 			hw->mac.orig_autoc =
 				(hw->mac.orig_autoc & ~IXGBE_AUTOC_LMS_MASK) |
 				curr_lms;

 		if (autoc != hw->mac.orig_autoc) {
		    //status = hw->mac.ops.prot_autoc_write(hw,
		    status = prot_autoc_write_82599(hw,
						    hw->mac.orig_autoc,
						    false);
 			if (status != IXGBE_SUCCESS)
 				goto reset_hw_out;
 		}

		if ((autoc2 & IXGBE_AUTOC2_UPPER_MASK) !=
		    (hw->mac.orig_autoc2 & IXGBE_AUTOC2_UPPER_MASK)) {
			autoc2 &= ~IXGBE_AUTOC2_UPPER_MASK;
			autoc2 |= (hw->mac.orig_autoc2 &
				   IXGBE_AUTOC2_UPPER_MASK);
			IXGBE_WRITE_REG(hw, IXGBE_AUTOC2, autoc2);
		}
	}

 	/* Store the permanent mac address */	
	ixgbe_get_mac_addr_generic(hw, hw->mac.perm_addr);
 	//hw->mac.ops.get_mac_addr(hw, hw->mac.perm_addr);

	/*
	 * Store MAC address from RAR0, clear receive address registers, and
	 * clear the multicast table.  Also reset num_rar_entries to 128,
	 * since we modify this value when programming the SAN MAC address.
	 */
	hw->mac.num_rar_entries = 128;
	ixgbe_init_rx_addrs_generic(hw);
	//hw->mac.ops.init_rx_addrs(hw);

	/* Store the permanent SAN mac address */
	//hw->mac.ops.get_san_mac_addr(hw, hw->mac.san_addr);
	// ixgbe_get_san_mac_addr_generic(hw, hw->mac.san_addr);

 	// /* Add the SAN MAC address to the RAR only if it's a valid address */
 	// if (ixgbe_validate_mac_addr(hw->mac.san_addr) == 0) {
 	// 	/* Save the SAN MAC RAR index */
 	// 	hw->mac.san_mac_rar_index = hw->mac.num_rar_entries - 1;

 	// 	//hw->mac.ops.set_rar(hw, hw->mac.san_mac_rar_index,
	// 	ixgbe_set_rar_generic(hw, hw->mac.san_mac_rar_index,
 	// 			    hw->mac.san_addr, 0, IXGBE_RAH_AV);

 	// 	/* clear VMDq pool/queue selection for this RAR */
 	// 	//hw->mac.ops.clear_vmdq(hw, hw->mac.san_mac_rar_index,
	// 	ixgbe_clear_vmdq_generic(hw, hw->mac.san_mac_rar_index,
 	// 			       IXGBE_CLEAR_VMDQ_ALL);

 	// 	/* Reserve the last RAR for the SAN MAC address */
 	// 	hw->mac.num_rar_entries--;
 	// }

 	/* Store the alternative WWNN/WWPN prefix */
 	//hw->mac.ops.get_wwn_prefix(hw, &hw->mac.wwnn_prefix,
 	//			   &hw->mac.wwpn_prefix);
	//ixgbe_get_wwn_prefix_generic(hw, &hw->mac.wwnn_prefix,
	//			     &hw->mac.wwpn_prefix);

reset_hw_out:
	return status;
}

/**
 *  ixgbe_clear_vfta_generic - Clear VLAN filter table
 *  @hw: pointer to hardware structure
 *
 *  Clears the VLAN filer table, and the VMDq index associated with the filter
 **/
s32 ebbrt::IxgbeDriver::ixgbe_clear_vfta_generic(struct ixgbe_hw *hw)
{
	u32 offset;

	DEBUGFUNC("ixgbe_clear_vfta_generic\n");

	ebbrt::kprintf("%d\n", hw->mac.vft_size);
	for (offset = 0; offset < hw->mac.vft_size; offset++)
		IXGBE_WRITE_REG(hw, IXGBE_VFTA(offset), 0);

	for (offset = 0; offset < IXGBE_VLVF_ENTRIES; offset++) {
		IXGBE_WRITE_REG(hw, IXGBE_VLVF(offset), 0);
		IXGBE_WRITE_REG(hw, IXGBE_VLVFB(offset * 2), 0);
		IXGBE_WRITE_REG(hw, IXGBE_VLVFB(offset * 2 + 1), 0);
	}

	return IXGBE_SUCCESS;
}

/**
 *  ixgbe_clear_hw_cntrs_generic - Generic clear hardware counters
 *  @hw: pointer to hardware structure
 *
 *  Clears all hardware statistics counters by reading them from the hardware
 *  Statistics counters are clear on read.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_clear_hw_cntrs_generic(struct ixgbe_hw *hw)
{
	u16 i = 0;

	DEBUGFUNC("ixgbe_clear_hw_cntrs_generic\n");

	IXGBE_READ_REG(hw, IXGBE_CRCERRS);
	IXGBE_READ_REG(hw, IXGBE_ILLERRC);
	IXGBE_READ_REG(hw, IXGBE_ERRBC);
	IXGBE_READ_REG(hw, IXGBE_MSPDC);
	for (i = 0; i < 8; i++)
		IXGBE_READ_REG(hw, IXGBE_MPC(i));

	IXGBE_READ_REG(hw, IXGBE_MLFC);
	IXGBE_READ_REG(hw, IXGBE_MRFC);
	IXGBE_READ_REG(hw, IXGBE_RLEC);
	IXGBE_READ_REG(hw, IXGBE_LXONTXC);
	IXGBE_READ_REG(hw, IXGBE_LXOFFTXC);
	if (hw->mac.type >= ixgbe_mac_82599EB) {
		IXGBE_READ_REG(hw, IXGBE_LXONRXCNT);
		IXGBE_READ_REG(hw, IXGBE_LXOFFRXCNT);
	} else {
		IXGBE_READ_REG(hw, IXGBE_LXONRXC);
		IXGBE_READ_REG(hw, IXGBE_LXOFFRXC);
	}

	for (i = 0; i < 8; i++) {
		IXGBE_READ_REG(hw, IXGBE_PXONTXC(i));
		IXGBE_READ_REG(hw, IXGBE_PXOFFTXC(i));
		if (hw->mac.type >= ixgbe_mac_82599EB) {
			IXGBE_READ_REG(hw, IXGBE_PXONRXCNT(i));
			IXGBE_READ_REG(hw, IXGBE_PXOFFRXCNT(i));
		} else {
			IXGBE_READ_REG(hw, IXGBE_PXONRXC(i));
			IXGBE_READ_REG(hw, IXGBE_PXOFFRXC(i));
		}
	}
	if (hw->mac.type >= ixgbe_mac_82599EB)
		for (i = 0; i < 8; i++)
			IXGBE_READ_REG(hw, IXGBE_PXON2OFFCNT(i));
	IXGBE_READ_REG(hw, IXGBE_PRC64);
	IXGBE_READ_REG(hw, IXGBE_PRC127);
	IXGBE_READ_REG(hw, IXGBE_PRC255);
	IXGBE_READ_REG(hw, IXGBE_PRC511);
	IXGBE_READ_REG(hw, IXGBE_PRC1023);
	IXGBE_READ_REG(hw, IXGBE_PRC1522);
	IXGBE_READ_REG(hw, IXGBE_GPRC);
	IXGBE_READ_REG(hw, IXGBE_BPRC);
	IXGBE_READ_REG(hw, IXGBE_MPRC);
	IXGBE_READ_REG(hw, IXGBE_GPTC);
	IXGBE_READ_REG(hw, IXGBE_GORCL);
	IXGBE_READ_REG(hw, IXGBE_GORCH);
	IXGBE_READ_REG(hw, IXGBE_GOTCL);
	IXGBE_READ_REG(hw, IXGBE_GOTCH);
	if (hw->mac.type == ixgbe_mac_82598EB)
		for (i = 0; i < 8; i++)
			IXGBE_READ_REG(hw, IXGBE_RNBC(i));
	IXGBE_READ_REG(hw, IXGBE_RUC);
	IXGBE_READ_REG(hw, IXGBE_RFC);
	IXGBE_READ_REG(hw, IXGBE_ROC);
	IXGBE_READ_REG(hw, IXGBE_RJC);
	IXGBE_READ_REG(hw, IXGBE_MNGPRC);
	IXGBE_READ_REG(hw, IXGBE_MNGPDC);
	IXGBE_READ_REG(hw, IXGBE_MNGPTC);
	IXGBE_READ_REG(hw, IXGBE_TORL);
	IXGBE_READ_REG(hw, IXGBE_TORH);
	IXGBE_READ_REG(hw, IXGBE_TPR);
	IXGBE_READ_REG(hw, IXGBE_TPT);
	IXGBE_READ_REG(hw, IXGBE_PTC64);
	IXGBE_READ_REG(hw, IXGBE_PTC127);
	IXGBE_READ_REG(hw, IXGBE_PTC255);
	IXGBE_READ_REG(hw, IXGBE_PTC511);
	IXGBE_READ_REG(hw, IXGBE_PTC1023);
	IXGBE_READ_REG(hw, IXGBE_PTC1522);
	IXGBE_READ_REG(hw, IXGBE_MPTC);
	IXGBE_READ_REG(hw, IXGBE_BPTC);
	for (i = 0; i < 16; i++) {
		IXGBE_READ_REG(hw, IXGBE_QPRC(i));
		IXGBE_READ_REG(hw, IXGBE_QPTC(i));
		if (hw->mac.type >= ixgbe_mac_82599EB) {
			IXGBE_READ_REG(hw, IXGBE_QBRC_L(i));
			IXGBE_READ_REG(hw, IXGBE_QBRC_H(i));
			IXGBE_READ_REG(hw, IXGBE_QBTC_L(i));
			IXGBE_READ_REG(hw, IXGBE_QBTC_H(i));
			IXGBE_READ_REG(hw, IXGBE_QPRDC(i));
		} else {
			IXGBE_READ_REG(hw, IXGBE_QBRC(i));
			IXGBE_READ_REG(hw, IXGBE_QBTC(i));
		}
	}

	return IXGBE_SUCCESS;
}

/**
 *  ixgbe_setup_fc_generic - Set up flow control
 *  @hw: pointer to hardware structure
 *
 *  Called at init time to set up flow control.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_setup_fc_generic(struct ixgbe_hw *hw)
{
	s32 ret_val = IXGBE_SUCCESS;
	u32 reg = 0, reg_bp = 0;
	u16 reg_cu = 0;
	bool locked = false;

	DEBUGFUNC("ixgbe_setup_fc_generic\n");

	/* Validate the requested mode */
	if (hw->fc.strict_ieee && hw->fc.requested_mode == ixgbe_fc_rx_pause) {
	    ebbrt::kprintf(
			   "ixgbe_fc_rx_pause not valid in strict IEEE mode\n");
		ret_val = IXGBE_ERR_INVALID_LINK_SETTINGS;
		goto out;
	}

	/*
	 * 10gig parts do not have a word in the EEPROM to determine the
	 * default flow control setting, so we explicitly set it to full.
	 */
	if (hw->fc.requested_mode == ixgbe_fc_default)
		hw->fc.requested_mode = ixgbe_fc_full;

	/*
	 * Set up the 1G and 10G flow control advertisement registers so the
	 * HW will be able to do fc autoneg once the cable is plugged in.  If
	 * we link at 10G, the 1G advertisement is harmless and vice versa.
	 */
	switch (hw->phy.media_type) {
	case ixgbe_media_type_backplane:
		/* some MAC's need RMW protection on AUTOC */
		//ret_val = hw->mac.ops.prot_autoc_read(hw, &locked, &reg_bp);
	    ret_val = prot_autoc_read_82599(hw, &locked, &reg_bp);
		if (ret_val != IXGBE_SUCCESS)
			goto out;

		/* only backplane uses autoc so fall though */
	case ixgbe_media_type_fiber_qsfp:
	case ixgbe_media_type_fiber:
		reg = IXGBE_READ_REG(hw, IXGBE_PCS1GANA);

		break;
		/*case ixgbe_media_type_copper:
		hw->phy.ops.read_reg(hw, IXGBE_MDIO_AUTO_NEG_ADVT,
				     IXGBE_MDIO_AUTO_NEG_DEV_TYPE, &reg_cu);
				     break;*/
	default:
		break;
	}

	/*
	 * The possible values of fc.requested_mode are:
	 * 0: Flow control is completely disabled
	 * 1: Rx flow control is enabled (we can receive pause frames,
	 *    but not send pause frames).
	 * 2: Tx flow control is enabled (we can send pause frames but
	 *    we do not support receiving pause frames).
	 * 3: Both Rx and Tx flow control (symmetric) are enabled.
	 * other: Invalid.
	 */
	/* Flow control completely disabled by software override. */
	reg &= ~(IXGBE_PCS1GANA_SYM_PAUSE | IXGBE_PCS1GANA_ASM_PAUSE);
	if (hw->phy.media_type == ixgbe_media_type_backplane)
	    reg_bp &= ~(IXGBE_AUTOC_SYM_PAUSE |
			IXGBE_AUTOC_ASM_PAUSE);
	else if (hw->phy.media_type == ixgbe_media_type_copper)
	    reg_cu &= ~(IXGBE_TAF_SYM_PAUSE | IXGBE_TAF_ASM_PAUSE);
	
	

	/*
	 * AUTOC restart handles negotiation of 1G and 10G on backplane
	 * and copper. There is no need to set the PCS1GCTL register.
	 *
	 */
	if (hw->phy.media_type == ixgbe_media_type_backplane) {
		reg_bp |= IXGBE_AUTOC_AN_RESTART;
		//ret_val = hw->mac.ops.prot_autoc_write(hw, reg_bp, locked);
		ret_val = prot_autoc_write_82599(hw, reg_bp, locked);
		if (ret_val)
			goto out;
	} // else if ((hw->phy.media_type == ixgbe_media_type_copper) &&
	// 	    (ixgbe_device_supports_autoneg_fc(hw))) {
	// 	hw->phy.ops.write_reg(hw, IXGBE_MDIO_AUTO_NEG_ADVT,
	// 			      IXGBE_MDIO_AUTO_NEG_DEV_TYPE, reg_cu);
	// }

	DEBUGOUT1("Set up FC; PCS1GLCTL = 0x%08X\n", reg);
out:
	return ret_val;
}

/**
 *  ixgbe_start_hw_gen2 - Init sequence for common device family
 *  @hw: pointer to hw structure
 *
 * Performs the init sequence common to the second generation
 * of 10 GbE devices.
 * Devices in the second generation:
 *     82599
 *     X540
 **/
s32 ebbrt::IxgbeDriver::ixgbe_start_hw_gen2(struct ixgbe_hw *hw)
{
	u32 i;
	u32 regval;

	/* Clear the rate limiters */
	for (i = 0; i < hw->mac.max_tx_queues; i++) {
		IXGBE_WRITE_REG(hw, IXGBE_RTTDQSEL, i);
		IXGBE_WRITE_REG(hw, IXGBE_RTTBCNRC, 0);
	}
	IXGBE_WRITE_FLUSH(hw);

	/* Disable relaxed ordering */
	for (i = 0; i < hw->mac.max_tx_queues; i++) {
		regval = IXGBE_READ_REG(hw, IXGBE_DCA_TXCTRL_82599(i));
		regval &= ~IXGBE_DCA_TXCTRL_DESC_WRO_EN;
		IXGBE_WRITE_REG(hw, IXGBE_DCA_TXCTRL_82599(i), regval);
	}

	for (i = 0; i < hw->mac.max_rx_queues; i++) {
		regval = IXGBE_READ_REG(hw, IXGBE_DCA_RXCTRL(i));
		regval &= ~(IXGBE_DCA_RXCTRL_DATA_WRO_EN |
			    IXGBE_DCA_RXCTRL_HEAD_WRO_EN);
		IXGBE_WRITE_REG(hw, IXGBE_DCA_RXCTRL(i), regval);
	}

	return IXGBE_SUCCESS;
}

/**
 *  ixgbe_start_hw_generic - Prepare hardware for Tx/Rx
 *  @hw: pointer to hardware structure
 *
 *  Starts the hardware by filling the bus info structure and media type, clears
 *  all on chip counters, initializes receive address registers, multicast
 *  table, VLAN filter table, calls routine to set up link and flow control
 *  settings, and leaves transmit and receive units disabled and uninitialized
 **/
s32 ebbrt::IxgbeDriver::ixgbe_start_hw_generic(struct ixgbe_hw *hw)
{
	s32 ret_val;
	u32 ctrl_ext;

	DEBUGFUNC("ixgbe_start_hw_generic\n");

	/* Set the media type */
	//hw->phy.media_type = hw->mac.ops.get_media_type(hw);
	hw->phy.media_type = ixgbe_get_media_type_82599(hw);

	/* PHY ops initialization must be done in reset_hw() */

	/* Clear the VLAN filter table */
	//hw->mac.ops.clear_vfta(hw);
	ixgbe_clear_vfta_generic(hw);

	/* Clear statistics registers */
	//w->mac.ops.clear_hw_cntrs(hw);
	ixgbe_clear_hw_cntrs_generic(hw);

	/* Set No Snoop Disable */
	ctrl_ext = IXGBE_READ_REG(hw, IXGBE_CTRL_EXT);
	ctrl_ext |= IXGBE_CTRL_EXT_NS_DIS;
	IXGBE_WRITE_REG(hw, IXGBE_CTRL_EXT, ctrl_ext);
	IXGBE_WRITE_FLUSH(hw);

	/* Setup flow control */
	//ret_val = ixgbe_setup_fc(hw);
	ret_val = ixgbe_setup_fc_generic(hw);
	if (ret_val != IXGBE_SUCCESS)
		goto out;

	/* Clear adapter stopped flag */
	hw->adapter_stopped = false;

out:
	return ret_val;
}

/**
 *  ixgbe_start_hw_82599 - Prepare hardware for Tx/Rx
 *  @hw: pointer to hardware structure
 *
 *  Starts the hardware using the generic start_hw function
 *  and the generation start_hw function.
 *  Then performs revision-specific operations, if any.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_start_hw_82599(struct ixgbe_hw *hw)
{
	s32 ret_val = IXGBE_SUCCESS;

	DEBUGFUNC("ixgbe_start_hw_82599\n");

	ret_val = ixgbe_start_hw_generic(hw);
	if (ret_val != IXGBE_SUCCESS)
		goto out;

	ret_val = ixgbe_start_hw_gen2(hw);
	if (ret_val != IXGBE_SUCCESS)
		goto out;

	/* We need to run link autotry after the driver loads */
	hw->mac.autotry_restart = true;

	if (ret_val == IXGBE_SUCCESS)
	    //t_val = ixgbe_verify_fw_version_82599(hw);
out:
	return ret_val;
}

/**
 *  ixgbe_get_link_capabilities_82599 - Determines link capabilities
 *  @hw: pointer to hardware structure
 *  @speed: pointer to link speed
 *  @autoneg: true when autoneg or autotry is enabled
 *
 *  Determines the link capabilities by reading the AUTOC register.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_get_link_capabilities_82599(struct ixgbe_hw *hw,
				      ixgbe_link_speed *speed,
				      bool *autoneg)
{
	s32 status = IXGBE_SUCCESS;
	u32 autoc = 0;

	DEBUGFUNC("ixgbe_get_link_capabilities_82599\n");


	/* Check if 1G SFP module. */
	if (hw->phy.sfp_type == ixgbe_sfp_type_1g_cu_core0 ||
	    hw->phy.sfp_type == ixgbe_sfp_type_1g_cu_core1 ||
	    hw->phy.sfp_type == ixgbe_sfp_type_1g_lx_core0 ||
	    hw->phy.sfp_type == ixgbe_sfp_type_1g_lx_core1 ||
	    hw->phy.sfp_type == ixgbe_sfp_type_1g_sx_core0 ||
	    hw->phy.sfp_type == ixgbe_sfp_type_1g_sx_core1) {
		*speed = IXGBE_LINK_SPEED_1GB_FULL;
		*autoneg = true;
		goto out;
	}

	/*
	 * Determine link capabilities based on the stored value of AUTOC,
	 * which represents EEPROM defaults.  If AUTOC value has not
	 * been stored, use the current register values.
	 */
	if (hw->mac.orig_link_settings_stored)
		autoc = hw->mac.orig_autoc;
	else
		autoc = IXGBE_READ_REG(hw, IXGBE_AUTOC);

	switch (autoc & IXGBE_AUTOC_LMS_MASK) {
	case IXGBE_AUTOC_LMS_1G_LINK_NO_AN:
		*speed = IXGBE_LINK_SPEED_1GB_FULL;
		*autoneg = false;
		break;

	case IXGBE_AUTOC_LMS_10G_LINK_NO_AN:
		*speed = IXGBE_LINK_SPEED_10GB_FULL;
		*autoneg = false;
		break;

	case IXGBE_AUTOC_LMS_1G_AN:
		*speed = IXGBE_LINK_SPEED_1GB_FULL;
		*autoneg = true;
		break;

	case IXGBE_AUTOC_LMS_10G_SERIAL:
		*speed = IXGBE_LINK_SPEED_10GB_FULL;
		*autoneg = false;
		break;

	case IXGBE_AUTOC_LMS_KX4_KX_KR:
	case IXGBE_AUTOC_LMS_KX4_KX_KR_1G_AN:
		*speed = IXGBE_LINK_SPEED_UNKNOWN;
		if (autoc & IXGBE_AUTOC_KR_SUPP)
			*speed |= IXGBE_LINK_SPEED_10GB_FULL;
		if (autoc & IXGBE_AUTOC_KX4_SUPP)
			*speed |= IXGBE_LINK_SPEED_10GB_FULL;
		if (autoc & IXGBE_AUTOC_KX_SUPP)
			*speed |= IXGBE_LINK_SPEED_1GB_FULL;
		*autoneg = true;
		break;

	case IXGBE_AUTOC_LMS_KX4_KX_KR_SGMII:
		*speed = IXGBE_LINK_SPEED_100_FULL;
		if (autoc & IXGBE_AUTOC_KR_SUPP)
			*speed |= IXGBE_LINK_SPEED_10GB_FULL;
		if (autoc & IXGBE_AUTOC_KX4_SUPP)
			*speed |= IXGBE_LINK_SPEED_10GB_FULL;
		if (autoc & IXGBE_AUTOC_KX_SUPP)
			*speed |= IXGBE_LINK_SPEED_1GB_FULL;
		*autoneg = true;
		break;

	case IXGBE_AUTOC_LMS_SGMII_1G_100M:
		*speed = IXGBE_LINK_SPEED_1GB_FULL | IXGBE_LINK_SPEED_100_FULL;
		*autoneg = false;
		break;

	default:
		status = IXGBE_ERR_LINK_SETUP;
		goto out;
		break;
	}

	if (hw->phy.multispeed_fiber) {
		*speed |= IXGBE_LINK_SPEED_10GB_FULL |
			  IXGBE_LINK_SPEED_1GB_FULL;

		/* QSFP must not enable full auto-negotiation
		 * Limited autoneg is enabled at 1G
		 */
		if (hw->phy.media_type == ixgbe_media_type_fiber_qsfp)
			*autoneg = false;
		else
			*autoneg = true;
	}

out:
	return status;
}

/**
 *  ixgbe_set_hard_rate_select_speed - Set module link speed
 *  @hw: pointer to hardware structure
 *  @speed: link speed to set
 *
 *  Set module link speed via RS0/RS1 rate select pins.
 */
void ebbrt::IxgbeDriver::ixgbe_set_hard_rate_select_speed(struct ixgbe_hw *hw,
					ixgbe_link_speed speed)
{
	u32 esdp_reg = IXGBE_READ_REG(hw, IXGBE_ESDP);

	switch (speed) {
	case IXGBE_LINK_SPEED_10GB_FULL:
		esdp_reg |= (IXGBE_ESDP_SDP5_DIR | IXGBE_ESDP_SDP5);
		break;
	case IXGBE_LINK_SPEED_1GB_FULL:
		esdp_reg &= ~IXGBE_ESDP_SDP5;
		esdp_reg |= IXGBE_ESDP_SDP5_DIR;
		break;
	default:
		DEBUGOUT("Invalid fixed module speed\n");
		return;
	}

	IXGBE_WRITE_REG(hw, IXGBE_ESDP, esdp_reg);
	IXGBE_WRITE_FLUSH(hw);
}

/**
 *  ixgbe_disable_tx_laser_multispeed_fiber - Disable Tx laser
 *  @hw: pointer to hardware structure
 *
 *  The base drivers may require better control over SFP+ module
 *  PHY states.  This includes selectively shutting down the Tx
 *  laser on the PHY, effectively halting physical link.
 **/
void ebbrt::IxgbeDriver::ixgbe_disable_tx_laser_multispeed_fiber(struct ixgbe_hw *hw)
{
	u32 esdp_reg = IXGBE_READ_REG(hw, IXGBE_ESDP);

		DEBUGFUNC("ixgbe_disable_tx_laser_multispeed_fiber\n");

	/* Blocked by MNG FW so bail */
	if (ixgbe_check_reset_blocked(hw))
		return;

	/* Disable Tx laser; allow 100us to go dark per spec */
	esdp_reg |= IXGBE_ESDP_SDP3;
	IXGBE_WRITE_REG(hw, IXGBE_ESDP, esdp_reg);
	IXGBE_WRITE_FLUSH(hw);
	usec_delay(100);
}

/**
 *  ixgbe_enable_tx_laser_multispeed_fiber - Enable Tx laser
 *  @hw: pointer to hardware structure
 *
 *  The base drivers may require better control over SFP+ module
 *  PHY states.  This includes selectively turning on the Tx
 *  laser on the PHY, effectively starting physical link.
 **/
void ebbrt::IxgbeDriver::ixgbe_enable_tx_laser_multispeed_fiber(struct ixgbe_hw *hw)
{
	u32 esdp_reg = IXGBE_READ_REG(hw, IXGBE_ESDP);
	
	DEBUGFUNC("ixgbe_enable_tx_laser_multispeed_fiber\n");

		
	/* Enable Tx laser; allow 100ms to light up */
	esdp_reg &= ~IXGBE_ESDP_SDP3;
	IXGBE_WRITE_REG(hw, IXGBE_ESDP, esdp_reg);
	IXGBE_WRITE_FLUSH(hw);
	msec_delay(100);
}

/**
 *  ixgbe_flap_tx_laser_multispeed_fiber - Flap Tx laser
 *  @hw: pointer to hardware structure
 *
 *  When the driver changes the link speeds that it can support,
 *  it sets autotry_restart to true to indicate that we need to
 *  initiate a new autotry session with the link partner.  To do
 *  so, we set the speed then disable and re-enable the Tx laser, to
 *  alert the link partner that it also needs to restart autotry on its
 *  end.  This is consistent with true clause 37 autoneg, which also
 *  involves a loss of signal.
 **/
void ebbrt::IxgbeDriver::ixgbe_flap_tx_laser_multispeed_fiber(struct ixgbe_hw *hw)
{
	DEBUGFUNC("ixgbe_flap_tx_laser_multispeed_fiber\n");

	/* Blocked by MNG FW so bail */
	if (ixgbe_check_reset_blocked(hw))
		return;

	if (hw->mac.autotry_restart) {
		ixgbe_disable_tx_laser_multispeed_fiber(hw);
		ixgbe_enable_tx_laser_multispeed_fiber(hw);
		hw->mac.autotry_restart = false;
	}
}

/**
 *  ixgbe_setup_mac_link_multispeed_fiber - Set MAC link speed
 *  @hw: pointer to hardware structure
 *  @speed: new link speed
 *  @autoneg_wait_to_complete: true when waiting for completion is needed
 *
 *  Set the link speed in the MAC and/or PHY register and restarts link.
 **/
s32 ebbrt::IxgbeDriver::ixgbe_setup_mac_link_multispeed_fiber(struct ixgbe_hw *hw,
					  ixgbe_link_speed speed,
					  bool autoneg_wait_to_complete)
{
	ixgbe_link_speed link_speed = IXGBE_LINK_SPEED_UNKNOWN;
	ixgbe_link_speed highest_link_speed = IXGBE_LINK_SPEED_UNKNOWN;
	s32 status = IXGBE_SUCCESS;
	u32 speedcnt = 0;
	u32 i = 0;
	bool autoneg, link_up = false;

	DEBUGFUNC("ixgbe_setup_mac_link_multispeed_fiber\n");

	/* Mask off requested but non-supported speeds */
	status = ixgbe_get_link_capabilities_82599(hw, &link_speed, &autoneg);
	if (status != IXGBE_SUCCESS)
		return status;

	speed &= link_speed;

	/* Try each speed one by one, highest priority first.  We do this in
	 * software because 10Gb fiber doesn't support speed autonegotiation.
	 */
	if (speed & IXGBE_LINK_SPEED_10GB_FULL) {
		speedcnt++;
		highest_link_speed = IXGBE_LINK_SPEED_10GB_FULL;

		/* If we already have link at this speed, just jump out */
		status = ixgbe_check_mac_link_generic(hw, &link_speed, &link_up, false);
		if (status != IXGBE_SUCCESS)
			return status;

		if ((link_speed == IXGBE_LINK_SPEED_10GB_FULL) && link_up)
			goto out;

		/* Set the module link speed */
		switch (hw->phy.media_type) {
		case ixgbe_media_type_fiber:
		    ixgbe_set_hard_rate_select_speed(hw,
						     IXGBE_LINK_SPEED_10GB_FULL);
			break;
		case ixgbe_media_type_fiber_qsfp:
		    /* QSFP module automatically detects MAC link speed */
			break;
		default:
			DEBUGOUT("Unexpected media type.\n");
			break;
		}

		/* Allow module to change analog characteristics (1G->10G) */
		msec_delay(40);

		//status = ixgbe_setup_mac_link_multispeed_fiber(hw,
		//			      IXGBE_LINK_SPEED_10GB_FULL,
		//			      autoneg_wait_to_complete);
		if (status != IXGBE_SUCCESS)
			return status;

		/* Flap the Tx laser if it has not already been done */
	        ixgbe_flap_tx_laser_multispeed_fiber(hw);

		/* Wait for the controller to acquire link.  Per IEEE 802.3ap,
		 * Section 73.10.2, we may have to wait up to 500ms if KR is
		 * attempted.  82599 uses the same timing for 10g SFI.
		 */
		for (i = 0; i < 5; i++) {
			/* Wait for the link partner to also set speed */
			msec_delay(100);

			/* If we have link, just jump out */
			status = ixgbe_check_mac_link_generic(hw, &link_speed,
						  &link_up, false);
			if (status != IXGBE_SUCCESS)
				return status;

			if (link_up)
				goto out;
		}
	}

out:
	/* Set autoneg_advertised value based on input link speed */
	hw->phy.autoneg_advertised = 0;

	if (speed & IXGBE_LINK_SPEED_10GB_FULL)
		hw->phy.autoneg_advertised |= IXGBE_LINK_SPEED_10GB_FULL;

	if (speed & IXGBE_LINK_SPEED_1GB_FULL)
	{
	    ebbrt::kabort("speed = 1GB\n");
		hw->phy.autoneg_advertised |= IXGBE_LINK_SPEED_1GB_FULL;
	}

	return status;
}

void ebbrt::IxgbeDriver::ixgbe_probe(pci::Device& dev) {
  struct ixgbe_hw* hw = NULL;
  struct net_device* netdev;
  struct ixgbe_adapter* adapter = NULL;
  static int cards_found;
  int err, pci_using_dac, expected_gts;
  u16 offset = 0;
  u16 eeprom_verh = 0, eeprom_verl = 0;
  u16 eeprom_cfg_blkh = 0, eeprom_cfg_blkl = 0;
  u32 etrack_id;
  u16 build, major, patch;
  char *info_string, *i_s_var;
  u8 part_str[IXGBE_PBANUM_LENGTH];
  enum ixgbe_mac_type mac_type = ixgbe_mac_unknown;
  bool disable_dev = false;
  u32 hw_features;

  bar0_.Map();  // allocate virtual memory

  hw = (struct ixgbe_hw*)malloc(sizeof(struct ixgbe_hw));
  if (hw == NULL) {
    ebbrt::kabort("Unable to malloc struct ixgbe_hw\n");
  } else {
    hw->vendor_id = kIxgbeVendorId;
    hw->device_id = kIxgbeDeviceId;
    ixgbe_set_mac_type(hw);
    mac_type = hw->mac.type;
  }

  /* setup the private structure */
  err = ixgbe_sw_init(hw, dev);
  if (err)
    goto err_sw_init;

  /* Make it possible the adapter to be woken up via WOL */
  IXGBE_WRITE_REG(hw, IXGBE_WUS, ~0);

  /* reset_hw fills in the perm_addr as well */
  hw->phy.reset_if_overtemp = true;
  //err = hw->mac.ops.reset_hw(hw);
redo:
  err = ixgbe_reset_hw_82599(hw);
  hw->phy.reset_if_overtemp = false;
  if (err == IXGBE_ERR_SFP_NOT_PRESENT) {
      err = IXGBE_SUCCESS;
  } else if (err == IXGBE_ERR_SFP_NOT_SUPPORTED) {
      ebbrt::kprintf("failed to load because an unsupported SFP+ or QSFP "
		     "module type was detected.\n");
      ebbrt::kprintf("Reload the driver after installing a supported "
		"module.\n");
      goto err_sw_init;
  } else if (err) {
      ebbrt::kprintf("HW Init failed: %d\n", err);
      goto redo;
  }
  ebbrt::kprintf("ixgbe_reset_hw_82599 complete\n");

  ixgbe_start_hw_82599(hw);
  if (err == IXGBE_ERR_EEPROM_VERSION) {
		/* We are running on a pre-production device, log a warning */
	        ebbrt::kprintf("This device is a pre-production adapter/LOM. "
			   "Please be aware there may be issues associated "
			   "with your hardware.  If you are experiencing "
			   "problems please contact your Intel or hardware "
			   "representative who provided you with this "
			   "hardware.\n");
  } else if (err) {
      ebbrt::kprintf("HW init failed\n");
      //goto err_register;
  }
  ebbrt::kprintf("ixgbe_start_hw_82599 complete\n");
  ixgbe_disable_tx_laser_multispeed_fiber(hw);
  ixgbe_setup_mac_link_multispeed_fiber(hw, IXGBE_LINK_SPEED_10GB_FULL, true);

  
  
err_sw_init:
  return;
}

void ebbrt::IxgbeDriver::SendPacket(uint32_t n) {
  auto tlen = 0;
  auto txbuf = (uint8_t*)malloc(sizeof(uint8_t) * 60);

  // dest 90:e2:ba:84:d7:38
  txbuf[tlen++] = 0x90;
  txbuf[tlen++] = 0xE2;
  txbuf[tlen++] = 0xBA;
  txbuf[tlen++] = 0x84;
  txbuf[tlen++] = 0xD7;
  txbuf[tlen++] = 0x38;

  // src
  txbuf[tlen++] = 0x90;
  txbuf[tlen++] = 0xE2;
  txbuf[tlen++] = 0xBA;
  txbuf[tlen++] = 0x82;
  txbuf[tlen++] = 0x33;
  txbuf[tlen++] = 0x24;

  // eth type
  txbuf[tlen++] = 0x08;
  txbuf[tlen++] = 0x00;

  // payload
  txbuf[tlen++] = 0xDE;
  txbuf[tlen++] = 0xAD;
  txbuf[tlen++] = 0xBE;
  txbuf[tlen++] = 0xFF;

  txbuf[tlen++] = 0xDE;
  txbuf[tlen++] = 0xAD;
  txbuf[tlen++] = 0xBE;
  txbuf[tlen++] = 0xFF;

  for (auto i = tlen; i < 60; i++) {
    txbuf[i] = 0x22;
  }

  for (auto i = 0; i < 60; i++) {
    ebbrt::kprintf("%02X ", txbuf[i]);
  }
  ebbrt::kprintf("\n\n");

  auto txphys = reinterpret_cast<uint64_t>(txbuf);
  auto tail = ixgq->tx_tail;
  // update buffer address for descriptor
  ixgq->tx_ring[tail].buffer_address = txphys;
  ixgq->tx_ring[tail].length = 60;
  ixgq->tx_ring[tail].eop = 1;
  ixgq->tx_ring[tail].rs = 1;

  ebbrt::kprintf("taddr - %p %p len:%d\n", txbuf,
                 ixgq->tx_ring[tail].buffer_address,
                 ixgq->tx_ring[tail].length);

  // bump tx_tail
  ixgq->tx_tail = (tail + 1) % ixgq->tx_size;
  WriteTdt(n, ixgq->tx_tail);
  ebbrt::kprintf("bump tail: tx_head = %d tx_tail = %d", ixgq->tx_head,
                 ixgq->tx_tail);

  auto head = ixgq->tx_head;
  std::atomic_thread_fence(std::memory_order_release);
  while (ixgq->tx_ring[head].dd == 0) {
  }

  ebbrt::kprintf("TX dma complete\n");
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

    auto txphys = bAddr;
    auto txtail = ixgq->tx_tail;
    // update buffer address for descriptor
    ixgq->tx_ring[txtail].buffer_address = txphys;
    ixgq->tx_ring[txtail].length = len;
    ixgq->tx_ring[txtail].eop = 1;
    ixgq->tx_ring[txtail].rs = 1;
    
    // bump tx_tail
    ixgq->tx_tail = (txtail + 1) % ixgq->tx_size;
    WriteTdt(n, ixgq->tx_tail);

    auto txhead = ixgq->tx_head;
    std::atomic_thread_fence(std::memory_order_release);
    //while (ixgq->tx_ring[txhead].dd == 0) {
    //}
    ebbrt::kprintf("TX dma complete\n");

    count++;
  }

  if (count > 0) {
    ebbrt::kprintf("NEW head=%d tail=%d\n", ixgq->rx_head, ixgq->rx_tail);
    // update reg
    WriteRdt_1(n, ixgq->rx_tail);
  }
}

void ebbrt::IxgbeDriver::InitStruct() {
  /*struct IxgbeRegs* r = static_cast<struct IxgbeRegs*>(bar0_.GetVaddr());

ebbrt::kprintf(
    "0x00000: CTRL        (Device Control)                 0x%08X\n",
    r->kIxgbeCtrl);

ebbrt::kprintf(
    "0x00008: STATUS      (Device Status)                  0x%08X\n",
    r->kIxgbeStatus);*/
}

void ebbrt::IxgbeDriver::DeviceInfo() {
  /*uint32_t reg;

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

ebbrt::kprintf(
    "0x00000: CTRL        (Device Control)                 0x%08X\n",
    bar0_.Read32(0x0));

ebbrt::kprintf(
    "0x00008: STATUS      (Device Status)                  0x%08X\n",
    bar0_.Read32(0x8));*/
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
void ebbrt::IxgbeDriver::WriteEicr(uint32_t m) { bar0_.Write32(0x00800, m); }

// 8.2.3.5.3 Extended Interrupt Mask Set/Read Register- EIMS (0x00880; RWS)
uint32_t ebbrt::IxgbeDriver::ReadEims() { return bar0_.Read32(0x00880); }
void ebbrt::IxgbeDriver::WriteEims(uint32_t m) { bar0_.Write32(0x00880, m); }

// 8.2.3.5.4 Extended Interrupt Mask Clear Register- EIMC (0x00888; WO)
void ebbrt::IxgbeDriver::WriteEimc(uint32_t m) { bar0_.Write32(0x00888, m); }

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
  bar0_.Write32(0x05090, reg | m);
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
  ebbrt::kprintf("mac %p valid = %x\n", d_mac, ReadRahAv(0));

  // Wait for link to come up
  while (!ReadLinksLinkUp())
    ;  // TODO: timeout
  ebbrt::kprintf("Link is up\n");
  ebbrt::clock::SleepMilli(50);

  // Initialize interrupts
  WriteEicr(0xFFFFFFFF);

  // TODO: use interrupt
  // using interrupts I believe
  // WriteGpie(0x1 << 6);  // EIMEN bit 6
  // WriteEimc(ReadEims());
  // WriteEims(0x7FFFFFFF);

  // ebbrt::kprintf("EIMS: %p\n", ReadEims());

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
  WriteMcstctrl(0x1 << 2);  // setting Multicast filter enable

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
  // auto ii = i / 2;
  // if((i % 2) == 0) {
  // WriteIvarAlloc0(i, 0);
  // WriteIvarAllocval0(i, 0x1 << 7);
  // WriteIvarAlloc1(i, 0 << 8);
  // WriteIvarAllocval1(i, 0x1 << 15);
  //}

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
  //WriteTdh(i, 0x0);
  //WriteTdt(i, 0x0);

  // enable transmit path
  WriteDmatxctl_te(0x1);

  // transmit queue enable
  WriteTxdctl(i, 0x1 << 25);
  // poll until set, TODO: Timeout
  while (ReadTxdctl_enable(i) == 0)
    ;
  ebbrt::kprintf("TX queue enabled\n");

  // TODO: set up dca txctrl FreeBSD?
  WriteDcaTxctrlTxdescWbro(i , ~(0x1 << 11)); //clear TXdescWBROen  
  
}
