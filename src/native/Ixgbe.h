#ifndef BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_H_
#define BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_H_

//from https://github.com/cisco-open-source/ethtool/ixgbe.c

/* Register Bit Masks */
#define IXGBE_FCTRL_SBP            0x00000002
#define IXGBE_FCTRL_MPE            0x00000100
#define IXGBE_FCTRL_UPE            0x00000200
#define IXGBE_FCTRL_BAM            0x00000400
#define IXGBE_FCTRL_PMCF           0x00001000
#define IXGBE_FCTRL_DPF            0x00002000
#define IXGBE_FCTRL_RPFCE          0x00004000
#define IXGBE_FCTRL_RFCE           0x00008000
#define IXGBE_VLNCTRL_VET          0x0000FFFF
#define IXGBE_VLNCTRL_CFI          0x10000000
#define IXGBE_VLNCTRL_CFIEN        0x20000000
#define IXGBE_VLNCTRL_VFE          0x40000000
#define IXGBE_VLNCTRL_VME          0x80000000
#define IXGBE_LINKS_UP             0x40000000
#define IXGBE_LINKS_SPEED          0x20000000
#define IXGBE_SRRCTL_BSIZEPKT_MASK 0x0000007F
#define IXGBE_HLREG0_TXCRCEN       0x00000001
#define IXGBE_HLREG0_RXCRCSTRP     0x00000002
#define IXGBE_HLREG0_JUMBOEN       0x00000004
#define IXGBE_HLREG0_TXPADEN       0x00000400
#define IXGBE_HLREG0_LPBK          0x00008000
#define IXGBE_RMCS_TFCE_802_3X     0x00000008
#define IXGBE_RMCS_TFCE_PRIORITY   0x00000010
#define IXGBE_FCCFG_TFCE_802_3X    0x00000008
#define IXGBE_FCCFG_TFCE_PRIORITY  0x00000010
#define IXGBE_MFLCN_PMCF           0x00000001 /* Pass MAC Control Frames */
#define IXGBE_MFLCN_DPF            0x00000002 /* Discard Pause Frame */
#define IXGBE_MFLCN_RPFCE          0x00000004 /* Receive Priority FC Enable */
#define IXGBE_MFLCN_RFCE           0x00000008 /* Receive FC Enable */

#endif  // BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_H_
