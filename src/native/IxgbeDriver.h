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
        dev.GetDeviceId() == kIxgbeDeviceId &&
	dev.GetFunc() == 0) {
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

 private:
  void InitStruct();
  void DeviceInfo();
  void Init();
  void StopDevice();
  void GlobalReset();
  
  void WriteRxctrl(uint32_t m);
  void WriteDmatxctl(uint32_t m);
  void WriteEimc(uint32_t m);
  void WriteTxdctl(uint32_t n, uint32_t m);
  void WriteRxdctl_1(uint32_t n, uint32_t m);
  void WriteRxdctl_2(uint32_t n, uint32_t m);
  void WriteCtrl(uint32_t m);
  
  void ReadEicr();
  bool ReadStatus();
  void ReadCtrl();
  
  pci::Device& dev_;
  pci::Bar& bar0_;

  struct IxgbeRegs {
      volatile uint32_t kIxgbeCtrl;
      volatile uint32_t kIxgbeCtrlBak;
      volatile uint32_t kIxgbeStatus;
  };
};
}  // namespace ebbrt

#endif  // BAREMETAL_SRC_INCLUDE_EBBRT_IXGBE_DRIVER_H_
