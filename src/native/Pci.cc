//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#include "Pci.h"

#include <cinttypes>

#include "../Align.h"
#include "../ExplicitlyConstructed.h"
#include "Debug.h"
#include "Io.h"
#include "VMem.h"
#include "VMemAllocator.h"

#define BIT(n) (1ul<<(n))
#define MASK(n) (BIT(n)-1ul)
#define CTZ(x) __builtin_ctz(x)
#define PCI_CONF_PORT_ADDR     0x0CF8
#define PCI_CONF_PORT_DATA     0x0CFC
#define PCI_CONF_PORT_ADDR_END (PCI_CONF_PORT_ADDR + 4)
#define PCI_CONF_PORT_DATA_END (PCI_CONF_PORT_DATA + 4)
#define PCI_MAX_DEVICES 128
#define PCI_INTERRUPT_LINE      0x3c    /* 8 bits */
#define PCI_INTERRUPT_PIN       0x3d    /* 8 bits */
#define PCI_SUBSYSTEM_ID        0x2e
#define PCI_PRIMARY_BUS         0x18

#define PCI_BASE_ADDRESS_0      0x10    /* 32 bits */
#define PCI_BASE_ADDRESS_1      0x14    /* 32 bits [htype 0,1 only] */
#define PCI_BASE_ADDRESS_2      0x18    /* 32 bits [htype 0 only] */
#define PCI_BASE_ADDRESS_3      0x1c    /* 32 bits */
#define PCI_BASE_ADDRESS_4      0x20    /* 32 bits */
#define PCI_BASE_ADDRESS_5      0x24    /* 32 bits */
#define  PCI_BASE_ADDRESS_SPACE         0x01    /* 0 = memory, 1 = I/O */
#define  PCI_BASE_ADDRESS_SPACE_IO      0x01
#define  PCI_BASE_ADDRESS_SPACE_MEMORY  0x00
#define  PCI_BASE_ADDRESS_MEM_TYPE_MASK 0x06
#define  PCI_BASE_ADDRESS_MEM_TYPE_32   0x00    /* 32 bit address */
#define  PCI_BASE_ADDRESS_MEM_TYPE_1M   0x02    /* Below 1M [obsolete] */
#define  PCI_BASE_ADDRESS_MEM_TYPE_64   0x04    /* 64 bit address */
#define  PCI_BASE_ADDRESS_MEM_PREFETCH  0x08    /* prefetchable? */
#define  PCI_BASE_ADDRESS_MEM_MASK      (~0x0fUL)
#define  PCI_BASE_ADDRESS_IO_MASK       (~0x03UL)

struct libpci_device_iocfg {
    /* PCI_BASE_ADDRESS_MEM address or
       PCI_BASE_ADDRESS_IO address */
    uint32_t base_addr[6];
    /* PCI_BASE_ADDRESS_SPACE_IO or
       PCI_BASE_ADDRESS_SPACE_MEMORY */
    uint8_t base_addr_space[6];
    /* PCI_BASE_ADDRESS_MEM_TYPE_32 or
       PCI_BASE_ADDRESS_MEM_TYPE_64 */
    uint8_t base_addr_type[6];
    /* PCI_BASE_ADDRESS_MEM_PREFETCH */
    uint8_t base_addr_prefetchable[6];
    /* size */
    uint32_t base_addr_size_mask[6];
    uint32_t base_addr_size[6];
    /* raw addr */
    uint32_t base_addr_raw[6];
    uint32_t base_addr_raw2[6];
    /* Is this BAR the higher word of a 64-bit address? If true, then this BAR is partial
       and should not be directly processed in any way. */
    bool base_addr_64H[6];
};

typedef struct libpci_device_iocfg libpci_device_iocfg_t;

typedef struct libpci_device {
    uint8_t bus;
    uint8_t dev;
    uint8_t fun;

    uint16_t vendor_id;
    uint16_t device_id;
    uint16_t subsystem_id;
    uint8_t interrupt_pin;
    uint8_t interrupt_line;

    libpci_device_iocfg_t cfg;
} libpci_device_t;

libpci_device_t libpci_device_list[PCI_MAX_DEVICES];
uint32_t libpci_num_devices = 0;

namespace {
const constexpr uint32_t kPciAddressPort = 0xCF8;
const constexpr uint32_t kPciDataPort = 0xCFC;
const constexpr uint32_t kPciAddressEnable = 0x80000000;
const constexpr uint8_t kPciBusOffset = 16;
const constexpr uint8_t kPciDeviceOffset = 11;
const constexpr uint8_t kPciFuncOffset = 8;
const constexpr uint8_t PCI_HEADER_TYPE = 0x0E;
const constexpr uint16_t PCI_VENDOR_ID_INVALID = 0xFFFF;
const constexpr uint8_t PCI_VENDOR_ID = 0x00;
const constexpr uint8_t PCI_CLASS_DEVICE = 0x0a;
const constexpr uint8_t PCI_SECONDARY_BUS = 0x19;
const constexpr uint8_t PCI_DEVICE_ID = 0x02;

void PciSetAddr(uint8_t bus, uint8_t device, uint8_t func, uint8_t offset) {
ebbrt::io::Out32(kPciAddressPort, kPciAddressEnable | (bus << kPciBusOffset) |
                                        (device << kPciDeviceOffset) |
                                        (func << kPciFuncOffset) | offset);
}

uint8_t PciRead8(uint8_t bus, uint8_t dev, uint8_t fun, uint8_t reg) {
    PciSetAddr(bus, dev, fun, reg);
    return ebbrt::io::In8(kPciDataPort + (reg & 3));
    /*ebbrt::io::Out32(PCI_CONF_PORT_ADDR, 0x80000000 | bus << 16 | dev << 11 | fun << 8 | (reg & ~MASK(2)));
      return (ebbrt::io::In32(PCI_CONF_PORT_DATA) >> ((reg & MASK(2)) * 8) ) & 0xFF;*/
}
uint16_t PciRead16(uint8_t bus, uint8_t dev, uint8_t fun, uint8_t reg) {
    PciSetAddr(bus, dev, fun, reg);
    return ebbrt::io::In16(kPciDataPort + (reg & 2));
    /*reg &= ~MASK(1);
    ebbrt::io::Out32(PCI_CONF_PORT_ADDR, 0x80000000 | bus << 16 | dev << 11 | fun << 8 | (reg & ~MASK(2)));
    return ( ebbrt::io::In32(PCI_CONF_PORT_DATA) >> ((reg & MASK(2)) * 8) ) & 0xFFFF;*/
}

uint32_t PciRead32(uint8_t bus, uint8_t dev, uint8_t fun, uint8_t reg) {
    /*reg &= ~MASK(2);
    ebbrt::io::Out32(PCI_CONF_PORT_ADDR, 0x80000000 | bus << 16 | dev << 11 | fun << 8 | reg);
    return ebbrt::io::In32(PCI_CONF_PORT_DATA);*/
    PciSetAddr(bus, dev, fun, reg);
    return ebbrt::io::In32(kPciDataPort);
}

void PciWrite16(uint8_t bus, uint8_t dev, uint8_t fun, uint8_t reg,
                uint16_t val) {
  PciSetAddr(bus, dev, fun, reg);
  ebbrt::io::Out16(kPciDataPort + (reg & 2), val);
}

void PciWrite32(uint8_t bus, uint8_t dev, uint8_t fun, uint8_t reg,
                uint32_t val) {
    /*reg &= ~MASK(2);
    ebbrt::io::Out32(PCI_CONF_PORT_ADDR, 0x80000000 | bus << 16 | dev << 11 | fun << 8 | reg);
    ebbrt::io::Out32(PCI_CONF_PORT_DATA, val);*/
    PciSetAddr(bus, dev, fun, reg);
    ebbrt::io::Out32(kPciDataPort, val);
}

ebbrt::ExplicitlyConstructed<std::vector<ebbrt::pci::Device>> devices;
ebbrt::ExplicitlyConstructed<
    std::vector<std::function<bool(ebbrt::pci::Device&)>>>
    driver_probes;

void EnumerateBus(uint8_t bus) {
  for (auto device = 0; device < 32; ++device) {
    auto dev = ebbrt::pci::Function(bus, device, 0);
    if (dev)
      continue;

    auto nfuncs = dev.IsMultifunc() ? 8 : 1;
    for (auto func = 0; func < nfuncs; ++func) {
      dev = ebbrt::pci::Function(bus, device, func);
      if (dev)
        continue;
      
      dev.DumpAddress();
      dev.DumpInfo();
      
      if (dev.IsBridge()) 
      {
	  continue;
	  //uint8_t new_bus = PciRead8(bus, device, func, PCI_SECONDARY_BUS);
	  //ebbrt::kprintf("%s found ADDITIONAL bus %d from %d %d %d\n", __FUNCTION__, new_bus, bus, device, func);
	  //EnumerateBus(new_bus);
	  
//ebbrt::kabort("Secondary bus unsupported!\n");
      } else {
	  devices->emplace_back(bus, device, func);
      }
    }
  }
}

void EnumerateAllBuses() {
  auto bus = ebbrt::pci::Function(0, 0, 0);
  if (!bus.IsMultifunc()) {
    // single bus
    EnumerateBus(0);
  } else {
    // potentially multiple buses
    for (auto func = 0; func < 8; ++func) {
      bus = ebbrt::pci::Function(0, 0, func);
      if (!bus)
        continue;
      EnumerateBus(func);
    }
  }
}

inline uint64_t libpci_device_iocfg_get_baseaddr(libpci_device_iocfg_t *cfg, int index) {
    assert(cfg && index >= 0 && index < 6);
    if (cfg->base_addr_type[index] != PCI_BASE_ADDRESS_MEM_TYPE_64)
           return (uint64_t) cfg->base_addr[index];
    /* 64-bit mode BARs must have a word after it. */
    assert(index < 5);
    /* And the word before it better be set to 64L mode. */
    assert(cfg->base_addr_64H[index + 1]);
    ebbrt::kprintf("\tbase_addr_L[%d]: 0x%x \tbase_addr_H[%d]: 0x%x \tbase_addr_raw[%d]: 0x%x\n", index, cfg->base_addr[index], index+1, cfg->base_addr[index + 1], index+1, cfg->base_addr_raw[index + 1]);
    return ((uint64_t) cfg->base_addr[index]) | (((uint64_t) cfg->base_addr[index + 1]) << 32);
}

void libpci_device_iocfg_debug_print(libpci_device_iocfg_t *cfg, bool compact) {
    for(int i = 0; i < 6; i++) 
    {
        if (compact) 
	{
            /* Display in compact space mode, shoving as much information as possible in a few
             * lines. This is similar to how the Linux kernel PCI debug displays in dmesg. */
            if (cfg->base_addr[i] == 0 || cfg->base_addr_64H[i]) 
	    {
		continue;
	    }
	    
            if (cfg->base_addr_space[i] == PCI_BASE_ADDRESS_SPACE_IO) 
	    {
                ebbrt::kprintf("\tBAR%d : [ io 0x%" PRIx64 " sz 0x%x szmask 0x%x ]\n", i,
                       libpci_device_iocfg_get_baseaddr(cfg, i),
                       cfg->base_addr_size[i],
                       cfg->base_addr_size_mask[i]);
            } 
	    else 
	    {
                ebbrt::kprintf("\tBAR%d : [ mem 0x%" PRIx64 " sz 0x%x szmask 0x%x %s %s ]\n", i,
                       libpci_device_iocfg_get_baseaddr(cfg, i),
                       cfg->base_addr_size[i],
                       cfg->base_addr_size_mask[i],
                       cfg->base_addr_type[i] == PCI_BASE_ADDRESS_MEM_TYPE_64 ? "64bit" : "",
                       cfg->base_addr_prefetchable[i] ? "prefetch" : "");
            }
        } 
	else 
	{
            /* Very verbose and space wasting debug output. */
            ebbrt::kprintf("\tBASE_ADDR[%d] ----\n", i);
            if (cfg->base_addr[i] == 0 || cfg->base_addr_64H[i]) 
	    {
		continue;
	    }
            ebbrt::kprintf("\tbase_addr_space[%d]: 0x%x [%s]\n", i, cfg->base_addr_space[i], cfg->base_addr_space[i] ? "PCI_BASE_ADDRESS_SPACE_IO" : "PCI_BASE_ADDRESS_SPACE_MEMORY");
	    
            ebbrt::kprintf("\tbase_addr_type[%d]: 0x%x [ ", i, cfg->base_addr_type[i]);
	    
            if (cfg->base_addr_type[i] == PCI_BASE_ADDRESS_MEM_TYPE_32) ebbrt::kprintf("32bit ");
            if (cfg->base_addr_type[i] == PCI_BASE_ADDRESS_MEM_TYPE_64) ebbrt::kprintf("64bit ");
            if (cfg->base_addr_type[i] == PCI_BASE_ADDRESS_MEM_TYPE_1M) ebbrt::kprintf("<1M ");
            
	    ebbrt::kprintf("]\n");
            ebbrt::kprintf("\tbase_addr_prefetchable[%d]: %s\n", i, cfg->base_addr_prefetchable[i] ? "yes" : "no");
            ebbrt::kprintf("\tsize[%d]: 0x%x\n", i, cfg->base_addr_size[i]);
	    ebbrt::kprintf("\tbase_addr[%d]: 0x%" PRIx64 "\n", i, libpci_device_iocfg_get_baseaddr(cfg, i));
            ebbrt::kprintf("\tbase_addr_raw[%d]: 0x%x\n", i, cfg->base_addr_raw[i]);
	    ebbrt::kprintf("\tbase_addr_raw2[%d]: 0x%x\n", i, cfg->base_addr_raw2[i]);
        }
    }
}

void libpci_read_ioconfig(libpci_device_iocfg_t *cfg, uint8_t bus, uint8_t dev, uint8_t fun) {
    assert(cfg);
    memset(cfg, 0, sizeof(libpci_device_iocfg_t));
    
    // Header Type is 0x00 or 0x80 (multifunc) so 6 BAR addresses
    for (int i = 0; i < 6; i++) 
    {
        // Read and save the base address assigned by the BIOS.
        uint32_t bios_base_addr = PciRead32(bus, dev, fun, PCI_BASE_ADDRESS_0 + (i * 4));
        cfg->base_addr_raw[i] = bios_base_addr;

	// Don't bother processing further if this is already part of a 64-bit address.
        if (cfg->base_addr_64H[i]) {
	    cfg->base_addr[i] = cfg->base_addr_raw[i];
            cfg->base_addr_size_mask[i] = 0xFFFFFFFF;
            cfg->base_addr_size[i] = 0;
	    // Write 0xFFFFFFFF to start reading the config information
	    PciWrite32(bus, dev, fun, PCI_BASE_ADDRESS_0 + (i * 4), 0xFFFFFFFF);
	    // Do a subsequent read to get BAR information
	    uint32_t cfg_base_addr = PciRead32(bus, dev, fun, PCI_BASE_ADDRESS_0 + (i * 4));
	    cfg->base_addr[i] = cfg_base_addr;
            continue;
        }

        // Write 0xFFFFFFFF to start reading the config information
        PciWrite32(bus, dev, fun, PCI_BASE_ADDRESS_0 + (i * 4), 0xFFFFFFFF);
	// Do a subsequent read to get BAR information
        uint32_t cfg_base_addr = PciRead32(bus, dev, fun, PCI_BASE_ADDRESS_0 + (i * 4));

	// if 0, BAR isn't implemented
        if (cfg_base_addr == 0)
            continue;

	// Memory BAR or I/O BAR by looking at bit 0
        cfg->base_addr_space[i] = cfg_base_addr & PCI_BASE_ADDRESS_SPACE;
	cfg->base_addr_raw2[i] = cfg_base_addr;
	
	// if BAR is for memory
        if (cfg->base_addr_space[i] == PCI_BASE_ADDRESS_SPACE_MEMORY) 
	{
	    // bit [2:1] determines type, where block of memory can be located
            cfg->base_addr_type[i] = (cfg_base_addr & PCI_BASE_ADDRESS_MEM_TYPE_MASK);
	    
	    // bit [3] determines prefetchable
            cfg->base_addr_prefetchable[i] = (cfg_base_addr & PCI_BASE_ADDRESS_MEM_PREFETCH) > 0;
	    
	    // bits [31:4] or [63:4] (if 64 bit add) for base address field
            cfg->base_addr_size_mask[i] = cfg_base_addr & PCI_BASE_ADDRESS_MEM_MASK;
	    
	    // if address type is binary 10, means memory block can be located anywhere in 64 bit address space
            if (cfg->base_addr_type[i] == PCI_BASE_ADDRESS_MEM_TYPE_64) {
                /* Handle 64-bit addresses. */
                assert(i < 5);
                // Set up the next BAR entry to be 64H mode.
                cfg->base_addr_64H[i + 1] = true;
                // Set up this BAR entry to be in 64L mode.
                cfg->base_addr[i] = bios_base_addr & PCI_BASE_ADDRESS_MEM_MASK;
            } 
	    else 
	    {
                cfg->base_addr[i] = bios_base_addr & PCI_BASE_ADDRESS_MEM_MASK;
            }
        } 
	// if BAR is for I/O
	else   
	{
	    //bits [31:2] for base address field
            cfg->base_addr[i] = bios_base_addr & PCI_BASE_ADDRESS_IO_MASK;
	    
            cfg->base_addr_size_mask[i] = cfg_base_addr & PCI_BASE_ADDRESS_IO_MASK;
	    
            cfg->base_addr_type[i] = PCI_BASE_ADDRESS_MEM_TYPE_32;
        }

        /* Calculate size from size_mask. */
        cfg->base_addr_size[i] = 1 << CTZ(cfg->base_addr_size_mask[i]);
	//ebbrt::kprintf("msize = 0x%x\n", (~cfg->base_addr_size_mask[i]) + 1);
	
        // Write back the address set by the BIOS.
        PciWrite32(bus, dev, fun, PCI_BASE_ADDRESS_0 + (i * 4), bios_base_addr);
    }
}

void PciAddFun(uint8_t bus, uint8_t dev, uint8_t fun) {
    uint16_t vendor_id = PciRead16(bus, dev, fun, PCI_VENDOR_ID);
    if (vendor_id == PCI_VENDOR_ID_INVALID) {
        /* No device here. */
        return;
    }
    
    uint16_t device_id = PciRead16(bus, dev, fun, PCI_DEVICE_ID);


    assert(libpci_num_devices + 1<= PCI_MAX_DEVICES);
    libpci_device_list[libpci_num_devices].bus = bus;
    libpci_device_list[libpci_num_devices].dev = dev;
    libpci_device_list[libpci_num_devices].fun = fun;
    libpci_device_list[libpci_num_devices].vendor_id = vendor_id;
    libpci_device_list[libpci_num_devices].device_id = device_id;
    libpci_device_list[libpci_num_devices].interrupt_line = PciRead8(bus, dev, fun, PCI_INTERRUPT_LINE);
    libpci_device_list[libpci_num_devices].interrupt_pin = PciRead8(bus, dev, fun, PCI_INTERRUPT_PIN);
    libpci_device_list[libpci_num_devices].subsystem_id = PciRead8(bus, dev, fun, PCI_SUBSYSTEM_ID);
    libpci_read_ioconfig(&libpci_device_list[libpci_num_devices].cfg, bus, dev, fun);
    
    if(libpci_device_list[libpci_num_devices].device_id == 0x10fb)
    {
	ebbrt::kprintf("PCI :: Device found at BUS %d DEV %d FUN %d:\n", (int)bus, (int)dev, (int)fun);
	ebbrt::kprintf("    vendorID = [0x%x]\n", vendor_id);
	
	ebbrt::kprintf("    deviceID = [0x%x]\n", device_id);
	
	ebbrt::kprintf("    class code = [0x%x]\n", PciRead8(bus, dev, fun, 0x0B));
	ebbrt::kprintf("    header type = [0x%x]\n", PciRead8(bus, dev, fun, 0x0E));
	ebbrt::kprintf("    interrupt line = [0x%x]\n", libpci_device_list[libpci_num_devices].interrupt_line);
	ebbrt::kprintf("    interrupt pin = [0x%x]\n", libpci_device_list[libpci_num_devices].interrupt_pin);
	ebbrt::kprintf("    subsystem id = [0x%x]\n", libpci_device_list[libpci_num_devices].subsystem_id);
	
	libpci_device_iocfg_debug_print(&libpci_device_list[libpci_num_devices].cfg, false);
    }

    libpci_num_devices++;
}

void PciScanBus(int bus);

void PciScanFun(int bus, int dev, int fun) {
    PciAddFun(bus, dev, fun);
    if ( PciRead16(bus, dev, fun, PCI_CLASS_DEVICE) == 0x0604) {
        int new_bus = PciRead8(bus, dev, fun, PCI_SECONDARY_BUS);
	ebbrt::kprintf("%s found ADDITIONAL bus %d from %d %d %d\n", __FUNCTION__, new_bus, bus, dev, fun);
        PciScanBus(new_bus);
    }
}

void PciScanDev(int bus, int dev)
{
    uint16_t vendor_id = PciRead16(bus, dev, 0, PCI_VENDOR_ID);
    if (vendor_id == PCI_VENDOR_ID_INVALID) {
        return;
    }
    
    PciScanFun(bus, dev, 0);
    if ( (PciRead8(bus, dev, 0, PCI_HEADER_TYPE) & 0x80) != 0) {
	ebbrt::kprintf("%s found multi function device %d %d\n", __FUNCTION__, bus, dev);
	 for (int function = 1; function < 8; function++) {
            if (PciRead16(bus, dev, function, PCI_VENDOR_ID) != PCI_VENDOR_ID_INVALID) {
                PciScanFun(bus, dev, function);
            }
        }
    }
}

void PciScanBus(int bus) {
    ebbrt::kprintf("%s %d\n", __FUNCTION__, bus);
    for (int dev = 0; dev < 32; dev++) {
        PciScanDev(bus, dev);
    } 
}

void PciScan() {
    ebbrt::kprintf("PCI Scanning ... \n");
    if((PciRead8(0, 0, 0, PCI_HEADER_TYPE) & 0x80) == 0)
    {
	ebbrt::kprintf("Single bus detected\n");
	PciScanBus(0);
    }
    else
    {
	for(auto func = 0; func < 8; func ++)
	{
	    if(PciRead16(0, 0, func, PCI_VENDOR_ID) != PCI_VENDOR_ID_INVALID)
	    {
		ebbrt::kprintf("%s detected bus %d\n", __FUNCTION__, func);
		PciScanBus(func);
	    }
	}
    }

    //PciScanBus(0x7f);
    PciScanBus(0x80);
    //PciScanBus(0x82);
    //PciScanBus(0xff);
}

}  // namespace

uint32_t read_pci_config(uint8_t bus, uint8_t slot, uint8_t func, uint8_t offset)
{
    uint32_t v;
    
    ebbrt::io::Out32(0xcf8, 0x80000000 | (bus<<16) | (slot<<11) | (func<<8) | offset);
    v = ebbrt::io::In32(0xcfc);

    return v;

}

uint8_t read_pci_config_byte(uint8_t bus, uint8_t slot, uint8_t func, uint8_t offset)
{
    uint8_t v;
    ebbrt::io::Out32(0xcf8, 0x80000000 | (bus<<16) | (slot<<11) | (func<<8) | offset);
    v = ebbrt::io::In8(0xcfc + (offset&3));
    return v;
}

void early_dump_pci_device(uint8_t bus, uint8_t slot, uint8_t func)
{
    int i, j;
    uint32_t val;
    
    ebbrt::kprintf("pci 0000:%02x:%02x.%d config space: ", bus, slot, func);
    for(i=0;i<256;i+=4)
    {
	if(!(i & 0x0f))
	{
	    ebbrt::kprintf("\n  %02x:", i);
	}
	val = PciRead32(bus, slot, func, i);
	/*ebbrt::kprintf(" %02x%02x %02x%02x", ((val >> 8) & 0xff), (val & 0xff), ((val >> 24) % 0xff), ((val >> 16) & 0xff));*/

	for(j=0;j<4;j++)
	{
	    ebbrt::kprintf(" %02x", val & 0xff);
	    val >>= 8;
	}
	
    }
    ebbrt::kprintf("\n");
}

void early_dump_pci_devices()
{
    unsigned bus, slot, func;
    
    for(bus = 0; bus < 256; bus++)
    {
	for(slot = 0; slot < 32; slot ++)
	{
	    for(func = 0; func < 8; func ++)
	    {
		uint32_t clas;
		uint8_t type;
		
		clas = read_pci_config(bus, slot, func, 0x08);
		if(clas == 0xFFFFFFFF)
		    continue;

		early_dump_pci_device(bus, slot, func);
		if(func == 0)
		{
		    type = read_pci_config_byte(bus, slot, func, 0x0e);
		    if(!(type & 0x80))
			break;

		}
	    }
	}
    }
}

void ebbrt::pci::Init() {
    devices.construct();
    driver_probes.construct();
        
    
    EnumerateAllBuses();
    EnumerateBus(0x82);
    ebbrt::kabort("testing\n");
    PciScan();
    //early_dump_pci_device(0x82, 0x0, 0x0);
    
    
}

void ebbrt::pci::RegisterProbe(std::function<bool(pci::Device&)> probe) {
  driver_probes->emplace_back(std::move(probe));
}

void ebbrt::pci::LoadDrivers() {
  for (auto& dev : *devices) {
    for (auto& probe : *driver_probes) {
      if (probe(dev))
        break;
    }
  }
}

ebbrt::pci::Function::Function(uint8_t bus, uint8_t device, uint8_t func)
    : bus_(bus), device_(device), func_(func) {}

uint8_t ebbrt::pci::Function::Read8(uint8_t offset) const {
  return PciRead8(bus_, device_, func_, offset);
}

uint16_t ebbrt::pci::Function::Read16(uint8_t offset) const {
  return PciRead16(bus_, device_, func_, offset);
}

uint32_t ebbrt::pci::Function::Read32(uint8_t offset) const {
  return PciRead32(bus_, device_, func_, offset);
}

void ebbrt::pci::Function::Write16(uint8_t offset, uint16_t val) {
  return PciWrite16(bus_, device_, func_, offset, val);
}

void ebbrt::pci::Function::Write32(uint8_t offset, uint32_t val) {
  return PciWrite32(bus_, device_, func_, offset, val);
}

uint16_t ebbrt::pci::Function::GetVendorId() const {
  return Read16(kVendorIdAddr);
}
uint16_t ebbrt::pci::Function::GetDeviceId() const {
  return Read16(kDeviceIdAddr);
}
uint16_t ebbrt::pci::Function::GetCommand() const {
  return Read16(kCommandAddr);
}

uint8_t ebbrt::pci::Function::GetClassCode() const {
  return Read8(kClassCodeAddr);
}

uint8_t ebbrt::pci::Function::GetSubclass() const {
  return Read8(kSubclassAddr);
}

uint8_t ebbrt::pci::Function::GetProgIf() const {
  return Read8(kProgIfAddr);
}

uint8_t ebbrt::pci::Function::GetHeaderType() const {
  return Read8(kHeaderTypeAddr) & ~kHeaderMultifuncMask;
}

ebbrt::pci::Function::operator bool() const { return GetVendorId() == 0xffff; }

bool ebbrt::pci::Function::IsMultifunc() const {
  return Read8(kHeaderTypeAddr) & kHeaderMultifuncMask;
}

bool ebbrt::pci::Function::IsBridge() const {
  return GetHeaderType() == kHeaderTypeBridge;
}

void ebbrt::pci::Function::SetCommand(uint16_t cmd) {
  Write16(kCommandAddr, cmd);
}

void ebbrt::pci::Function::SetBusMaster(bool enable) {
  auto cmd = GetCommand();
  if (enable) {
    cmd |= kCommandBusMaster;
  } else {
    cmd &= ~kCommandBusMaster;
  }
  SetCommand(cmd);
}

void ebbrt::pci::Function::DisableInt() {
  auto cmd = GetCommand();
  cmd |= kCommandIntDisable;
  SetCommand(cmd);
}

void ebbrt::pci::Function::DumpAddress() const {
  kprintf("%u:%u:%u -- ", bus_, device_, func_);
}

void ebbrt::pci::Function::DumpInfo() const {
    kprintf("Vendor ID: 0x%x  ", GetVendorId());
    kprintf("Device ID: 0x%x\n", GetDeviceId());
    //kprintf("Class Code: 0x%x\n", GetClassCode());
    /*kprintf("Header Type: 0x%x\n", Read8(kHeaderTypeAddr));
    kprintf("Subclass: 0x%x\n", GetSubclass());
    kprintf("ProgIf: 0x%x\n", GetProgIf());*/
    //kprintf("test: 0x%x\n\n", Read16(kDeviceIdAddr));
}

ebbrt::pci::Bar::Bar(pci::Device& dev, uint32_t bar_val, uint8_t idx)
    : vaddr_(nullptr), is_64_(false), prefetchable_(false) {
  mmio_ = !(bar_val & kIoSpaceFlag);
  if (mmio_) {
    addr_ = bar_val & kMmioMask;
    is_64_ = (bar_val & kMmioTypeMask) == kMmioType64;
    prefetchable_ = bar_val & kMmioPrefetchableMask;

    dev.SetBarRaw(idx, 0xffffffff);
    auto low_sz = dev.GetBarRaw(idx) & kMmioMask;
    dev.SetBarRaw(idx, bar_val);

    uint32_t high_sz = ~0;
    if (is_64_) {
      auto bar_val2 = dev.GetBarRaw(idx + 1);
      addr_ |= static_cast<uint64_t>(bar_val2) << 32;
      dev.SetBarRaw(idx + 1, 0xffffffff);
      high_sz = dev.GetBarRaw(idx + 1);
      dev.SetBarRaw(idx + 1, bar_val2);
    }
    uint64_t sz = static_cast<uint64_t>(high_sz) << 32 | low_sz;
    size_ = ~sz + 1;
  } else {
    addr_ = bar_val & kIoSpaceMask;
    dev.SetBarRaw(idx, 0xffffffff);
    auto sz = dev.GetBarRaw(idx) & kIoSpaceMask;
    dev.SetBarRaw(idx, bar_val);

    size_ = ~sz + 1;
  }

  kprintf("PCI: %u:%u:%u - BAR%u %#018" PRIx64 " - %#018" PRIx64 "\n", dev.bus_,
          dev.device_, dev.func_, idx, addr_, addr_ + size_);
}

ebbrt::pci::Bar::~Bar() {
  kbugon(vaddr_ != nullptr, "pci::Bar: Need to free mapped region\n");
}

bool ebbrt::pci::Bar::Is64() const { return is_64_; }

void ebbrt::pci::Bar::Map() {
  if (!mmio_)
    return;

  auto npages = align::Up(size_, pmem::kPageSize) >> pmem::kPageShift;
  auto page = vmem_allocator->Alloc(npages);
  vaddr_ = reinterpret_cast<void*>(page.ToAddr());
  kbugon(page == Pfn::None(), "Failed to allocate virtual pages for mmio\n");
  vmem::MapMemory(page, Pfn::Down(addr_), size_);
}

uint8_t ebbrt::pci::Bar::Read8(size_t offset) {
  if (mmio_) {
    auto addr = static_cast<void*>(static_cast<char*>(vaddr_) + offset);
    return *static_cast<volatile uint8_t*>(addr);
  } else {
    return io::In8(addr_ + offset);
  }
}

uint16_t ebbrt::pci::Bar::Read16(size_t offset) {
  if (mmio_) {
    auto addr = static_cast<void*>(static_cast<char*>(vaddr_) + offset);
    return *static_cast<volatile uint16_t*>(addr);
  } else {
    return io::In16(addr_ + offset);
  }
}

uint32_t ebbrt::pci::Bar::Read32(size_t offset) {
  if (mmio_) {
    auto addr = static_cast<void*>(static_cast<char*>(vaddr_) + offset);
    return *static_cast<volatile uint32_t*>(addr);
  } else {
    return io::In32(addr_ + offset);
  }
}

void ebbrt::pci::Bar::Write8(size_t offset, uint8_t val) {
  if (mmio_) {
    auto addr = static_cast<void*>(static_cast<char*>(vaddr_) + offset);
    *static_cast<volatile uint8_t*>(addr) = val;
  } else {
    io::Out8(addr_ + offset, val);
  }
}

void ebbrt::pci::Bar::Write16(size_t offset, uint16_t val) {
  if (mmio_) {
    auto addr = static_cast<void*>(static_cast<char*>(vaddr_) + offset);
    *static_cast<volatile uint16_t*>(addr) = val;
  } else {
    io::Out16(addr_ + offset, val);
  }
}

void ebbrt::pci::Bar::Write32(size_t offset, uint32_t val) {
  if (mmio_) {
    auto addr = static_cast<void*>(static_cast<char*>(vaddr_) + offset);
    *static_cast<volatile uint32_t*>(addr) = val;
  } else {
    io::Out32(addr_ + offset, val);
  }
}

ebbrt::pci::Device::Device(uint8_t bus, uint8_t device, uint8_t func)
    : Function(bus, device, func), msix_bar_idx_(-1) {
  for (auto idx = 0; idx <= 6; ++idx) {
    auto bar = Read32(BarAddr(idx));
    if (bar == 0)
      continue;

    bars_[idx] = Bar(*this, bar, idx);

    if (bars_[idx]->Is64())
      idx++;
  }
}

uint8_t ebbrt::pci::Device::FindCapability(uint8_t capability) const {
  auto ptr = Read8(kCapabilitiesPtrAddr);
  while (ptr != 0) {
    auto cap = Read8(ptr);
    if (cap == capability)
      return ptr;

    ptr = Read8(ptr + 1);
  }

  return 0xFF;
}

uint32_t ebbrt::pci::Device::GetBarRaw(uint8_t idx) const {
  return Read32(BarAddr(idx));
}

void ebbrt::pci::Device::SetBarRaw(uint8_t idx, uint32_t val) {
  Write32(BarAddr(idx), val);
}

uint16_t ebbrt::pci::Device::MsixGetControl() {
  return Read16(msix_offset_ + kMsixControl);
}

void ebbrt::pci::Device::MsixSetControl(uint16_t control) {
  Write16(msix_offset_ + kMsixControl, control);
}

bool ebbrt::pci::Device::MsixEnabled() const { return msix_bar_idx_ != -1; }

ebbrt::pci::Bar& ebbrt::pci::Device::GetBar(uint8_t idx) {
  if (!bars_[idx])
    throw std::runtime_error("BAR does not exist\n");

  return *bars_[idx];
}

bool ebbrt::pci::Device::MsixEnable() {
  auto offset = FindCapability(kCapMsix);
  if (offset == 0xFF)
    return false;

  msix_offset_ = offset;

  auto control = MsixGetControl();
  msix_table_size_ = (control & kMsixControlTableSizeMask) + 1;
  auto table_offset_val = Read32(offset + kMsixTableOffset);
  msix_bar_idx_ = table_offset_val & kMsixTableOffsetBirMask;
  msix_table_offset_ = table_offset_val & ~kMsixTableOffsetBirMask;

  kprintf("MSIX - %u entries at BAR%u:%lx\n", msix_table_size_, msix_bar_idx_,
          msix_table_offset_);

  auto& msix_bar = GetBar(msix_bar_idx_);
  msix_bar.Map();

  DisableInt();

  auto ctrl = MsixGetControl();
  ctrl |= kMsixControlEnable;
  ctrl |= kMsixControlFunctionMask;
  MsixSetControl(ctrl);

  for (size_t i = 0; i < msix_table_size_; ++i) {
    MsixMaskEntry(i);
  }

  ctrl &= ~kMsixControlFunctionMask;
  MsixSetControl(ctrl);

  return true;
}

void ebbrt::pci::Device::MsixMaskEntry(size_t idx) {
  if (!MsixEnabled())
    return;

  if (idx >= msix_table_size_)
    return;

  auto& msix_bar = GetBar(msix_bar_idx_);
  auto offset =
      msix_table_offset_ + idx * kMsixTableEntrySize + kMsixTableEntryControl;
  auto ctrl = msix_bar.Read32(offset);
  ctrl |= kMsixTableEntryControlMaskBit;
  msix_bar.Write32(offset, ctrl);
}

void ebbrt::pci::Device::MsixUnmaskEntry(size_t idx) {
  if (!MsixEnabled())
    return;

  if (idx >= msix_table_size_)
    return;

  auto& msix_bar = GetBar(msix_bar_idx_);
  auto offset =
      msix_table_offset_ + idx * kMsixTableEntrySize + kMsixTableEntryControl;
  auto ctrl = msix_bar.Read32(offset);
  ctrl &= ~kMsixTableEntryControlMaskBit;
  msix_bar.Write32(offset, ctrl);
}

void ebbrt::pci::Device::SetMsixEntry(size_t entry, uint8_t vector,
                                      uint8_t dest) {
  auto& msix_bar = GetBar(msix_bar_idx_);
  auto offset = msix_table_offset_ + entry * kMsixTableEntrySize;
  msix_bar.Write32(offset + kMsixTableEntryAddr, 0xFEE00000 | dest << 12);
  msix_bar.Write32(offset + kMsixTableEntryData, vector);
  MsixUnmaskEntry(entry);
}
