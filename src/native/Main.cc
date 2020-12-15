//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#include "Main.h"

#include <cstdarg>
#include <cstring>
#include <stdexcept>

#include "../Console.h"
#include "../EbbAllocator.h"
#include "Acpi.h"
#include "Apic.h"
#include "Clock.h"
#include "Cpuid.h"
#include "Debug.h"
#include "E820.h"
#include "EarlyPageAllocator.h"
#include "EventManager.h"
#include "GeneralPurposeAllocator.h"
#ifdef __EBBRT_ENABLE_DISTRIBUTED_RUNTIME__
#include "GlobalIdMap.h"
#endif
#include "LocalIdMap.h"
#include "MemMap.h"
#include "Messenger.h"
#include "Multiboot.h"
#ifdef __EBBRT_ENABLE_NETWORKING__
#include "Net.h"
#endif
#include "Numa.h"
#include "PageAllocator.h"
#include "Pci.h"
#include "Pic.h"
#include "Random.h"
#ifdef __EBBRT_ENABLE_DISTRIBUTED_RUNTIME__
#include "Runtime.h"
#endif
#include "../Timer.h"
#include "SlabAllocator.h"
#include "Smp.h"
#include "Tls.h"
#ifdef __EBBRT_ENABLE_TRACE__
#include "Trace.h"
#endif
#include "Trans.h"
#include "VMem.h"
#include "VMemAllocator.h"
#ifdef __EBBRT_ENABLE_BAREMETAL_NIC__
#include "IxgbeDriver.h"
#else
#include "VirtioNet.h"
#endif

namespace {
bool started_once = false;
}

extern void AppMain() __attribute__((weak));

// for c++ runtime
extern char __eh_frame_start[];
extern "C" void __register_frame(void* frame);
void* __dso_handle;
extern void (*start_ctors[])();
extern void (*end_ctors[])();

extern "C" __attribute__((noreturn)) void
ebbrt::Main(multiboot::Information* mbi) {
  console::Init();

  memset(nsleep_states, 0, sizeof(nsleep_states));
  memset(sleep_state, 0, sizeof(sleep_state));
  sleep_state[0]=48;
  sleep_state[1]=48;
  sleep_state[2]=48;
  sleep_state[3]=48;
  sleep_state[4]=48;
  sleep_state[5]=48;
  sleep_state[6]=48;
  sleep_state[7]=48;
  sleep_state[8]=48;
  sleep_state[9]=48;
  sleep_state[10]=48;
  sleep_state[11]=48;
  sleep_state[12]=48;
  sleep_state[13]=48;
  sleep_state[14]=48;
  sleep_state[15]=48;  
  
  /*memset(processCnt, 0, sizeof(processCnt));
  memset(swEventCnt, 0, sizeof(swEventCnt));
  memset(idleEventCnt, 0, sizeof(idleEventCnt));
  memset(rxPollCnt, 0, sizeof(rxPollCnt));
  memset(processInterruptCntAll, 0, sizeof(processInterruptCntAll));
  memset(processInterruptCntA, 0, sizeof(processInterruptCntA));
  memset(processInterruptCntB, 0, sizeof(processInterruptCntB));
  memset(processInterruptCntC, 0, sizeof(processInterruptCntC));
  memset(passTokenCnt, 0, sizeof(passTokenCnt));
  memset(receiveTokenCnt, 0, sizeof(receiveTokenCnt));
  memset(genFireCnt, 0, sizeof(genFireCnt));
  memset(timerCnt, 0, sizeof(timerCnt));
  memset(fireCntA, 0, sizeof(fireCntA));
  memset(fireCntB, 0, sizeof(fireCntB));*/

#ifdef __EBBRT_ENABLE_TRACE__
  trace::Init();
#endif

  /* If by chance we reboot back into the kernel, panic */
  kbugon(started_once, "EbbRT reboot detected... aborting!\n");
  started_once = true;

  kprintf_force("EbbRT Copyright 2013-2018 Boston University SESA Developers\n");

  idt::Init();
  cpuid::Init();
  tls::Init();
  clock::Init();
  random::Init();
  pic::Disable();
  Cpu::EarlyInit();
  // bring up the first cpu structure early
  auto& cpu = Cpu::Create();
  cpu.set_acpi_id(0);
  cpu.set_apic_id(0);
  cpu.Init();

  e820::Init(mbi);
  e820::PrintMap();

  early_page_allocator::Init();
  multiboot::Reserve(mbi);
  vmem::Init();
  extern char kend[];
  auto kend_addr = reinterpret_cast<uint64_t>(kend);
  const constexpr uint64_t initial_map_end =
      1 << 30;  // we only map 1 gb initially
  e820::ForEachUsableRegion([=](e820::Entry entry) {
    if (entry.addr() + entry.length() <= kend_addr)
      return;

    auto begin = entry.addr();
    auto length = entry.length();

    entry.TrimBelow(kend_addr);
    entry.TrimAbove(initial_map_end);
    if (entry.addr() < initial_map_end) {
      early_page_allocator::FreeMemoryRange(entry.addr(), entry.length());
    }
    vmem::EarlyMapMemory(begin, length);
  });

  vmem::EnableRuntimePageTable();

  e820::ForEachUsableRegion([=](e820::Entry entry) {
    if (entry.addr() + entry.length() < initial_map_end)
      return;

    entry.TrimBelow(initial_map_end);
    early_page_allocator::FreeMemoryRange(entry.addr(), entry.length());
  });

  numa::EarlyInit();
  acpi::BootInit();
  numa::Init();
  mem_map::Init();
  trans::Init();
  PageAllocator::Init();
  slab::Init();
  GeneralPurposeAllocatorType::Init();
  LocalIdMap::Init();
  EbbAllocator::Init();
  VMemAllocator::Init();
  EventManager::Init();

  event_manager->SpawnLocal(
      [=]() {
        /// Enable exceptions
        __register_frame(__eh_frame_start);
        apic::Init();
        apic::PVEoiInit(0);
        Timer::Init();
        smp::Init();
        event_manager->ReceiveToken();

#ifdef __EBBRT_ENABLE_NETWORKING__
        NetworkManager::Init();
        pci::Init();

#ifdef __EBBRT_ENABLE_BAREMETAL_NIC__
        pci::RegisterProbe(IxgbeDriver::Probe);
#else
        pci::RegisterProbe(VirtioNetDriver::Probe);
#endif

        pci::LoadDrivers();
        network_manager->StartDhcp().Then([](Future<void> fut) {
          fut.Get();
// Dhcp completed
#ifdef __EBBRT_ENABLE_DISTRIBUTED_RUNTIME__
// Currently not supported in BMNIC since we don't pass arguments
// via grub
#ifndef __EBBRT_ENABLE_BAREMETAL_NIC__
          Messenger::Init();
          runtime::Init();
#endif
#endif
#endif
          // run global ctors
          for (unsigned i = 0; i < (end_ctors - start_ctors); ++i) {
            start_ctors[i]();
          }
          kprintf("System initialization complete\n");
          if (AppMain) {
            event_manager->SpawnLocal([=]() { AppMain(); });
          }
#ifdef __EBBRT_ENABLE_NETWORKING__
        });
#endif
      },
      /* force_async = */ true);
  event_manager->StartProcessingEvents();
}
