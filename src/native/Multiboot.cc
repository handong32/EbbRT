//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#include "Multiboot.h"

#include <cstring>

#include "EarlyPageAllocator.h"
#include "Debug.h"

uintptr_t ebbrt::multiboot::cmdline_addr_;
uintptr_t ebbrt::multiboot::cmdline_len_;
ebbrt::multiboot::Information* ebbrt::multiboot::info;

void ebbrt::multiboot::Reserve(Information* mbi) {
  ebbrt::kprintf("%s\n", __PRETTY_FUNCTION__);
  info = mbi;
  auto mbi_addr = reinterpret_cast<uintptr_t>(mbi);
  early_page_allocator::ReserveRange(mbi_addr, mbi_addr + sizeof(*mbi));
  if (mbi->has_command_line_) {
    ebbrt::kprintf("\t mbi->has_command_line_\n");
    cmdline_addr_ = mbi->command_line_;
    auto cmdline = reinterpret_cast<const char*>(cmdline_addr_);
    ebbrt::kprintf("\t %s %p\n", cmdline, cmdline_addr_);
    cmdline_len_ = std::strlen(cmdline);
    early_page_allocator::ReserveRange(cmdline_addr_,
                                       cmdline_addr_ + cmdline_len_ + 1);
  }

  if (mbi->has_boot_modules_ && mbi->modules_count_ > 0) {
    uintptr_t module_addr = mbi->modules_address_;
    auto module_end = module_addr + mbi->modules_count_ * sizeof(Module);
    early_page_allocator::ReserveRange(module_addr, module_end);
    auto mods = reinterpret_cast<Module*>(module_addr);
    for (uint32_t i = 0; i < mbi->modules_count_; ++i) {
      early_page_allocator::ReserveRange(mods[i].start, mods[i].end);
      uintptr_t str_addr = mods[i].string;
      auto str = reinterpret_cast<const char*>(str_addr);
      auto len = std::strlen(str);
      early_page_allocator::ReserveRange(str_addr, str_addr + len + 1);
    }
  }

  // TODO(dschatz): reserve ELF section headers and others
}

const char* ebbrt::multiboot::CmdLine() {
  //ebbrt::kprintf("%s %p %s\n", __PRETTY_FUNCTION__, cmdline_addr_, reinterpret_cast<const char*>(cmdline_addr_));
  return reinterpret_cast<const char*>(cmdline_addr_);
}
