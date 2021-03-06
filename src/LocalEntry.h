//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#ifndef COMMON_SRC_INCLUDE_EBBRT_LOCALENTRY_H_
#define COMMON_SRC_INCLUDE_EBBRT_LOCALENTRY_H_

#include "EbbId.h"

namespace ebbrt {
struct LocalEntry {
  LocalEntry() : ref(nullptr) {}
  void* ref;
};

LocalEntry GetLocalEntry(EbbId id);
void SetLocalEntry(EbbId id, LocalEntry le);
}  // namespace ebbrt

#endif  // COMMON_SRC_INCLUDE_EBBRT_LOCALENTRY_H_
