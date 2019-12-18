#include "Debug.h"
//#include "Msr.h"
#include "Rapl.h"

ebbrt::rapl::RaplCounter::~RaplCounter() {
  return;
}

double ebbrt::rapl::RaplCounter::Read() {
  return counter_offset;
}
