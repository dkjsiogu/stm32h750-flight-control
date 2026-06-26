#include "flight_control/runtime/synchronization.hpp"

namespace flight_control {

CriticalSectionGuard::CriticalSectionGuard(ICriticalSection& section)
    : section_(section) {
    section_.lock();
}

CriticalSectionGuard::~CriticalSectionGuard() {
    section_.unlock();
}

void NullCriticalSection::lock() {}

void NullCriticalSection::unlock() {}

}  // namespace flight_control
