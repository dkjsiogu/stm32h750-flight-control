#include "flight_control/platform/host/host_synchronization.hpp"

namespace flight_control {

void HostMutexCriticalSection::lock() {
    mutex_.lock();
}

void HostMutexCriticalSection::unlock() {
    mutex_.unlock();
}

}  // namespace flight_control
