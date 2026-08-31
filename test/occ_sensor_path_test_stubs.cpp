// Stub definitions for symbols pulled in transitively by
// occ_poll_kernel_handler.cpp but not needed by the sensor-path tests.
//
// occ_poll_kernel_handler.cpp includes occ_object.hpp which drags in:
//
//   OccObject  (key fn: occActive    — vtable/typeinfo emitted here)
//     └─ Device member
//         └─ Presence member (key fn: analyzeEvent — vtable/typeinfo emitted
//         here)
//             └─ Error base  (key fn: analyzeEvent — already in occ_errors.cpp)
//
// occ_errors.cpp is already in the test source list so Error's vtable is
// satisfied.  We only need to provide the key functions for Presence and
// OccObject, plus the two methods that were originally missing.

// occ_presence.hpp has a private 'manager' field that is only used in the
// real Presence::analyzeEvent() body.  Our stub leaves it empty, so clang
// warns about an unused private field.  Suppress just that diagnostic here.
// The guard is needed because GCC rejects #pragma clang diagnostic as an
// unknown pragma (which is -Werror under this project's build flags).
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-private-field"
#endif
#include "occ_presence.hpp"
#ifdef __clang__
#pragma clang diagnostic pop
#endif
#include "occ_object.hpp"

namespace open_power
{
namespace occ
{

// Presence::analyzeEvent is the key function for Presence; defining it here
// emits Presence's vtable and typeinfo.
void Presence::analyzeEvent() {}

// OccObject::occActive is the key function for OccObject; defining it here
// emits OccObject's vtable and typeinfo (required by RTTI in sanitiser builds).
bool OccObject::occActive(bool value)
{
    return value;
}

fs::path OccObject::getHwmonPath()
{
    return {};
}

bool Device::master() const
{
    return false;
}

} // namespace occ
} // namespace open_power
