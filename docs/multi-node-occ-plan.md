# Multi-Chassis OCC Support Plan

## Overview

Add support for up to 8 chassis (expandable to 12). A single BMC manages all
chassis. Initially one OCC per chassis is supported, with the architecture
designed to allow multiple OCCs per chassis in the future. The OCC in each
chassis is the master for that chassis.

Key constraints driving the design:

- Chassis are numbered **1–8** (chassis 0 is reserved for the patch panel).

  > **Terminology note:** Internally this design uses the term _chassis_.
  > Customers and field/PE personnel will typically refer to the same hardware
  > unit as a _node_ — the two terms are interchangeable. Chassis 0 is the patch
  > panel and contains no processors or OCCs, so **the first processor node
  > visible to a customer corresponds to chassis 1**.

  The kernel `/dev/occN` device indices are **independent** of chassis numbering
  — `/dev/occ0` is not necessarily chassis 1. The kernel device index is an
  opaque handle derived from `path.back() - '0'` as today; `chassisID` is
  sourced from PLDM / inventory and stored explicitly.

- sysfs device is `occ-hwmon.<N>` where N is the zero-based OCC instance within
  the chassis (always 0 today; no +1 offset)
- Initially one OCC per chassis; that OCC is always the master for its chassis.
  The D-Bus path structure (`chassis<N>/occ0`) and data structures are designed
  to accommodate multiple OCCs per chassis later
- All system-wide data (power mode, IPS, power cap) is broadcast to **every
  active chassis's master OCC**
- IPS is disabled when there is more than one chassis; it is only allowed on a
  single-chassis system
- Safe mode is **per-chassis** — when chassis N enters safe mode, other chassis
  continue operating normally
- PLDM safe mode event does **not** yet carry a chassis identifier; it must be
  extended
- Maximum 8 chassis (future: 12); this is a compile-time constant
  (`MAX_CHASSIS`)

### Polling Model

This plan defaults to **app polling** (`ENABLE_APP_POLL_SUPPORT`) but preserves
kernel polling support (`OccPollKernelHandler`). The existing
`#ifdef ENABLE_APP_POLL_SUPPORT` guards in `occ_status.hpp` (later
`occ_object.hpp`) remain in place — both code paths must continue to compile and
function correctly. The kernel poll handler files
(`occ_poll_kernel_handler.hpp/cpp`) are left untouched.

The sole app-poll path construction in `occ_poll_app_handler.cpp:38-39` builds
an `OccCommand` using the old `OCC_CONTROL_ROOT / occ<N>` path — this must be
updated as part of Sub-Task 7.

### What Is Not Changing

- D-Bus paths for power mode / IPS / power cap
  (`/xyz/openbmc_project/control/host0/...`) remain unchanged — these are
  system-wide objects
- The `PassThrough` interface, PLDM active-OCC handling, and error/FFDC handling
  logic are preserved
- The existing `OccCommand`, `Device`, and `OccPollHandler` abstract interface
  are not being redesigned — only their instantiation and per-chassis dispatch
  changes
- `OccPollKernelHandler` is not modified

---

## Target Class Organization

```text
BMC
└── Manager  (occ_manager.hpp)
    │
    ├── PowerMode  (powermode.hpp)
    │   ├── occCmds[0..MAX_CHASSIS]  OccCommand  ← sends SET_MODE to each chassis master
    │   └── chassisActive[0..MAX_CHASSIS]
    │
    ├── PowerCap  (powercap.hpp)
    │   └── chassisOccObjs[0..N]  OccObject*  ← writes power_cap_user hwmon per chassis
    │
    ├── SystemInfo  (occ_system_info.hpp)
    │   └── D-Bus object at /org/open_power/control
    │       ActiveChassisCount, TotalChassisCount, SystemSafeMode, ChassisPaths
    │
    ├── pldmHandle  pldm::Interface  (pldm.hpp)
    │   └── safeModeCallBack(chassisID, bool)  ← extended for per-chassis safe mode
    │
    └── chassisObjects  map[chassisID -> ChassisObject]  (occ_chassis.hpp)
        │
        └── ChassisObject  (one per chassis, up to MAX_CHASSIS)
            │   chassis        chassisID   (1-based: 1–8)
            │   active         bool
            │   safeMode       bool
            │   resetRequired  bool
            │   resetInProgress bool
            │   waitForAllOccsTimer
            │
            ├── occObjects[0..N]  OccObject  (occ_object.hpp, renamed from occ_status)
            │   │   instance       uint8_t      (0-based within chassis; always 0 today)
            │   │   occCmd         OccCommand   ← /dev/occN  (kernel index, not chassisID)
            │   │   device         Device       ← occ-hwmon.N sysfs
            │   │   occPollObj     OccPollAppHandler
            │   │   throttleCause  uint8_t
            │   │   safeStateDelayTimer
            │   └── (hwmonPath, lastState, sensor validity flags, ...)
            │
            └── passThroughObjects[0..N]  PassThrough
                    chassis    chassisID   ← new explicit field
                    occCmd     OccCommand
                    devicePath /dev/occN   (kernel index)
```

**D-Bus tree (after Sub-Task 8):**

```text
/org/open_power/control                          ← SystemInfo root object
    .ActiveChassisCount  .TotalChassisCount
    .SystemSafeMode      .ChassisPaths[]

/org/open_power/control/chassis1/occ0            ← OccObject D-Bus presence + PassThrough
/org/open_power/control/chassis2/occ0
...
/org/open_power/control/chassisN/occ0

/xyz/openbmc_project/control/host0/power_mode      ← PowerMode  (unchanged)
/xyz/openbmc_project/control/host0/power_ips       ← IPS        (single-chassis only)
/xyz/openbmc_project/control/host0/power_cap_limits ← PowerCap  (unchanged)
```

---

## Sub-Tasks

### Sub-Task 1 — Introduce `chassisID` type and `MAX_CHASSIS` constant

**Status:** `[x] complete`

**Intent:** Establish the foundational types and constants so that all
subsequent sub-tasks can use them consistently throughout the codebase.

**Expected Outcomes:**

- A `chassisID` type alias (`uint8_t`) is defined in `utils.hpp` under namespace
  `open_power::occ` to avoid cyclic header dependencies.
- A `MAX_CHASSIS` compile-time constant (default 8) is added in `meson.options`
  and `meson.build`, generating `MAX_CHASSIS` in `config.h` (similar to
  `MAX_CPUS`)
- Existing code compiles without change — these are purely additive

**Todo List:**

1. Add `option('max-chassis', type: 'integer', min: 1, max: 12, value: 8)` to
   `meson.options`
2. Wire `MAX_CHASSIS` into `meson.build` the same way `MAX_CPUS` is handled so
   it appears in `config.h`
3. Add `using chassisID = uint8_t;` inside the namespace `open_power::occ` in
   `utils.hpp`.

**Relevant Context:**

- [`meson.options`](meson.options) — existing `max-cpus` option for reference
- [`meson.build`](meson.build) — existing `MAX_CPUS` config generation for
  reference
- [`utils.hpp`](utils.hpp) — location for the new `chassisID` type definition

---

### Sub-Task 2 — Remove all `/dev/occX` and sysfs `+1` offsets

**Status:** `[x] complete`

**Intent:** Under the new device scheme, `/dev/occN` is an opaque kernel index
with no semantic tie to chassis numbering. The code had two inconsistencies that
have been fixed:

1. `PassThrough` was adding `+1` when building its `/dev/occX` path — removed.
2. `OccObject` (previously `Status`) was using `instance + 1` when naming the
   sysfs hwmon device (`occ-hwmon.1` for instance 0); now correctly
   `occ-hwmon.0`.

`OccCommand` already used `path.back() - '0'` without any offset and was already
correct.

**Expected Outcomes:**

- `OccCommand::devicePath` → `/dev/occ<N>` (no change needed; verified and
  documented)
- `PassThrough::devicePath` → `/dev/occ<N>` with no `+1`
- `OccObject` device sysfs path → `occ-hwmon.<N>` with no `+1`

**Todo List:**

1. ~~In `occ_pass_through.cpp:28`, remove the `+1` from the device path
   construction~~ ✓ done
2. ~~In `occ_status.hpp:101` (renamed to `occ_object.hpp` in Sub-Task 3), change
   `std::to_string(instance + 1)` to `std::to_string(instance)`~~ ✓ done
3. ~~Confirm `occ_command.cpp:56` uses `path.back() - '0'` directly (no +1); add
   a comment documenting the kernel-device mapping~~ ✓ done

**Relevant Context:**

- [`occ_command.cpp:54-65`](occ_command.cpp:54) — `OccCommand` constructor,
  device path (already correct, comment added)
- [`occ_object.cpp:372-373`](occ_object.cpp:372) — `occ-hwmon.<instance>` sysfs
  path (fixed, no +1)
- [`occ_pass_through.cpp:28`](occ_pass_through.cpp:28) — PassThrough devicePath
  (fixed, no +1)

---

### Sub-Task 3 — Rename `Status` → `OccObject` and `occ_status` → `occ_object`

**Status:** `[x] complete`

**Intent:** Rename the `Status` class to `OccObject` (parallel to
`ChassisObject` in the new hierarchy) and rename the source files from
`occ_status.hpp/cpp` to `occ_object.hpp/cpp`. This is a purely mechanical rename
with no behavioral change — doing it as its own sub-task keeps the diff easy to
review before the structural changes in Sub-Task 4 begin.

The existing `instance` field already carries its identity. `OccObject` does not
need to know which chassis it is associated with.

**Expected Outcomes:**

- `Status` class renamed to `OccObject` everywhere in the codebase
- Source files renamed: `occ_status.hpp` → `occ_object.hpp`, `occ_status.cpp` →
  `occ_object.cpp`
- All `#include "occ_status.hpp"` → `#include "occ_object.hpp"` throughout
- `meson.build` updated to compile `occ_object.cpp` instead of `occ_status.cpp`
- No behavioral change — compile and run identical to before

**Todo List:**

1. Rename `occ_status.hpp` to `occ_object.hpp` and `occ_status.cpp` to
   `occ_object.cpp`
2. Rename the `Status` class to `OccObject` within those files; update the
   include guards
3. Update all `#include "occ_status.hpp"` references in every file that includes
   it
4. Update `meson.build` to build `occ_object.cpp` instead of `occ_status.cpp`
5. Global search for any remaining `Status` references that refer to the OCC
   status class (not the D-Bus `Base::Status` interface typedef) and rename them
   to `OccObject`

**Relevant Context:**

- [`occ_status.hpp`](occ_status.hpp) — class definition to rename →
  `occ_object.hpp`
- [`occ_status.cpp`](occ_status.cpp) — implementation to rename →
  `occ_object.cpp`
- [`occ_status.hpp:91-126`](occ_status.hpp:91) — constructor and instance
  derivation
- [`occ_manager.hpp`](occ_manager.hpp) — includes `occ_status.hpp`, uses
  `Status` type
- [`occ_manager.cpp`](occ_manager.cpp) — uses `Status` in `createObjects()`,
  `statusCallBack()`
- [`occ_manager.cpp:574`](occ_manager.cpp:574) — `find_if` by
  `getOccInstanceID()`
- [`occ_device.hpp`](occ_device.hpp) — forward-declares or includes `Status`
- [`meson.build`](meson.build) — lists source files

---

### Sub-Task 4 — Introduce `ChassisObject` class

**Status:** `[ ] pending`

**Intent:** Define the `ChassisObject` class that will own all per-chassis state
and the collection of OCC objects for that chassis. This is a purely additive
step — the new file is created and wired into the build, but `Manager` is not
yet changed. Sub-Task 5 then migrates `Manager` to use it.

**What `ChassisObject` owns:**

- `std::vector<std::unique_ptr<OccObject>> occObjects` — the OCCs on this
  chassis (one today, multiple later); replaces and renames `statusObjects`
- `std::vector<std::unique_ptr<PassThrough>> passThroughObjects` — one per OCC
  on this chassis
- `chassisID chassis` — this chassis's identifier (1-based)
- `bool active` — true when this chassis's OCCs are active
- `bool safeMode` — true when this chassis is in safe mode; used by Sub-Task 6
- `bool resetRequired` / `bool resetInProgress` — per-chassis reset state; used
  by Sub-Task 10
- Reference to the master `OccObject` for this chassis (the first/only OCC
  today), exposed for `PowerMode` and `PowerCap` to use

**What stays in `OccObject` (per-OCC, renamed from `Status`):**

- `instance`, `throttleCause`, `safeStateDelayTimer`, `occCmd`, `device`,
  `hwmonPath`, `lastState`, sensor validity flags — all truly per-OCC, not
  per-chassis

**Expected Outcomes:**

- `occ_chassis.hpp` exists and compiles cleanly
- `ChassisObject` is listed in `meson.build`
- No existing code is changed — `Manager` still uses `statusObjects` until
  Sub-Task 5

**Todo List:**

1. Create `occ_chassis.hpp` defining `ChassisObject` with the members listed
   above; `Manager` and `OccObject` are forward-declared as needed
2. Add `occ_chassis.hpp` to the build in `meson.build`

**Relevant Context:**

- [`occ_object.hpp`](occ_object.hpp) — `OccObject` class (renamed from `Status`
  in Sub-Task 3)
- [`occ_manager.hpp:176-212`](occ_manager.hpp:176) — data members that Sub-Task
  5 will restructure

---

### Sub-Task 5 — Update `Manager` to use `ChassisObject`

**Status:** `[ ] pending`

**Intent:** Migrate `Manager` from its current flat `statusObjects` /
`passThroughObjects` vectors to a
`std::map<chassisID, std::unique_ptr<ChassisObject>> chassisObjects` container,
using the `ChassisObject` class introduced in Sub-Task 4. The rename of `Status`
→ `OccObject` is already done by Sub-Task 3; this sub-task uses `OccObject`
throughout.

The existing `statusObjects` vector is renamed to `occObjects` throughout — both
as the member name in `ChassisObject` and across all source files that reference
it.

**What stays in `Manager`:**

- `std::map<chassisID, std::unique_ptr<ChassisObject>> chassisObjects` — the
  chassis-keyed container
- `uint8_t activeCount` — system-wide count of active OCCs (sum across all
  chassis)
- `uint8_t numChassis` — how many chassis were discovered (replaces
  `statusObjects.size()`)
- System-wide timers, ambient/altitude, PLDM handle, power mode, power cap

**Expected Outcomes:**

- Chassis N's master `OccObject` is accessed via
  `chassisObjects[n]->getMasterOcc()`; `Manager` never indexes `occObjects`
  directly
- `chassisObjects[n]->passThroughObjects[0]` similarly
- `statusObjects` no longer exists anywhere in the codebase; all per-OCC
  operations on a chassis are delegated to member functions of `ChassisObject`
  (`Status` → `OccObject` already done in Sub-Task 3)
- Per-chassis state (`active`, `safeMode`, `resetRequired`, `resetInProgress`)
  lives in `ChassisObject`, not in parallel maps/arrays in `Manager`
- All `find_if` searches replaced with
  `chassisObjects[instance]->getMasterOcc()`
- All loops over OCCs become
  `for (auto& [chassis, chassisObj] : chassisObjects)` with `Manager` delegating
  per-OCC work via member functions on `ChassisObject` rather than directly
  iterating `occObjects`

**Todo List:**

1. Add `std::map<chassisID, std::unique_ptr<ChassisObject>> chassisObjects` to
   `occ_manager.hpp`; remove `statusObjects` and `passThroughObjects` members
2. Add `uint8_t numChassis = 0` to `occ_manager.hpp`
3. Rename all remaining references to `statusObjects` in `occ_manager.cpp` to
   the appropriate `ChassisObject` member function call (search-replace starting
   point, then adjust per-access semantics)
4. Update `createObjects(occ)` to extract chassis number, construct or retrieve
   `chassisObjects[chassis]`, then call `chassisObjects[chassis]->addOcc(...)`
   and `chassisObjects[chassis]->addPassThrough(...)` to append the new objects,
   and increment `numChassis`
5. Replace all `statusObjects.size()` comparisons with `numChassis`
6. Replace all `find_if` calls (lines 551, 574, 660) with
   `chassisObjects[instance]->getMasterOcc()`
7. Update `statusCallBack(instance, status)` to set
   `chassisObjects[instance]->active = status` alongside the existing
   `activeCount` update
8. Update all loops in `Manager` to iterate `chassisObjects` and delegate
   per-OCC work via a method on `ChassisObject` (e.g.
   `chassisObj->forEachOcc(...)`) rather than directly accessing `occObjects`
   from `Manager`
9. Move `validateOccMaster()` into `ChassisObject` as a per-chassis method; the
   master-detection logic is retained so it can correctly handle multiple OCCs
   per chassis in the future — `Manager` calls `chassisObj->validateOccMaster()`
   for each chassis

**Relevant Context:**

- [`occ_chassis.hpp`](occ_chassis.hpp) — `ChassisObject` class from Sub-Task 4
- [`occ_object.hpp`](occ_object.hpp) — `OccObject` class (renamed from `Status`
  in Sub-Task 3)
- [`occ_manager.hpp:176-212`](occ_manager.hpp:176) — data members to restructure
- [`occ_manager.cpp:326-357`](occ_manager.cpp:326) — `createObjects()` — uses
  `OccObject`
- [`occ_manager.cpp:410-547`](occ_manager.cpp:410) — `statusCallBack()` — uses
  `statusObjects`
- [`occ_manager.cpp:431`](occ_manager.cpp:431) —
  `activeCount == statusObjects.size()` → `numChassis`
- [`occ_manager.cpp:551`](occ_manager.cpp:551) — `find_if` in `sbeTimeout()`
- [`occ_manager.cpp:574`](occ_manager.cpp:574) — `find_if` in
  `updateOCCActive()`
- [`occ_manager.cpp:660`](occ_manager.cpp:660) — `find_if` in
  `sbeHRESETResult()`
- [`occ_manager.cpp:812-857`](occ_manager.cpp:812) — `pollerTimerExpired()`
- [`occ_manager.cpp:1093-1168`](occ_manager.cpp:1093) — `validateOccMaster()` to
  be moved into `ChassisObject`

---

### Sub-Task 6 — Per-chassis safe mode isolation

**Status:** `[ ] pending`

**Intent:** Currently `updateOccSafeMode(bool)` applies safe mode globally to
ALL OCCs and updates the single `SafeMode` D-Bus property. Under the new model,
safe mode is per-chassis: only the OCC in the affected chassis is throttled;
other chassis remain unaffected.

The PLDM `safeModeCallBack` signature is `std::function<void(bool)>` — it does
not carry a chassis identifier. Both the PLDM interface and its caller in
`Manager` must be extended to pass a `chassisID`. In `pldm.cpp` the safe mode
trigger occurs at the point where the active-OCC state sensor reads DORMANT; the
`instance` is already known at both call sites (line 186 and line 978), so the
chassis ID is available and simply needs to be forwarded.

The D-Bus `SafeMode` property on `PowerMode` remains system-wide: it is `true`
only when all present chassis are in safe mode, `false` when any chassis is
operating normally.

**Expected Outcomes:**

- `pldm::Interface::safeModeCallBack` signature changes to
  `std::function<void(chassisID, bool)>`
- In `pldm.cpp`, both call sites pass `instance` as the chassis ID:
  `safeModeCallBack(instance, true)`
- `Manager::updateOccSafeMode(chassisID chassis, bool safeMode)` delegates to
  `chassisObjects[chassis]->setSafeMode(safeMode)` which sets the chassis's
  `safeMode` flag and throttles its OCCs
- System-wide `SafeMode` D-Bus property is true only when all active chassis are
  in safe mode
- A chassis going safe does not stop polling or affect OCCs in other chassis

**Todo List:**

1. In `pldm.hpp:81`, change `std::function<void(bool)> safeModeCallBack` to
   `std::function<void(chassisID, bool)> safeModeCallBack`
2. In `pldm.cpp:186` and `pldm.cpp:978`, pass `instance` as the first argument:
   `safeModeCallBack(instance, true)`
3. In `occ_manager.hpp:249`, update `updateOccSafeMode` signature to
   `(chassisID chassis, bool safeState)`
4. In `occ_manager.cpp:639-647`, update `updateOccSafeMode` body: call
   `chassisObjects[chassis]->setSafeMode(safeMode)` to set the flag and throttle
   that chassis's OCCs; then compute the system-wide aggregate (`all_of` over
   `chassisObjects` checking `->safeMode`) and call
   `pmode->updateDbusSafeMode(allSafe)`
5. Update the `createPldmHandle()` bind in `occ_manager.cpp:39` to match the new
   signature

**Relevant Context:**

- [`pldm.hpp:72-84`](pldm.hpp:72) — `safeModeCallBack` member and constructor
  parameter
- [`pldm.cpp:177-186`](pldm.cpp:177) — `sensorEvent` safe mode trigger
  (`instance` available)
- [`pldm.cpp:970-978`](pldm.cpp:970) — `pldmRspCallback` safe mode trigger
  (`instance` available)
- [`occ_manager.cpp:32-42`](occ_manager.cpp:32) — `createPldmHandle()` bind
- [`occ_manager.cpp:639-647`](occ_manager.cpp:639) — `updateOccSafeMode()`

---

### Sub-Task 7 — Broadcast power mode, IPS, and power cap to all active chassis

**Status:** `[ ] pending`

**Intent:** All system-wide data commands must be sent to the master OCC in
**every** active chassis. `PowerMode` and `PowerCap` do not hold OCC references
directly — instead they call an interface on `ChassisObject` to trigger the send
or write, keeping OCC access encapsulated in `ChassisObject`:

- **Power mode** — `PowerMode` calls `chassisObj->sendModeData(...)` for each
  active chassis; `ChassisObject` forwards the command to its master OCC via
  `OccCommand`
- **IPS** — same delegation pattern; **disabled when more than one chassis is
  present** (IPS is only meaningful for single-chassis systems)
- **Power cap** — `PowerCap` calls `chassisObj->writePcap(value)` for each
  active chassis; `ChassisObject` writes to the hwmon sysfs `power_cap_user`
  file of its master OCC

#### PowerMode changes

`PowerMode` currently holds a single `occCmd` (`OccCommand`) and single
`masterActive`/`masterOccSet` booleans. These are replaced with a reference to
`Manager`'s `chassisObjects` map so `sendModeChange()` and `sendIpsData()` can
iterate active chassis and delegate to each `ChassisObject`.

IPS object creation is gated on the total **discovered/configured chassis
count** (`numChassis` or `chassisObjects.size()`) being exactly 1. This prevents
the D-Bus IPS object from "flickering" (creating and deleting) during boot if
chassis come online sequentially. `sendIpsData()` is a no-op when more than one
chassis is present in the system.

#### PowerCap changes

`PowerCap::writeOcc()` currently writes a single `power_cap_user` sysfs file
directly. With multiple chassis, `writeOcc()` iterates the active
`chassisObjects` and calls `chassisObj->writePcap(value)` on each, delegating
the hwmon path lookup and file write to `ChassisObject`.

Defensive programming is added in `PowerCap::updatePcapBounds()` when querying
chassis 1's bounds to ensure that chassis 1 exists in the map and contains a
valid master OCC before attempting access.

**Expected Outcomes:**

- `PowerMode` no longer owns `occCmd` or per-chassis OCC references;
  `sendModeChange()` iterates active chassis and calls
  `chassisObj->sendModeData(...)` on each
- `sendModeChange()` skips chassis where `chassisObj->active` is false
- IPS object and `sendIpsData()` gated: only active when the total discovered
  chassis count is exactly 1 (prevents boot flickering)
- `PowerCap::writeOcc(pcapValue)` iterates active chassis and calls
  `chassisObj->writePcap(value)` on each; `ChassisObject` handles the hwmon path
  and file write
- `PowerCap::updatePcapBounds()` safely checks if chassis 1 is present and has a
  valid master OCC, then calls `chassisObjects[1]->getPcapBounds()` to read
  system-wide bounds

**Todo List:**

_PowerMode:_

1. In `powermode.hpp`, remove `std::unique_ptr<OccCommand> occCmd`,
   `int occInstance`, `bool masterOccSet`, `bool masterActive`; add a reference
   to `Manager`'s `chassisObjects` map
2. Replace `setMasterOcc(const std::string&)` and `setMasterActive(bool)` with
   `setChassisActive(chassisID chassis, bool active)` that updates the chassis's
   active state
3. In `sendModeChange()`, replace the single `occCmd->send(...)` with a loop
   over active chassis calling `chassisObj->sendModeData(...)`; log per-chassis
   success/failure
4. In `sendIpsData()`, add an early-return guard: if the number of
   discovered/configured chassis is greater than 1, log and skip (IPS not
   supported in multi-chassis configuration)
5. Gate `createIpsObject()` / `removeIpsObject()` on single-chassis-only: when
   the total discovered chassis count is greater than 1, do not create the IPS
   object
6. Update the "master not active" guard in `sendModeChange()` to check that no
   chassis is active
7. In `occ_manager.cpp:statusCallBack`, call
   `pmode->setChassisActive(chassis, true/false)`

_ChassisObject (mode/cap send):_

1. Add `sendModeData(...)` method to `ChassisObject` that forwards the mode
   command to the master OCC via its `OccCommand`
2. Add `writePcap(value)` method to `ChassisObject` that writes `power_cap_user`
   to the master OCC's hwmon sysfs path
3. Add `getPcapBounds()` method to `ChassisObject` that reads the power cap
   bounds from the master OCC's hwmon sysfs path

_PowerCap:_

1. Remove `masterOccObj` from `powercap.hpp`; replace `writeOcc()` to iterate
   active chassis and call `chassisObj->writePcap(value)` on each
2. Replace `setMasterOccObj(OccObject&)` with `setChassisObjects(...)` that
   gives `PowerCap` access to the `chassisObjects` map
3. In `updatePcapBounds()`, defensively verify that `chassisObjects` contains
   key `1` (first chassis) and its master OCC is not null, then call
   `chassisObjects[1]->getPcapBounds()` to read system-wide bounds

_occ_object.cpp:_

1. Change `pmode->setMasterActive(false)` calls to
   `pmode->setChassisActive(chassis, false)` and `pmode->setMasterActive()` to
   `pmode->setChassisActive(chassis, true)`

**Relevant Context:**

- [`powermode.hpp:345-370`](powermode.hpp:345) — private members: `occCmd`,
  `occInstance`, `masterOccSet`, `masterActive`
- [`powermode.cpp:141-166`](powermode.cpp:141) — `setMasterOcc()`
- [`powermode.cpp:434-543`](powermode.cpp:434) — `sendModeChange()`
- [`powermode.cpp:708-790`](powermode.cpp:708) — `sendIpsData()`
- [`powermode.cpp:104-136`](powermode.cpp:104) — `createIpsObject()` /
  `removeIpsObject()`
- [`powercap.hpp:170-232`](powercap.hpp:170) — `setMasterOccObj`,
  `masterOccObj`, `writeOcc`
- [`powercap.cpp:257-294`](powercap.cpp:257) — `writeOcc()` — writes single
  hwmon file
- [`powercap.cpp:218-230`](powercap.cpp:218) — `getPcapFilename()` — reads from
  `masterOccObj`
- [`occ_object.cpp:93`](occ_object.cpp:93) and
  [`occ_object.cpp:172`](occ_object.cpp:172) — `setMasterActive` calls
- [`occ_manager.cpp:345-353`](occ_manager.cpp:345) — `createObjects` master
  detection
- [`occ_manager.cpp:1170-1187`](occ_manager.cpp:1170) — `updatePcapBounds()`

---

### Sub-Task 8 — Update D-Bus object paths to chassis/occ subtree

**Status:** `[ ] pending`

**Intent:** OCC Status D-Bus objects are currently at
`/org/open_power/control/occ<N>`. With multiple OCCs per chassis anticipated in
the future, adopt the subtree form now:
`/org/open_power/control/chassis<N>/occ0`. This cleanly separates the chassis
identifier from the per-chassis OCC index and makes the tree extensible without
a future rename.

Since there is currently only one OCC per chassis, the OCC index within the
chassis subtree is always `occ0`. Chassis numbering is 1-based (chassis 1–8);
chassis 0 is the patch panel and is never represented in the D-Bus tree.

The system-wide power mode, IPS, and power cap paths
(`/xyz/openbmc_project/control/host0/...`) are **not** changed.

The key challenge is that `getInstance()` and every other place that extracts a
number from the OCC D-Bus path currently uses `path.back() - '0'` — a
single-character shortcut that reads the last digit of the path. With the new
format, the chassis number must be parsed from the `chassis<N>` segment instead,
and must handle multi-digit chassis numbers (e.g., `chassis10` for a future
12-chassis system).

**Expected Outcomes:**

- OCC Status and PassThrough D-Bus paths:
  `/org/open_power/control/chassis<N>/occ0`
- A shared helper `getChassisFromPath(const std::string& path) -> chassisID` is
  introduced in `utils.hpp` that parses the `chassis<N>` segment with
  robustness: it searches for the `"chassis"` token, parses any multi-digit
  number, but falls back to `path.back() - '0'` if `"chassis"` is absent
  (ensuring complete backward compatibility with legacy test paths and
  transitions).
- `OccObject::getInstance(path)` delegates to `getChassisFromPath()`
- `OccObject::getDbusPath()` continues to work — it uses `getInstance()` to key
  the sensor map, and the sensor map is keyed on instance/chassis number so no
  map change is needed
- `OccCommand`, `PassThrough`, and `powermode.cpp` all use
  `getChassisFromPath()` for device and instance number derivation
- `OccObject` exposes a public `getPath()` method so other classes can retrieve
  its path directly
- `occ_poll_app_handler.cpp` gets the D-Bus path directly from the parent
  `statusObject.getPath().c_str()`, completely avoiding hardcoded path
  formatting and duplications
- `app.cpp` D-Bus object manager root (`OCC_CONTROL_ROOT`) is unchanged

**Todo List:**

1. In `utils.hpp`, add a robust
   `getChassisFromPath(const std::string& path) -> chassisID` free function
   (with a fallback to legacy `path.back() - '0'` extraction if the token
   `"chassis"` is absent; avoid regex)
2. In `occ_object.hpp`, replace `path.back() - '0'` in `getInstance()` with a
   call to `getChassisFromPath(path)`
3. In `occ_command.cpp:56`, replace `this->path.back() - '0'` with
   `getChassisFromPath(this->path)`
4. In `occ_pass_through.hpp/cpp`, add an explicit `chassisID chassis`
   constructor parameter so the chassis is stored directly rather than derived
   from the path. Update `occInstance` to be initialised from `chassis` rather
   than `path.back() - '0'`. This makes the interface forward-compatible: when
   multiple OCCs per chassis exist, `PassThrough` will already carry the chassis
   as a first-class field alongside the OCC index derived from the path
5. In `occ_manager.cpp:355-356`, pass the chassis number explicitly when
   constructing `PassThrough`
6. In `powermode.cpp:157` (inside `setMasterOcc()` or its `addChassis()`
   replacement from Sub-Task 7), replace `path.back() - '0'` with
   `getChassisFromPath(path)`
7. In `occ_manager.cpp:99`, change the string passed to `createObjects()` from
   `OCC_NAME + std::to_string(id)` to `"chassis" + std::to_string(id) + "/occ0"`
   so the full path constructed at line 328 becomes
   `OCC_CONTROL_ROOT/chassis<N>/occ0`
8. In `occ_object.hpp`, add a public
   `const fs::path& getPath() const { return path; }` getter
9. In `occ_poll_app_handler.cpp:38-39`, update `OccCommand` path initialization
   to use `statusObject.getPath().c_str()` instead of reconstructing any
   hardcoded path formatting

**Relevant Context:**

- [`occ_object.hpp`](occ_object.hpp) — `getInstance()` and `getDbusPath()`
  (after Sub-Task 3 rename)
- [`occ_manager.cpp:99`](occ_manager.cpp:99) — string passed to
  `createObjects()`
- [`occ_manager.cpp:328`](occ_manager.cpp:328) — full path construction
- [`occ_manager.cpp:355-356`](occ_manager.cpp:355) — `PassThrough` construction
- [`occ_command.cpp:56`](occ_command.cpp:56) — `path.back() - '0'` for device
  path
- [`occ_pass_through.hpp:42-44`](occ_pass_through.hpp:42) — `PassThrough`
  constructor signature
- [`occ_pass_through.cpp:24-38`](occ_pass_through.cpp:24) — path derivation
- [`powermode.cpp:157`](powermode.cpp:157) — `path.back() - '0'` for
  `occInstance`
- [`occ_poll_app_handler.cpp:38-39`](occ_poll_app_handler.cpp:38) — path
  construction
- [`utils.hpp`](utils.hpp) — home for the new `getChassisFromPath()` helper

---

### Sub-Task 9 — Add root system-info D-Bus object

**Status:** `[ ] pending`

**Intent:** Expose a system-wide summary object at the OCC control root path
(`/org/open_power/control`) so that callers can query overall OCC system state
without knowing chassis numbers or enumerating per-chassis paths. This is the
discovery entry point for the multi-chassis tree.

The object exposes four pieces of information:

- **ActiveChassisCount** — number of chassis whose OCC is currently active
- **TotalChassisCount** — total number of chassis configured (`MAX_CHASSIS`)
- **SystemSafeMode** — true only when all present chassis are currently in safe
  mode
- **ChassisPaths** — array of D-Bus object paths, one per known chassis (e.g.,
  `["/org/open_power/control/chassis1", "/org/open_power/control/chassis2", ...]`)

Because this project consumes interfaces from `phosphor-dbus-interfaces` as a
dependency rather than defining them locally, a new interface YAML must be
contributed to `phosphor-dbus-interfaces` (or the interface can be defined
inline using raw sdbusplus property registration as a stopgap). The plan uses
the inline approach as the initial implementation, noting that a proper
interface definition is the long-term target.

`Manager` owns this object and updates it whenever chassis active state or safe
mode changes.

**Expected Outcomes:**

- A `SystemInfo` class (in new files `occ_system_info.hpp`) wraps a sdbusplus
  object at `OCC_CONTROL_ROOT` and exposes the four properties above
- `Manager` holds a `std::unique_ptr<SystemInfo> systemInfo` member
- `Manager` creates the `SystemInfo` object during `findAndCreateObjects()`
  after the `pmode` object is created
- `Manager::statusCallBack()` calls `systemInfo->update()` whenever a chassis's
  active state changes
- `Manager::updateOccSafeMode()` calls `systemInfo->update()` whenever safe mode
  changes
- The `ChassisPaths` array is populated at construction time and does not change
  at runtime

**Todo List:**

1. Create `occ_system_info.hpp` defining a `SystemInfo` class that:
   - Registers a sdbusplus object at `OCC_CONTROL_ROOT`
   - Exposes `ActiveChassisCount` (uint8), `TotalChassisCount` (uint8),
     `SystemSafeMode` (bool), and `ChassisPaths` (array of object_path) as D-Bus
     properties
   - Provides an `update(uint8_t activeCount, bool allSafeMode)` method that
     refreshes the live properties
2. In `occ_manager.hpp`, add `std::unique_ptr<SystemInfo> systemInfo` member
3. In `Manager::findAndCreateObjects()`, construct `systemInfo` once (after
   `pmode` is constructed), passing `MAX_CHASSIS` and the list of chassis root
   paths
4. In `Manager::statusCallBack()`, after updating `chassisActive`, call
   `systemInfo->update(activeCount, anySafeMode)`
5. In `Manager::updateOccSafeMode()`, call `systemInfo->update(...)` after
   updating throttle state
6. Add `occ_system_info.hpp` to the build in `meson.build`

**Relevant Context:**

- [`app.cpp:40`](app.cpp:40) — `objManager` at `OCC_CONTROL_ROOT` (ObjectManager
  already registered)
- [`occ_manager.hpp:44-90`](occ_manager.hpp:44) — `Manager` constructor and
  members
- [`occ_manager.cpp:52-153`](occ_manager.cpp:52) — `findAndCreateObjects()`
- [`occ_manager.cpp:410-547`](occ_manager.cpp:410) — `statusCallBack()`
- [`occ_manager.cpp:639-647`](occ_manager.cpp:639) — `updateOccSafeMode()`
- [`powermode.hpp:29`](powermode.hpp:29) — example of sdbusplus
  `server::object_t` pattern

---

### Sub-Task 10 — Per-chassis reset isolation

**Status:** `[ ] pending`

**Intent:** Currently a reset request stops communication with **all** OCCs
before issuing an HRESET. With per-chassis independence, a reset of chassis N
must only affect that chassis's OCCs; other chassis continue running.

To keep the architecture clean and decoupled, the **chassis itself
(`ChassisObject`) is responsible for checking and initiating its own resets**.
Rather than the global `Manager` inspecting reset flags and controlling when
polling occurs, the `Manager` simply notifies each chassis to poll (e.g.
`chassisObj->poll()`). The `ChassisObject` then decides how to proceed:

- If a reset is required (`resetRequired == true` and not yet in progress), the
  `ChassisObject` initiates the reset of its chassis via
  `pldmHandle->resetOCC(chassis)`, sets `resetInProgress = true`, deactivates
  its own OCCs, and restarts its own `waitForAllOccsTimer`.
- If a reset is already in progress (`resetInProgress == true`), the
  `ChassisObject` skips polling.
- Otherwise, the `ChassisObject` performs polling on its active OCCs.

**Expected Outcomes:**

- `bool resetRequired` and `bool resetInProgress` removed from `Manager`;
  per-chassis equivalents live in `ChassisObject` (defined in Sub-Task 4)
- `resetOccRequest(chassisID)` delegates directly to
  `chassisObjects[chassis]->resetOccRequest()`, which sets its
  `resetRequired = true`
- The manager's global `pollerTimerExpired` calls `chassisObj->poll()` for each
  discovered chassis
- `ChassisObject::poll()` performs the reset check: if `resetRequired` is set,
  it deactivates its own OCCs and calls `pldmHandle->resetOCC(chassis)` to
  initiate the per-chassis reset (no global early return in the poller; other
  chassis keep polling)
- `waitForAllOccsTimer` is moved entirely into `ChassisObject` so each chassis
  independently tracks and handles its own post-reset timeout
- Global poll timer stops only when all chassis have `active == false`

**Todo List:**

1. Remove `bool resetRequired`, `bool resetInProgress`, `uint8_t resetInstance`
   from `occ_manager.hpp` (they now live in `ChassisObject` from Sub-Task 4)
2. In `ChassisObject`, implement `poll()` that:
   - Checks if `resetRequired && !resetInProgress`. If so, calls
     `initiateReset()`
   - If `resetInProgress`, skips polling (but sets sensor values to NaN if
     inactive)
   - Otherwise, invokes `PollHandler()` on each of its active OCCs
3. Implement `ChassisObject::initiateReset()` to set `resetInProgress = true`,
   set `resetRequired = false`, call `setOccsActive(false)` to deactivate that
   chassis's OCCs, trigger `pldmHandle->resetOCC(chassis)`, and start its
   `waitForAllOccsTimer`
4. Update `resetOccRequest(chassisID)` in `Manager` to delegate to
   `chassisObjects[chassis]->resetOccRequest()`
5. Refactor `Manager::pollerTimerExpired()` to loop over `chassisObjects` and
   call `chassisObj->poll()`, removing the global `resetRequired` early return
6. Update `statusCallBack` to stop the poll timer only when all chassis have
   `active == false` (check `chassisObjects` map, not a scalar
   `activeCount == 0`)
7. Move `waitForAllOccsTimer` into `ChassisObject` so each chassis independently
   tracks when its OCCs should have returned to active after a reset, and
   handles its own expiration callback

**Relevant Context:**

- [`occ_manager.cpp:361-408`](occ_manager.cpp:361) — `resetOccRequest` and
  `initiateOccRequest`
- [`occ_manager.hpp:225-231`](occ_manager.hpp:225) — reset flags to remove
- [`occ_manager.cpp:410-547`](occ_manager.cpp:410) — `statusCallBack` reset
  logic
- [`occ_chassis.hpp`](occ_chassis.hpp) — `ChassisObject` members from Sub-Task 4
