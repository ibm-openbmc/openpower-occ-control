# Multi-Node OCC Support Plan

## Overview

Add support for up to 8 nodes (expandable to 12). A single BMC manages all nodes. Initially one
OCC per node is supported, with the architecture designed to allow multiple OCCs per node in the
future. The OCC in each node is the master for that node.

Key constraints driving the design:

- `/dev/occ0` → node 0, `/dev/occ1` → node 1, … `/dev/occN` → node N (direct mapping, no +1 offset)
- sysfs device is `occ-hwmon.0` for node 0 (no +1 offset — matches `/dev/occ` numbering)
- Initially one OCC per node; that OCC is always the master for its node. The D-Bus path structure
  (`node<N>/occ0`) and data structures are designed to accommodate multiple OCCs per node later
- All system-wide data (power mode, IPS, power cap) is broadcast to **every active node's master OCC**
- IPS is disabled when there is more than one node; it is only allowed on a single-node system
- Safe mode is **per-node** — when node N enters safe mode, other nodes continue operating normally
- PLDM safe mode event does **not** yet carry a node identifier; it must be extended
- Maximum 8 nodes (future: 12); this is a compile-time constant (`MAX_NODES`)

### Polling Model

This plan defaults to **app polling** (`ENABLE_APP_POLL_SUPPORT`) but preserves kernel polling
support (`OccPollKernelHandler`). The existing `#ifdef ENABLE_APP_POLL_SUPPORT` guards in
`occ_status.hpp` (later `occ_object.hpp`) remain in place — both code paths must continue to
compile and function correctly. The kernel poll handler files
(`occ_poll_kernel_handler.hpp/cpp`) are left untouched.

The sole app-poll path construction in `occ_poll_app_handler.cpp:38-39` builds an `OccCommand`
using the old `OCC_CONTROL_ROOT / occ<N>` path — this must be updated as part of Sub-Task 7.

### What Is Not Changing

- D-Bus paths for power mode / IPS / power cap (`/xyz/openbmc_project/control/host0/...`) remain
  unchanged — these are system-wide objects
- The `PassThrough` interface, PLDM active-OCC handling, and error/FFDC handling logic are preserved
- The existing `OccCommand`, `Device`, and `OccPollHandler` abstract interface are not being
  redesigned — only their instantiation and per-node dispatch changes
- `OccPollKernelHandler` is not modified

---

## Target Class Organization

```
BMC
└── Manager  (occ_manager.hpp)
    │
    ├── PowerMode  (powermode.hpp)
    │   ├── occCmds[0..MAX_NODES]  OccCommand  ← sends SET_MODE to each node master
    │   └── nodeActive[0..MAX_NODES]
    │
    ├── PowerCap  (powercap.hpp)
    │   └── nodeOccObjs[0..N]  OccObject*  ← writes power_cap_user hwmon per node
    │
    ├── SystemInfo  (occ_system_info.hpp)
    │   └── D-Bus object at /org/open_power/control
    │       ActiveNodeCount, TotalNodeCount, SystemSafeMode, NodePaths
    │
    ├── pldmHandle  pldm::Interface  (pldm.hpp)
    │   └── safeModeCallBack(nodeID, bool)  ← extended for per-node safe mode
    │
    └── nodeObjects  map[nodeID -> NodeObject]  (occ_node.hpp)
        │
        └── NodeObject  (one per node, up to MAX_NODES)
            │   node          nodeID
            │   active        bool
            │   safeMode      bool
            │   resetRequired bool
            │   resetInProgress bool
            │   waitForAllOccsTimer
            │
            ├── occObjects[0..N]  OccObject  (occ_object.hpp, renamed from occ_status)
            │   │   instance       nodeID
            │   │   occCmd         OccCommand   ← /dev/occN
            │   │   device         Device       ← occ-hwmon.N sysfs
            │   │   occPollObj     OccPollAppHandler
            │   │   throttleCause  uint8_t
            │   │   safeStateDelayTimer
            │   └── (hwmonPath, lastState, sensor validity flags, ...)
            │
            └── passThroughObjects[0..N]  PassThrough
                    node       nodeID   ← new explicit field
                    occCmd     OccCommand
                    devicePath /dev/occN
```

**D-Bus tree (after Sub-Task 8):**
```
/org/open_power/control                     ← SystemInfo root object
    .ActiveNodeCount  .TotalNodeCount
    .SystemSafeMode   .NodePaths[]

/org/open_power/control/node0/occ0          ← OccObject D-Bus presence + PassThrough
/org/open_power/control/node1/occ0
...
/org/open_power/control/nodeN/occ0

/xyz/openbmc_project/control/host0/power_mode      ← PowerMode  (unchanged)
/xyz/openbmc_project/control/host0/power_ips       ← IPS        (single-node only)
/xyz/openbmc_project/control/host0/power_cap_limits ← PowerCap  (unchanged)
```

---

## Sub-Tasks

---

### Sub-Task 1 — Introduce `nodeID` type and `MAX_NODES` constant

**Status:** `[ ] pending`

**Intent:**
Establish the foundational types and constants so that all subsequent sub-tasks can use them
consistently throughout the codebase.

**Expected Outcomes:**
- A `nodeID` type alias (`uint8_t`) is defined in `utils.hpp` under namespace `open_power::occ` to avoid cyclic header dependencies.
- A `MAX_NODES` compile-time constant (default 8) is added in `meson.options` and `meson.build`,
  generating `MAX_NODES` in `config.h` (similar to `MAX_CPUS`)
- Existing code compiles without change — these are purely additive

**Todo List:**
1. Add `option('max-nodes', type: 'integer', min: 1, max: 12, value: 8)` to `meson.options`
2. Wire `MAX_NODES` into `meson.build` the same way `MAX_CPUS` is handled so it appears in `config.h`
3. Add `using nodeID = uint8_t;` inside the namespace `open_power::occ` in `utils.hpp`.

**Relevant Context:**
- [`meson.options`](meson.options) — existing `max-cpus` option for reference
- [`meson.build`](meson.build) — existing `MAX_CPUS` config generation for reference
- [`utils.hpp`](utils.hpp) — location for the new `nodeID` type definition

---

### Sub-Task 2 — Remove all `/dev/occX` and sysfs `+1` offsets

**Status:** `[ ] pending`

**Intent:**
Under the new device scheme, `/dev/occN` is node N — a direct zero-based mapping. The code
currently has two inconsistencies that must be fixed:

1. `PassThrough` adds `+1` when building its `/dev/occX` path (node 0 would incorrectly open
   `/dev/occ1`).
2. `OccObject` (currently `Status`) uses `instance + 1` when naming the sysfs hwmon device (`occ-hwmon.1` for node 0);
   this must become `occ-hwmon.0`.

`OccCommand` already uses `path.back() - '0'` without any offset, so it is already correct.

**Expected Outcomes:**
- `OccCommand::devicePath` → `/dev/occ<N>` (no change needed; verify and document)
- `PassThrough::devicePath` → `/dev/occ<N>` with no `+1`
- `OccObject` device sysfs path → `occ-hwmon.<N>` with no `+1`

**Todo List:**
1. In `occ_pass_through.cpp:28`, remove the `+1` from the device path construction
2. In `occ_status.hpp:101` (renamed to `occ_object.hpp` in Sub-Task 3), change
   `std::to_string(instance + 1)` to `std::to_string(instance)`
3. Confirm `occ_command.cpp:56` uses `path.back() - '0'` directly (no +1); add a comment
   documenting the node-to-device mapping

**Relevant Context:**
- [`occ_command.cpp:54-65`](occ_command.cpp:54) — `OccCommand` constructor, device path (already correct)
- [`occ_status.hpp:99-102`](occ_status.hpp:99) — `device(...)` init, sysfs path `occ-hwmon.N+1` (→ `occ_object.hpp` after Sub-Task 3)
- [`occ_pass_through.cpp`](occ_pass_through.cpp) — PassThrough devicePath with `+1`

---

### Sub-Task 3 — Rename `Status` → `OccObject` and `occ_status` → `occ_object`

**Status:** `[ ] pending`

**Intent:**
Rename the `Status` class to `OccObject` (parallel to `NodeObject` in the new hierarchy) and
rename the source files from `occ_status.hpp/cpp` to `occ_object.hpp/cpp`. This is a purely
mechanical rename with no behavioral change — doing it as its own sub-task keeps the diff
easy to review before the structural changes in Sub-Task 4 begin.

The existing `instance` field already carries its identity. `OccObject` does not need to know
which node it is associated with. 

**Expected Outcomes:**
- `Status` class renamed to `OccObject` everywhere in the codebase
- Source files renamed: `occ_status.hpp` → `occ_object.hpp`, `occ_status.cpp` → `occ_object.cpp`
- All `#include "occ_status.hpp"` → `#include "occ_object.hpp"` throughout
- `meson.build` updated to compile `occ_object.cpp` instead of `occ_status.cpp`
- No behavioral change — compile and run identical to before

**Todo List:**
1. Rename `occ_status.hpp` to `occ_object.hpp` and `occ_status.cpp` to `occ_object.cpp`
2. Rename the `Status` class to `OccObject` within those files; update the include guards
3. Update all `#include "occ_status.hpp"` references in every file that includes it
4. Update `meson.build` to build `occ_object.cpp` instead of `occ_status.cpp`
5. Global search for any remaining `Status` references that refer to the OCC status class
   (not the D-Bus `Base::Status` interface typedef) and rename them to `OccObject`

**Relevant Context:**
- [`occ_status.hpp`](occ_status.hpp) — class definition to rename → `occ_object.hpp`
- [`occ_status.cpp`](occ_status.cpp) — implementation to rename → `occ_object.cpp`
- [`occ_status.hpp:91-126`](occ_status.hpp:91) — constructor and instance derivation
- [`occ_manager.hpp`](occ_manager.hpp) — includes `occ_status.hpp`, uses `Status` type
- [`occ_manager.cpp`](occ_manager.cpp) — uses `Status` in `createObjects()`, `statusCallBack()`
- [`occ_manager.cpp:574`](occ_manager.cpp:574) — `find_if` by `getOccInstanceID()`
- [`occ_device.hpp`](occ_device.hpp) — forward-declares or includes `Status`
- [`meson.build`](meson.build) — lists source files

---

### Sub-Task 4 — Introduce `NodeObject` class

**Status:** `[ ] pending`

**Intent:**
Define the `NodeObject` class that will own all per-node state and the collection of OCC
objects for that node. This is a purely additive step — the new file is created and wired
into the build, but `Manager` is not yet changed. Sub-Task 5 then migrates `Manager` to
use it.

**What `NodeObject` owns:**
- `std::vector<std::unique_ptr<OccObject>> occObjects` — the OCCs on this node (one today,
  multiple later); replaces and renames `statusObjects`
- `std::vector<std::unique_ptr<PassThrough>> passThroughObjects` — one per OCC on this node
- `nodeID node` — this node's identifier
- `bool active` — true when this node's OCCs are active
- `bool safeMode` — true when this node is in safe mode; used by Sub-Task 6
- `bool resetRequired` / `bool resetInProgress` — per-node reset state; used by Sub-Task 10
- Reference to the master `OccObject` for this node (the first/only OCC today), exposed
  for `PowerMode` and `PowerCap` to use

**What stays in `OccObject` (per-OCC, renamed from `Status`):**
- `instance`, `throttleCause`, `safeStateDelayTimer`, `occCmd`, `device`, `hwmonPath`,
  `lastState`, sensor validity flags — all truly per-OCC, not per-node

**Expected Outcomes:**
- `occ_node.hpp` exists and compiles cleanly
- `NodeObject` is listed in `meson.build`
- No existing code is changed — `Manager` still uses `statusObjects` until Sub-Task 5

**Todo List:**
1. Create `occ_node.hpp` defining `NodeObject` with the members listed above; `Manager` and
   `OccObject` are forward-declared as needed
2. Add `occ_node.hpp` to the build in `meson.build`

**Relevant Context:**
- [`occ_object.hpp`](occ_object.hpp) — `OccObject` class (renamed from `Status` in Sub-Task 3)
- [`occ_manager.hpp:176-212`](occ_manager.hpp:176) — data members that Sub-Task 5 will restructure

---

### Sub-Task 5 — Update `Manager` to use `NodeObject`

**Status:** `[ ] pending`

**Intent:**
Migrate `Manager` from its current flat `statusObjects` / `passThroughObjects` vectors to a
`std::map<nodeID, std::unique_ptr<NodeObject>> nodeObjects` container, using the `NodeObject`
class introduced in Sub-Task 4. The rename of `Status` → `OccObject` is already done by
Sub-Task 3; this sub-task uses `OccObject` throughout.

The existing `statusObjects` vector is renamed to `occObjects` throughout — both as the
member name in `NodeObject` and across all source files that reference it.

**What stays in `Manager`:**
- `std::map<nodeID, std::unique_ptr<NodeObject>> nodeObjects` — the node-keyed container
- `uint8_t activeCount` — system-wide count of active OCCs (sum across all nodes)
- `uint8_t numNodes` — how many nodes were discovered (replaces `statusObjects.size()`)
- System-wide timers, ambient/altitude, PLDM handle, power mode, power cap

**Expected Outcomes:**
- Node N's master `OccObject` is accessed via `nodeObjects[n]->getMasterOcc()`; `Manager`
  never indexes `occObjects` directly
- `nodeObjects[n]->passThroughObjects[0]` similarly
- `statusObjects` no longer exists anywhere in the codebase; all per-OCC operations on a node
  are delegated to member functions of `NodeObject` (`Status` → `OccObject` already done in Sub-Task 3)
- Per-node state (`active`, `safeMode`, `resetRequired`, `resetInProgress`) lives in
  `NodeObject`, not in parallel maps/arrays in `Manager`
- All `find_if` searches replaced with `nodeObjects[instance]->getMasterOcc()`
- All loops over OCCs become `for (auto& [node, nodeObj] : nodeObjects)` with `Manager`
  delegating per-OCC work via member functions on `NodeObject` rather than directly
  iterating `occObjects`

**Todo List:**
1. Add `std::map<nodeID, std::unique_ptr<NodeObject>> nodeObjects` to `occ_manager.hpp`;
   remove `statusObjects` and `passThroughObjects` members
2. Add `uint8_t numNodes = 0` to `occ_manager.hpp`
3. Rename all remaining references to `statusObjects` in `occ_manager.cpp` to the appropriate
   `NodeObject` member function call (search-replace starting point, then adjust per-access
   semantics)
4. Update `createObjects(occ)` to extract node number, construct or retrieve `nodeObjects[node]`,
   then call `nodeObjects[node]->addOcc(...)` and `nodeObjects[node]->addPassThrough(...)`
   to append the new objects, and increment `numNodes`
5. Replace all `statusObjects.size()` comparisons with `numNodes`
6. Replace all `find_if` calls (lines 551, 574, 660) with
   `nodeObjects[instance]->getMasterOcc()`
7. Update `statusCallBack(instance, status)` to set `nodeObjects[instance]->active = status`
   alongside the existing `activeCount` update
8. Update all loops in `Manager` to iterate `nodeObjects` and delegate per-OCC work via a
   method on `NodeObject` (e.g. `nodeObj->forEachOcc(...)`) rather than directly accessing
   `occObjects` from `Manager`
9. Move `validateOccMaster()` into `NodeObject` as a per-node method; the master-detection
   logic is retained so it can correctly handle multiple OCCs per node in the future —
   `Manager` calls `nodeObj->validateOccMaster()` for each node

**Relevant Context:**
- [`occ_node.hpp`](occ_node.hpp) — `NodeObject` class from Sub-Task 4
- [`occ_object.hpp`](occ_object.hpp) — `OccObject` class (renamed from `Status` in Sub-Task 3)
- [`occ_manager.hpp:176-212`](occ_manager.hpp:176) — data members to restructure
- [`occ_manager.cpp:326-357`](occ_manager.cpp:326) — `createObjects()` — uses `OccObject`
- [`occ_manager.cpp:410-547`](occ_manager.cpp:410) — `statusCallBack()` — uses `statusObjects`
- [`occ_manager.cpp:431`](occ_manager.cpp:431) — `activeCount == statusObjects.size()` → `numNodes`
- [`occ_manager.cpp:551`](occ_manager.cpp:551) — `find_if` in `sbeTimeout()`
- [`occ_manager.cpp:574`](occ_manager.cpp:574) — `find_if` in `updateOCCActive()`
- [`occ_manager.cpp:660`](occ_manager.cpp:660) — `find_if` in `sbeHRESETResult()`
- [`occ_manager.cpp:812-857`](occ_manager.cpp:812) — `pollerTimerExpired()`
- [`occ_manager.cpp:1093-1168`](occ_manager.cpp:1093) — `validateOccMaster()` to be moved into `NodeObject`

---

### Sub-Task 6 — Per-node safe mode isolation

**Status:** `[ ] pending`

**Intent:**
Currently `updateOccSafeMode(bool)` applies safe mode globally to ALL OCCs and updates the
single `SafeMode` D-Bus property. Under the new model, safe mode is per-node: only the OCC in
the affected node is throttled; other nodes remain unaffected.

The PLDM `safeModeCallBack` signature is `std::function<void(bool)>` — it does not carry a node
identifier. Both the PLDM interface and its caller in `Manager` must be extended to pass a
`nodeID`. In `pldm.cpp` the safe mode trigger occurs at the point where the active-OCC state
sensor reads DORMANT; the `instance` is already known at both call sites (line 186 and line 978),
so the node ID is available and simply needs to be forwarded.

The D-Bus `SafeMode` property on `PowerMode` remains system-wide: it is `true` only when all
present nodes are in safe mode, `false` when any node is operating normally.

**Expected Outcomes:**
- `pldm::Interface::safeModeCallBack` signature changes to `std::function<void(nodeID, bool)>`
- In `pldm.cpp`, both call sites pass `instance` as the node ID:
  `safeModeCallBack(instance, true)`
- `Manager::updateOccSafeMode(nodeID node, bool safeMode)` delegates to
  `nodeObjects[node]->setSafeMode(safeMode)` which sets the node's `safeMode` flag and
  throttles its OCCs
- System-wide `SafeMode` D-Bus property is true only when all active nodes are in safe mode
- A node going safe does not stop polling or affect OCCs in other nodes

**Todo List:**
1. In `pldm.hpp:81`, change `std::function<void(bool)> safeModeCallBack` to
   `std::function<void(nodeID, bool)> safeModeCallBack`
2. In `pldm.cpp:186` and `pldm.cpp:978`, pass `instance` as the first argument:
   `safeModeCallBack(instance, true)`
3. In `occ_manager.hpp:249`, update `updateOccSafeMode` signature to `(nodeID node, bool safeState)`
4. In `occ_manager.cpp:639-647`, update `updateOccSafeMode` body: call
   `nodeObjects[node]->setSafeMode(safeMode)` to set the flag and throttle that node's OCCs;
   then compute the system-wide aggregate (`all_of` over `nodeObjects` checking `->safeMode`)
   and call `pmode->updateDbusSafeMode(allSafe)`
5. Update the `createPldmHandle()` bind in `occ_manager.cpp:39` to match the new signature

**Relevant Context:**
- [`pldm.hpp:72-84`](pldm.hpp:72) — `safeModeCallBack` member and constructor parameter
- [`pldm.cpp:177-186`](pldm.cpp:177) — `sensorEvent` safe mode trigger (`instance` available)
- [`pldm.cpp:970-978`](pldm.cpp:970) — `pldmRspCallback` safe mode trigger (`instance` available)
- [`occ_manager.cpp:32-42`](occ_manager.cpp:32) — `createPldmHandle()` bind
- [`occ_manager.cpp:639-647`](occ_manager.cpp:639) — `updateOccSafeMode()`

---

### Sub-Task 7 — Broadcast power mode, IPS, and power cap to all active nodes

**Status:** `[ ] pending`

**Intent:**
All system-wide data commands must be sent to the master OCC in **every** active node.
`PowerMode` and `PowerCap` do not hold OCC references directly — instead they call an interface
on `NodeObject` to trigger the send or write, keeping OCC access encapsulated in `NodeObject`:

- **Power mode** — `PowerMode` calls `nodeObj->sendModeData(...)` for each active node;
  `NodeObject` forwards the command to its master OCC via `OccCommand`
- **IPS** — same delegation pattern; **disabled when more than one node is present** (IPS is
  only meaningful for single-node systems)
- **Power cap** — `PowerCap` calls `nodeObj->writePcap(value)` for each active node;
  `NodeObject` writes to the hwmon sysfs `power_cap_user` file of its master OCC

#### PowerMode changes

`PowerMode` currently holds a single `occCmd` (`OccCommand`) and single `masterActive`/
`masterOccSet` booleans. These are replaced with a reference to `Manager`'s `nodeObjects` map
so `sendModeChange()` and `sendIpsData()` can iterate active nodes and delegate to each
`NodeObject`.

IPS object creation is gated on the total **discovered/configured node count** (`numNodes` or `nodeObjects.size()`) being exactly 1. This prevents the D-Bus IPS object from "flickering" (creating and deleting) during boot if nodes come online sequentially. `sendIpsData()` is a no-op when more than one node is present in the system.

#### PowerCap changes

`PowerCap::writeOcc()` currently writes a single `power_cap_user` sysfs file directly. With
multiple nodes, `writeOcc()` iterates the active `nodeObjects` and calls
`nodeObj->writePcap(value)` on each, delegating the hwmon path lookup and file write to
`NodeObject`.

Defensive programming is added in `PowerCap::updatePcapBounds()` when querying node 0's bounds to ensure that node 0 exists in the map and contains a valid master OCC before attempting access.

**Expected Outcomes:**
- `PowerMode` no longer owns `occCmd` or per-node OCC references; `sendModeChange()` iterates
  active nodes and calls `nodeObj->sendModeData(...)` on each
- `sendModeChange()` skips nodes where `nodeObj->active` is false
- IPS object and `sendIpsData()` gated: only active when the total discovered node count is exactly 1 (prevents boot flickering)
- `PowerCap::writeOcc(pcapValue)` iterates active nodes and calls `nodeObj->writePcap(value)`
  on each; `NodeObject` handles the hwmon path and file write
- `PowerCap::updatePcapBounds()` safely checks if node 0 is present and has a valid master OCC, then calls `nodeObjects[0]->getPcapBounds()` to read system-wide bounds

**Todo List:**

*PowerMode:*
1. In `powermode.hpp`, remove `std::unique_ptr<OccCommand> occCmd`, `int occInstance`,
   `bool masterOccSet`, `bool masterActive`; add a reference to `Manager`'s `nodeObjects` map
2. Replace `setMasterOcc(const std::string&)` and `setMasterActive(bool)` with
   `setNodeActive(nodeID node, bool active)` that updates the node's active state
3. In `sendModeChange()`, replace the single `occCmd->send(...)` with a loop over active nodes
   calling `nodeObj->sendModeData(...)`; log per-node success/failure
4. In `sendIpsData()`, add an early-return guard: if the number of discovered/configured nodes is greater than 1,
   log and skip (IPS not supported in multi-node configuration)
5. Gate `createIpsObject()` / `removeIpsObject()` on single-node-only: when the total discovered node count is greater than 1, do not create the IPS object
6. Update the "master not active" guard in `sendModeChange()` to check that no node is active
7. In `occ_manager.cpp:statusCallBack`, call `pmode->setNodeActive(node, true/false)`

*NodeObject (mode/cap send):*
8. Add `sendModeData(...)` method to `NodeObject` that forwards the mode command to the master
   OCC via its `OccCommand`
9. Add `writePcap(value)` method to `NodeObject` that writes `power_cap_user` to the master
   OCC's hwmon sysfs path
10. Add `getPcapBounds()` method to `NodeObject` that reads the power cap bounds from the
    master OCC's hwmon sysfs path

*PowerCap:*
11. Remove `masterOccObj` from `powercap.hpp`; replace `writeOcc()` to iterate active nodes
    and call `nodeObj->writePcap(value)` on each
12. Replace `setMasterOccObj(OccObject&)` with `setNodeObjects(...)` that gives `PowerCap`
    access to the `nodeObjects` map
13. In `updatePcapBounds()`, defensively verify that `nodeObjects` contains key `0` and its master OCC is not null, then call `nodeObjects[0]->getPcapBounds()` to read system-wide bounds

*occ_object.cpp:*
14. Change `pmode->setMasterActive(false)` calls to `pmode->setNodeActive(node, false)` and
    `pmode->setMasterActive()` to `pmode->setNodeActive(node, true)`

**Relevant Context:**
- [`powermode.hpp:345-370`](powermode.hpp:345) — private members: `occCmd`, `occInstance`,
  `masterOccSet`, `masterActive`
- [`powermode.cpp:141-166`](powermode.cpp:141) — `setMasterOcc()`
- [`powermode.cpp:434-543`](powermode.cpp:434) — `sendModeChange()`
- [`powermode.cpp:708-790`](powermode.cpp:708) — `sendIpsData()`
- [`powermode.cpp:104-136`](powermode.cpp:104) — `createIpsObject()` / `removeIpsObject()`
- [`powercap.hpp:170-232`](powercap.hpp:170) — `setMasterOccObj`, `masterOccObj`, `writeOcc`
- [`powercap.cpp:257-294`](powercap.cpp:257) — `writeOcc()` — writes single hwmon file
- [`powercap.cpp:218-230`](powercap.cpp:218) — `getPcapFilename()` — reads from `masterOccObj`
- [`occ_object.cpp:93`](occ_object.cpp:93) and [`occ_object.cpp:172`](occ_object.cpp:172) — `setMasterActive` calls
- [`occ_manager.cpp:345-353`](occ_manager.cpp:345) — `createObjects` master detection
- [`occ_manager.cpp:1170-1187`](occ_manager.cpp:1170) — `updatePcapBounds()`

---

### Sub-Task 8 — Update D-Bus object paths to node/occ subtree

**Status:** `[ ] pending`

**Intent:**
OCC Status D-Bus objects are currently at `/org/open_power/control/occ<N>`. With multiple OCCs
per node anticipated in the future, adopt the subtree form now:
`/org/open_power/control/node<N>/occ0`. This cleanly separates the node identifier from the
per-node OCC index and makes the tree extensible without a future rename.

Since there is currently only one OCC per node, the OCC index within the node subtree is always
`occ0`. Node N maps directly to the existing instance number N.

The system-wide power mode, IPS, and power cap paths (`/xyz/openbmc_project/control/host0/...`)
are **not** changed.

The key challenge is that `getInstance()` and every other place that extracts a number from the
OCC D-Bus path currently uses `path.back() - '0'` — a single-character shortcut that reads the
last digit of the path. With the new format, the node number must be parsed from the `node<N>`
segment instead, and must handle multi-digit node numbers (e.g., `node10` for a future 12-node
system).

**Expected Outcomes:**
- OCC Status and PassThrough D-Bus paths: `/org/open_power/control/node<N>/occ0`
- A shared helper `getNodeFromPath(const std::string& path) -> nodeID` is introduced in `utils.hpp` that parses the `node<N>` segment with robustness: it searches for the `"node"` token, parses any multi-digit number, but falls back to `path.back() - '0'` if `"node"` is absent (ensuring complete backward compatibility with legacy test paths and transitions).
- `OccObject::getInstance(path)` delegates to `getNodeFromPath()`
- `OccObject::getDbusPath()` continues to work — it uses `getInstance()` to key the sensor map,
  and the sensor map is keyed on instance/node number so no map change is needed
- `OccCommand`, `PassThrough`, and `powermode.cpp` all use `getNodeFromPath()` for device and
  instance number derivation
- `OccObject` exposes a public `getPath()` method so other classes can retrieve its path directly
- `occ_poll_app_handler.cpp` gets the D-Bus path directly from the parent `statusObject.getPath().c_str()`, completely avoiding hardcoded path formatting and duplications
- `app.cpp` D-Bus object manager root (`OCC_CONTROL_ROOT`) is unchanged

**Todo List:**
1. In `utils.hpp`, add a robust `getNodeFromPath(const std::string& path) -> nodeID` free function (with a fallback to legacy `path.back() - '0'` extraction if the token `"node"` is absent; avoid regex)
2. In `occ_object.hpp`, replace `path.back() - '0'` in `getInstance()` with a call to
   `getNodeFromPath(path)`
3. In `occ_command.cpp:56`, replace `this->path.back() - '0'` with `getNodeFromPath(this->path)`
4. In `occ_pass_through.hpp/cpp`, add an explicit `nodeID node` constructor parameter so the
   node is stored directly rather than derived from the path. Update `occInstance` to be
   initialised from `node` rather than `path.back() - '0'`. Remove the `+1` from `devicePath`
   (per Sub-Task 2). This makes the interface forward-compatible: when multiple OCCs per node
   exist, `PassThrough` will already carry the node as a first-class field alongside the OCC
   index derived from the path
5. In `occ_manager.cpp:355-356`, pass the node number explicitly when constructing `PassThrough`
6. In `powermode.cpp:157` (inside `setMasterOcc()` or its `addNode()` replacement from
   Sub-Task 7), replace `path.back() - '0'` with `getNodeFromPath(path)`
7. In `occ_manager.cpp:99`, change the string passed to `createObjects()` from
   `OCC_NAME + std::to_string(id)` to `"node" + std::to_string(id) + "/occ0"` so the full
   path constructed at line 328 becomes `OCC_CONTROL_ROOT/node<N>/occ0`
8. In `occ_object.hpp`, add a public `const fs::path& getPath() const { return path; }` getter
9. In `occ_poll_app_handler.cpp:38-39`, update `OccCommand` path initialization to use `statusObject.getPath().c_str()` instead of reconstructing any hardcoded path formatting

**Relevant Context:**
- [`occ_object.hpp`](occ_object.hpp) — `getInstance()` and `getDbusPath()` (after Sub-Task 3 rename)
- [`occ_manager.cpp:99`](occ_manager.cpp:99) — string passed to `createObjects()`
- [`occ_manager.cpp:328`](occ_manager.cpp:328) — full path construction
- [`occ_manager.cpp:355-356`](occ_manager.cpp:355) — `PassThrough` construction
- [`occ_command.cpp:56`](occ_command.cpp:56) — `path.back() - '0'` for device path
- [`occ_pass_through.hpp:42-44`](occ_pass_through.hpp:42) — `PassThrough` constructor signature
- [`occ_pass_through.cpp:24-38`](occ_pass_through.cpp:24) — `path.back() - '0'` twice, `+1` offset
- [`powermode.cpp:157`](powermode.cpp:157) — `path.back() - '0'` for `occInstance`
- [`occ_poll_app_handler.cpp:38-39`](occ_poll_app_handler.cpp:38) — path construction
- [`utils.hpp`](utils.hpp) — home for the new `getNodeFromPath()` helper

---

### Sub-Task 9 — Add root system-info D-Bus object

**Status:** `[ ] pending`

**Intent:**
Expose a system-wide summary object at the OCC control root path
(`/org/open_power/control`) so that callers can query overall OCC system state without
knowing node numbers or enumerating per-node paths. This is the discovery entry point for
the multi-node tree.

The object exposes four pieces of information:
- **ActiveNodeCount** — number of nodes whose OCC is currently active
- **TotalNodeCount** — total number of nodes configured (`MAX_NODES`)
- **SystemSafeMode** — true only when all present nodes are currently in safe mode
- **NodePaths** — array of D-Bus object paths, one per known node
  (e.g., `["/org/open_power/control/node0", "/org/open_power/control/node1", ...]`)

Because this project consumes interfaces from `phosphor-dbus-interfaces` as a dependency rather
than defining them locally, a new interface YAML must be contributed to
`phosphor-dbus-interfaces` (or the interface can be defined inline using raw sdbusplus
property registration as a stopgap). The plan uses the inline approach as the initial
implementation, noting that a proper interface definition is the long-term target.

`Manager` owns this object and updates it whenever node active state or safe mode changes.

**Expected Outcomes:**
- A `SystemInfo` class (in new files `occ_system_info.hpp`) wraps a sdbusplus object at
  `OCC_CONTROL_ROOT` and exposes the four properties above
- `Manager` holds a `std::unique_ptr<SystemInfo> systemInfo` member
- `Manager` creates the `SystemInfo` object during `findAndCreateObjects()` after the
  `pmode` object is created
- `Manager::statusCallBack()` calls `systemInfo->update()` whenever a node's active state changes
- `Manager::updateOccSafeMode()` calls `systemInfo->update()` whenever safe mode changes
- The `NodePaths` array is populated at construction time and does not change at runtime

**Todo List:**
1. Create `occ_system_info.hpp` defining a `SystemInfo` class that:
   - Registers a sdbusplus object at `OCC_CONTROL_ROOT`
   - Exposes `ActiveNodeCount` (uint8), `TotalNodeCount` (uint8), `SystemSafeMode` (bool),
     and `NodePaths` (array of object_path) as D-Bus properties
   - Provides an `update(uint8_t activeCount, bool allSafeMode)` method that refreshes the
     live properties
2. In `occ_manager.hpp`, add `std::unique_ptr<SystemInfo> systemInfo` member
3. In `Manager::findAndCreateObjects()`, construct `systemInfo` once (after `pmode` is
   constructed), passing `MAX_NODES` and the list of node root paths
4. In `Manager::statusCallBack()`, after updating `nodeActive`, call
   `systemInfo->update(activeCount, anySafeMode)`
5. In `Manager::updateOccSafeMode()`, call `systemInfo->update(...)` after updating throttle
   state
6. Add `occ_system_info.hpp` to the build in `meson.build`

**Relevant Context:**
- [`app.cpp:40`](app.cpp:40) — `objManager` at `OCC_CONTROL_ROOT` (ObjectManager already registered)
- [`occ_manager.hpp:44-90`](occ_manager.hpp:44) — `Manager` constructor and members
- [`occ_manager.cpp:52-153`](occ_manager.cpp:52) — `findAndCreateObjects()`
- [`occ_manager.cpp:410-547`](occ_manager.cpp:410) — `statusCallBack()`
- [`occ_manager.cpp:639-647`](occ_manager.cpp:639) — `updateOccSafeMode()`
- [`powermode.hpp:29`](powermode.hpp:29) — example of sdbusplus `server::object_t` pattern

---

### Sub-Task 10 — Per-node reset isolation

**Status:** `[ ] pending`

**Intent:**
Currently a reset request stops communication with **all** OCCs before issuing an HRESET. With
per-node independence, a reset of node N must only affect that node's OCCs; other nodes continue
running.

To keep the architecture clean and decoupled, the **node itself (`NodeObject`) is responsible for checking and initiating its own resets**.
Rather than the global `Manager` inspecting reset flags and controlling when polling occurs, the `Manager` simply notifies each node to poll (e.g. `nodeObj->poll()`).
The `NodeObject` then decides how to proceed:
- If a reset is required (`resetRequired == true` and not yet in progress), the `NodeObject` initiates the reset of its node via `pldmHandle->resetOCC(node)`, sets `resetInProgress = true`, deactivates its own OCCs, and restarts its own `waitForAllOccsTimer`.
- If a reset is already in progress (`resetInProgress == true`), the `NodeObject` skips polling.
- Otherwise, the `NodeObject` performs polling on its active OCCs.

**Expected Outcomes:**
- `bool resetRequired` and `bool resetInProgress` removed from `Manager`; per-node equivalents
  live in `NodeObject` (defined in Sub-Task 4)
- `resetOccRequest(nodeID)` delegates directly to `nodeObjects[node]->resetOccRequest()`, which sets its `resetRequired = true`
- The manager's global `pollerTimerExpired` calls `nodeObj->poll()` for each discovered node
- `NodeObject::poll()` performs the reset check: if `resetRequired` is set, it deactivates its own OCCs and calls `pldmHandle->resetOCC(node)` to initiate the per-node reset (no global early return in the poller; other nodes keep polling)
- `waitForAllOccsTimer` is moved entirely into `NodeObject` so each node independently tracks and handles its own post-reset timeout
- Global poll timer stops only when all nodes have `active == false`

**Todo List:**
1. Remove `bool resetRequired`, `bool resetInProgress`, `uint8_t resetInstance` from
   `occ_manager.hpp` (they now live in `NodeObject` from Sub-Task 4)
2. In `NodeObject`, implement `poll()` that:
   - Checks if `resetRequired && !resetInProgress`. If so, calls `initiateReset()`
   - If `resetInProgress`, skips polling (but sets sensor values to NaN if inactive)
   - Otherwise, invokes `PollHandler()` on each of its active OCCs
3. Implement `NodeObject::initiateReset()` to set `resetInProgress = true`, set `resetRequired = false`, call `setOccsActive(false)` to deactivate that node's OCCs, trigger `pldmHandle->resetOCC(node)`, and start its `waitForAllOccsTimer`
4. Update `resetOccRequest(nodeID)` in `Manager` to delegate to `nodeObjects[node]->resetOccRequest()`
5. Refactor `Manager::pollerTimerExpired()` to loop over `nodeObjects` and call `nodeObj->poll()`, removing the global `resetRequired` early return
6. Update `statusCallBack` to stop the poll timer only when all nodes have `active == false`
   (check `nodeObjects` map, not a scalar `activeCount == 0`)
7. Move `waitForAllOccsTimer` into `NodeObject` so each node independently tracks when
   its OCCs should have returned to active after a reset, and handles its own expiration callback

**Relevant Context:**
- [`occ_manager.cpp:361-408`](occ_manager.cpp:361) — `resetOccRequest` and `initiateOccRequest`
- [`occ_manager.hpp:225-231`](occ_manager.hpp:225) — reset flags to remove
- [`occ_manager.cpp:410-547`](occ_manager.cpp:410) — `statusCallBack` reset logic
- [`occ_node.hpp`](occ_node.hpp) — `NodeObject` members from Sub-Task 4
