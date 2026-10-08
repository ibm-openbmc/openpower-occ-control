# OCC Host Activation and Deactivation

This document describes what happens when the host starts up and shuts down with
respect to OCC activation and deactivation.

**Pre-condition:** `Manager` has already created `OccObject` and `ChassisObject`
instances for all discovered OCCs (e.g. OCC0 on chassis 1, OCC1 on chassis 2).

---

## Activation Sequence

When the host starts, HTMGT sets the PLDM state sensor for each OCC to
`IN_SERVICE`. The `pldm::Interface` receives the sensor event and calls
`Manager::updateOCCActive()` for each OCC.

```mermaid
sequenceDiagram
    participant HTMGT
    participant PLDM as pldm::Interface
    participant Mgr as Manager
    participant C1 as ChassisObject(chassis 1)
    participant O0 as OccObject(OCC0)
    participant C2 as ChassisObject(chassis 2)
    participant O1 as OccObject(OCC1)

    note over HTMGT,O1: HTMGT sets PLDM state sensor: OCC0 → IN_SERVICE

    HTMGT->>PLDM: SetStateSensorReadings(OCC0 sensor, IN_SERVICE)
    PLDM->>Mgr: updateOCCActive(inst=0, active=true)
    Mgr->>C1: setPldmSensorReceived(0, true)
    Mgr->>C1: setOccActive(0, true)
    C1->>O0: occActive(true) → occActivated()
    note over O0: updateThrottle(false, ALL)<br/>lastState = 0<br/>device.setActive(true)<br/>addErrorWatch()<br/>Base::Status::occActive(true)
    O0->>C1: chassisCallback(cid=1, true)<br/>→ onOccActiveChanged(true)
    note over C1: allOccsActive() → true<br/>chassis1.active = true
    C1->>Mgr: managerCallback(cid=1, true)<br/>→ chassisStatusCallback(1, true)
    note over Mgr: chassis1 active, chassis2 not yet<br/>waitForAllOccsTimer.restartOnce(60s)<br/>_pollTimer not running → pollerTimerExpired()<br/>poll timer started (5 s interval)
    Mgr-->>PLDM: return true

    note over HTMGT,O1: HTMGT sets PLDM state sensor: OCC1 → IN_SERVICE

    HTMGT->>PLDM: SetStateSensorReadings(OCC1 sensor, IN_SERVICE)
    PLDM->>Mgr: updateOCCActive(inst=1, active=true)
    Mgr->>C2: setPldmSensorReceived(1, true)
    Mgr->>C2: setOccActive(1, true)
    C2->>O1: occActive(true) → occActivated()
    note over O1: updateThrottle(false, ALL)<br/>lastState = 0<br/>device.setActive(true)<br/>addErrorWatch()<br/>Base::Status::occActive(true)
    O1->>C2: chassisCallback(cid=2, true)<br/>→ onOccActiveChanged(true)
    note over C2: allOccsActive() → true<br/>chassis2.active = true
    C2->>Mgr: managerCallback(cid=2, true)<br/>→ chassisStatusCallback(2, true)
    note over Mgr: ALL chassis active ✓<br/>waitForAllOccsTimer.setEnabled(false)<br/>validateOccMaster() per chassis<br/>(starts presence monitor, clears safe mode)
    Mgr-->>PLDM: return true
```

### Step-by-Step Description

#### OCC0 activation (chassis 1)

1. **HTMGT → `pldm::Interface`**: HTMGT sends a PLDM `SetStateSensorReadings`
   message over MCTP with `IN_SERVICE` for OCC0's sensor.

2. **`pldm::Interface` → `Manager::updateOCCActive(0, true)`**: The PLDM
   interface fires the `occActiveCallBack` registered at construction time.

3. **Manager → `ChassisObject::setPldmSensorReceived(0, true)`**: Records that
   the PLDM sensor state has been received for OCC0. This flag is used by
   `isSensorResolved()` to avoid re-querying PLDM once the state has already
   arrived.

4. **Manager → `ChassisObject::setOccActive(0, true)`**: Delegates to
   `OccObject::occActive(true)` → `occActivated()`:
   - Clears any prior throttle reason (`updateThrottle(false, THROTTLED_ALL)`)
   - Resets `lastState = 0`
   - Binds the kernel device (`device.setActive(true)`)
   - Starts error/throttle watching (`addErrorWatch()`)
   - Sets the D-Bus `occActive` property to `true`

5. **`OccObject` → `ChassisObject::onOccActiveChanged(true)`**: OccObject fires
   `chassisCallback`. Since this is the only OCC on chassis 1, `allOccsActive()`
   returns `true` and `chassis1.active` is set.

6. **`ChassisObject` → `Manager::chassisStatusCallback(1, true)`**:
   - chassis 1 is active but chassis 2 is not yet — starts `waitForAllOccsTimer`
     (60 s)
   - Poll timer is not yet running → calls `pollerTimerExpired()` which starts
     the 5 s poll cycle

#### OCC1 activation (chassis 2)

Steps 7–12 mirror OCC0's flow through chassis 2.

On `chassisStatusCallback(2, true)` the Manager sees **all** chassis are now
active:

- Stops `waitForAllOccsTimer`
- Calls `validateOccMaster()` for each chassis, which starts presence monitoring
  and clears safe mode (`pmode->updateDbusSafeMode(false)`)

---

## Deactivation Sequences

### Path A — Normal shutdown (BMC-driven)

The BMC initiates a power-off. The host state D-Bus property transitions to
`Off`. `pldm::Interface` receives the `PropertiesChanged` signal, calls
`clearData()` to wipe the PDR cache, then calls `poweredOffCallBack()` which
invokes `Manager::hostPoweredOff()`. The Manager is the single point that drives
OCC deactivation via `deactivateAll()` on each chassis.

```mermaid
sequenceDiagram
    participant BMC as BMC (host state)
    participant PLDM as pldm::Interface
    participant Mgr as Manager
    participant C1 as ChassisObject(chassis 1)
    participant O0 as OccObject(OCC0)
    participant C2 as ChassisObject(chassis 2)
    participant O1 as OccObject(OCC1)

    note over BMC,O1: Host state transitions to Off (BMC-initiated power-off)

    BMC->>PLDM: PropertiesChanged(CurrentHostState = HostState.Off)
    PLDM->>PLDM: hostStateEvent() → clearData()
    note over PLDM: PDR cache cleared (sensorToOCCInstance etc.)<br/>no OCC callbacks here
    PLDM->>Mgr: poweredOffCallBack() → hostPoweredOff()
    note over Mgr: resetRequired/resetInProgress cleared<br/>chassisRequestingReset/instanceRequestingReset reset

    Mgr->>C1: deactivateAll()
    C1->>O0: occActive(false) → occDeactivated()
    note over O0: pldmSensorStateReceived=false<br/>setSensorValueToNaN()<br/>pmode->setMasterActive(false)<br/>removeErrorWatch()<br/>device.setActive(false)<br/>updateThrottle(false, ALL)
    O0->>C1: chassisCallback(cid=1, false) → onOccActiveChanged(false)
    note over C1: chassis1.active = false
    C1->>Mgr: managerCallback(cid=1, false) → chassisStatusCallback(1, false)
    note over Mgr: anyActive() still true (OCC1 not yet deactivated)<br/>_pollTimer left running

    Mgr->>C2: deactivateAll()
    C2->>O1: occActive(false) → occDeactivated()
    note over O1: (same teardown as OCC0)
    O1->>C2: chassisCallback(cid=2, false) → onOccActiveChanged(false)
    note over C2: chassis2.active = false
    C2->>Mgr: managerCallback(cid=2, false) → chassisStatusCallback(2, false)
    note over Mgr: anyActive() = false<br/>_pollTimer.setEnabled(false)<br/>waitForAllOccsTimer.setEnabled(false)
```

---

### Path B — OCC failure (host-side STOPPED or DORMANT sensor)

HTMGT sends a `SetStateSensorReadings` with `STOPPED` or `DORMANT` for an OCC,
indicating an OCC has failed. This is an error/recovery event, not a planned
shutdown.

- **`STOPPED`** — OCC has stopped; deactivates the OCC only.
- **`DORMANT`** — OCC has stopped and the system has entered **safe mode**;
  additionally fires `safeModeCallBack(true)` before deactivating.

The host state Off signal will arrive later and clean up the PDR cache via
`clearData()`, followed by `poweredOffCallBack()` → `hostPoweredOff()` →
`deactivateAll()` (which will be a no-op since the OCCs are already inactive).

```mermaid
sequenceDiagram
    participant HTMGT
    participant PLDM as pldm::Interface
    participant Mgr as Manager
    participant C1 as ChassisObject(chassis 1)
    participant O0 as OccObject(OCC0)

    note over HTMGT,O0: HTMGT sends STOPPED (or DORMANT) for OCC0

    HTMGT->>PLDM: SetStateSensorReadings(OCC0 sensor, STOPPED)
    note over PLDM: DORMANT path: safeModeCallBack(true) fires here<br/>before occActiveCallBack, setting safe mode on pmode<br/>and throttling all processors
    PLDM->>Mgr: occActiveCallBack(inst=0, false) → updateOCCActive(0, false)
    Mgr->>C1: setPldmSensorReceived(0, false)
    Mgr->>C1: setOccActive(0, false)
    C1->>O0: occActive(false) → occDeactivated()
    note over O0: full teardown (same as Path A)
    O0->>C1: chassisCallback(cid=1, false) → onOccActiveChanged(false)
    note over C1: chassis1.active = false
    C1->>Mgr: managerCallback(cid=1, false) → chassisStatusCallback(1, false)
    note over Mgr: _pollTimer stops if no chassis remain active<br/>Host state Off signal will follow later →<br/>clearData() + poweredOffCallBack() → hostPoweredOff()<br/>deactivateAll() (no-op: OCCs already inactive)
```

---

### OccObject teardown detail (`occDeactivated`)

All deactivation paths reach `OccObject::occDeactivated()` which performs these
steps in order:

| Step | Action                                                                          |
| ---- | ------------------------------------------------------------------------------- |
| 1    | Clear `pldmSensorStateReceived`, `pcapBoundsUpdated`, `stateValid`              |
| 2    | If sensors were valid: `setSensorValueToNaN()` (marks D-Bus sensors NaN)        |
| 3    | If master OCC: `pmode->setMasterActive(false)` (blocks mode changes)            |
| 4    | Stop `safeStateDelayTimer` if running                                           |
| 5    | `removeErrorWatch()` — stops throttle/error file descriptor watching            |
| 6    | `device.setActive(false)` — unbinds the kernel OCC device                       |
| 7    | Fire `chassisCallback(cid, false)` → `ChassisObject::onOccActiveChanged(false)` |
| 8    | `updateThrottle(false, THROTTLED_ALL)` — clears processor throttle D-Bus state  |
| 9    | `MyPollHandler->clearOccPollTraceFlags()`                                       |

---

## Key Invariants

| Condition                                                     | Behaviour                                                                                                                                           |
| ------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| Host not yet running when `IN_SERVICE` arrives                | `updateOCCActive` clears `pldmSensorReceived`, skips `setOccActive`, restarts `discoverTimer` (30 s)                                                |
| Only one chassis active after both sensors arrive             | `waitForAllOccsTimer` fires after 60 s; `occsNotAllRunning()` calls `validateOccMaster()` anyway (guards may mean fewer OCCs than expected)         |
| `pldmSensorStateReceived` set by HRESET or queued-state paths | It is **not** — those paths call `setOccActive` directly and do not touch the flag, preserving its meaning as "PLDM has delivered a sensor reading" |
| Normal shutdown deactivation                                  | `clearData()` only wipes the PDR cache; `hostPoweredOff()` → `deactivateAll()` is the single point of OCC deactivation                              |
