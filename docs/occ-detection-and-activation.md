# OCC Detection and Activation (2-Chassis Configuration)

## Overview

This document describes how OCCs (On-Chip Controllers) are discovered in `/dev`,
instantiated across multiple chassis, resolved via PLDM sensors, and
transitioned to the active state in `openpower-occ-control` for a **2-chassis**
system.

In a 2-chassis system:

- `/dev/occ0` maps to **Chassis 1** (`.../chassis1/occ0`, OCC instance 0).
- `/dev/occ1` maps to **Chassis 2** (`.../chassis2/occ0`, OCC instance 1).

---

## Timing and Sequence Diagram (2 Chassis)

```mermaid
sequenceDiagram
    autonumber
    participant Host as Host / Kernel
    participant PLDM as PLDM Stack
    participant Mgr as Manager
    participant C1 as ChassisObject (Chassis 1)
    participant O0 as OccObject 0 (Chassis 1)
    participant C2 as ChassisObject (Chassis 2)
    participant O1 as OccObject 1 (Chassis 2)
    participant Timers as Discovery / Wait / Poll Timers

    Note over Mgr,Timers: Phase 1: Device Discovery & Object Creation (2 Chassis)
    Mgr->>Timers: Start discoverTimer (10s)
    Timers-->>Mgr: discoverTimer expires -> findAndCreateObjects()
    Mgr->>Host: findOCCsInDev() reads /dev/occ*
    Host-->>Mgr: Returns [/dev/occ0, /dev/occ1]

    rect rgb(240, 248, 255)
    Note over Mgr,O0: Chassis 1 Creation (device 0 -> chassis 1)
    Mgr->>C1: Create ChassisObject(cid=1)
    Mgr->>O0: Create OccObject(inst=0, path=.../chassis1/occ0) & PassThrough
    C1->>C1: updateProcAssociations()
    end

    rect rgb(245, 245, 255)
    Note over Mgr,O1: Chassis 2 Creation (device 1 -> chassis 2)
    Mgr->>C2: Create ChassisObject(cid=2)
    Mgr->>O1: Create OccObject(inst=1, path=.../chassis2/occ0) & PassThrough
    C2->>C2: updateProcAssociations()
    end

    Note over Mgr,PLDM: Phase 2: Host Running & PLDM Sensor Resolution
    Mgr->>Mgr: checkAllActiveSensors()
    Mgr->>PLDM: checkActiveSensor(instance 0 & 1)
    PLDM->>Host: Query State Sensor PDRs / GetSensorReadings

    Note over Host,O0: Phase 3: Chassis 1 Goes Active (OCC 0)
    Host->>PLDM: PLDM StateSensorEvent (msgSensorOffset=OCCSensorOffset, eventState=STATUS_IN_SERVICE)
    PLDM->>PLDM: sensorEvent() matches OCCSensorOffset & maps sensorId -> instance 0
    PLDM->>Mgr: occActiveCallBack(instance=0, status=true)
    Mgr->>C1: setOccActive(instance=0, true)
    C1->>O0: occActive(true) -> occActivated()
    O0->>O0: updateThrottle(false) & reset lastState
    O0->>O0: Base::Status::occActive(true) (D-Bus property)
    O0->>C1: chassisCallback(chassis1, true)
    C1->>Mgr: onOccActiveChanged() -> chassisStatusCallback(chassis1, true)

    Note over Mgr,Timers: Phase 4: First Chassis Active — Timers & Polling Triggered
    Mgr->>Timers: 1st chassis active (1 of 2): start waitForAllOccsTimer (60s)
    Mgr->>Timers: Start _pollTimer & invoke pollerTimerExpired()
    loop Every pollInterval (e.g. 1s)
        Mgr->>C1: poll() -> O0: PollHandler() -> Send POLL to /dev/occ0
        Host-->>O0: POLL response -> Push Chassis 1 sensors to D-Bus
        Mgr->>C2: poll() (O1 not active -> setSensorValueToNaN())
    end

    Note over Host,O1: Phase 5: Chassis 2 Goes Active (OCC 1)
    Host->>PLDM: PLDM StateSensorEvent (msgSensorOffset=OCCSensorOffset, eventState=STATUS_IN_SERVICE)
    PLDM->>PLDM: sensorEvent() matches OCCSensorOffset & maps sensorId -> instance 1
    PLDM->>Mgr: occActiveCallBack(instance=1, status=true)
    Mgr->>C2: setOccActive(instance=1, true)
    C2->>O1: occActive(true) -> occActivated()
    O1->>O1: updateThrottle(false) & reset lastState
    O1->>O1: Base::Status::occActive(true) (D-Bus property)
    O1->>C2: chassisCallback(chassis2, true)
    C2->>Mgr: onOccActiveChanged() -> chassisStatusCallback(chassis2, true)

    Note over Mgr,Timers: Phase 6: All Chassis Active Completion
    Mgr->>Mgr: Check allActive() == true (Chassis 1 & Chassis 2 both active)
    Mgr->>Timers: Stop waitForAllOccsTimer (all chassis up before 60s timeout)
    Mgr->>Mgr: validateOccMaster()
    Mgr->>C1: Validate master OCC on Chassis 1 & start presence monitor
    Mgr->>C2: Validate master OCC on Chassis 2 & start presence monitor

    loop Every pollInterval (e.g. 1s) - Steady State Polling
        Mgr->>C1: poll() -> O0: PollHandler() -> Send POLL to /dev/occ0
        Host-->>O0: POLL response -> Push Chassis 1 sensors to D-Bus
        Mgr->>C2: poll() -> O1: PollHandler() -> Send POLL to /dev/occ1
        Host-->>O1: POLL response -> Push Chassis 2 sensors to D-Bus
    end
```

---

## Detailed Step-by-Step Lifecycle (2 Chassis)

### 1. Device Discovery & Object Creation

- **Scanning `/dev`**: `Manager::findOCCsInDev()` discovers both `/dev/occ0` and
  `/dev/occ1`.
- **Instantiation**:
  - **Chassis 1**: `Manager::createObjects(0)` instantiates `ChassisObject` for
    `chassis1` and `OccObject` for OCC instance 0 at D-Bus path
    `/xyz/openbmc_project/control/chassis1/occ0`.
  - **Chassis 2**: `Manager::createObjects(1)` instantiates `ChassisObject` for
    `chassis2` and `OccObject` for OCC instance 1 at D-Bus path
    `/xyz/openbmc_project/control/chassis2/occ0`.
  - `PassThrough` objects are registered for `/dev/occ0` and `/dev/occ1`.
  - Processor associations are updated via
    `ChassisObject::updateProcAssociations()` for each chassis.

### 2. Host Running & PLDM Sensor Resolution

- When the host reaches the running state (`utils::isHostRunning()`),
  `Manager::checkAllActiveSensors()` checks sensor resolution for both
  `chassis1` and `chassis2`.
- Queries PLDM (`pldm::Interface::checkActiveSensor()`) to obtain state sensor
  PDRs and current sensor states for both OCC 0 and OCC 1.

### 3. Chassis 1 (OCC 0) Activation

- Host sends a PLDM state sensor event for OCC 0.
- `pldm::Interface::sensorEvent(msg)` receives the signal on
  `/xyz/openbmc_project/pldm`:
  - Matches `msgSensorOffset == OCCSensorOffset`.
  - Resolves `sensorId` to OCC instance 0.
  - Identifies
    `eventState == PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_IN_SERVICE`
    (`STATUS_IN_SERVICE`).
  - Calls `occActiveCallBack(0, true)`.
- `Manager::updateOCCActive(0, true)` locates `chassis1` and calls
  `ChassisObject::setOccActive(0, true)`:
  - `OccObject::occActivated()` clears throttles
    (`updateThrottle(false, THROTTLED_ALL)`), resets `lastState`, and sets D-Bus
    property `occActive` to `true`.
  - `ChassisObject::onOccActiveChanged(true)` sets `chassis1` status to active
    and notifies `Manager::chassisStatusCallback(1, true)`.

### 4. First Chassis Active Handling & Polling

- Because `chassis1` is the first chassis to go active (`activeCount == 1` out
  of 2):
  - `waitForAllOccsTimer` is started with a 60-second timeout to allow
    `chassis2` time to come up.
  - Polling timer (`_pollTimer`) is started, kicking off periodic polling
    (`pollerTimerExpired()`).
  - In this intermediate state, `chassis1` polls `/dev/occ0` and pushes live
    sensor data to D-Bus, while `chassis2` sets its unactivated sensors to
    `NaN`.

### 5. Chassis 2 (OCC 1) Activation

- Host sends a PLDM state sensor event for OCC 1.
- `pldm::Interface::sensorEvent(msg)` verifies
  `msgSensorOffset == OCCSensorOffset`, maps `sensorId` to OCC instance 1, and
  verifies `STATUS_IN_SERVICE`.
- `Manager::updateOCCActive(1, true)` routes to `chassis2` and sets
  `OccObject 1` to active.
- `ChassisObject::onOccActiveChanged(true)` sets `chassis2` active and invokes
  `Manager::chassisStatusCallback(2, true)`.

### 6. Full 2-Chassis Active Completion

- `Manager::chassisStatusCallback()` checks `allActive()`:
  - Both `chassis1` and `chassis2` are now active.
  - `waitForAllOccsTimer` is stopped (cancelling the 60s timeout).
  - `Manager::validateOccMaster()` verifies the master OCC per chassis and
    starts OCC presence monitoring on each chassis.
  - Steady-state polling now continuously queries both `/dev/occ0` and
    `/dev/occ1` every poll cycle (`pollInterval`), parsing OCC responses and
    updating D-Bus sensors.
