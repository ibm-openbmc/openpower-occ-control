#include "config.h"

#include "occ_manager.hpp"

#include "occ_dbus.hpp"
#include "occ_errors.hpp"
#include "occ_ffdc.hpp"
#include "utils.hpp"

#include <nlohmann/json.hpp>
#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <regex>

namespace open_power
{
namespace occ
{

const auto HOST_ON_FILE = "/run/openbmc/host@0-on";
const std::string Manager::dumpFile = "/tmp/occ_control_dump.json";

using namespace phosphor::logging;
using namespace std::literals::chrono_literals;
using json = nlohmann::json;

void Manager::createPldmHandle()
{
    pldmHandle = std::make_unique<pldm::Interface>(
        std::bind(std::mem_fn(&Manager::updateOCCActive), this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(std::mem_fn(&Manager::sbeHRESETResult), this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(std::mem_fn(&Manager::updateOccSafeMode), this,
                  std::placeholders::_1),
        std::bind(std::mem_fn(&Manager::hostPoweredOff), this), event);
}

// findAndCreateObjects():
// Takes care of getting the required objects created and
// finds the available devices/processors.
// (function is called every time the discoverTimer expires)
// - create the PowerMode object to control OCC modes
// - create occObjects for each OCC device found
// - waits for OCC Active sensors PDRs to become available
// - restart discoverTimer if all data is not available yet
void Manager::findAndCreateObjects()
{
    if (!pmode)
    {
        // Create the power mode object
        pmode = std::make_unique<powermode::PowerMode>(
            powermode::PMODE_PATH, powermode::PIPS_PATH, event);
    }

    if (!fs::exists(HOST_ON_FILE))
    {
        static bool occObjCreated = false;
        if (!occObjCreated)
        {
            // Create the OCCs based on on the /dev/occX devices
            auto occDevices = findOCCsInDev();

            if (occDevices.empty() ||
                (prevOCCSearch.size() != occDevices.size()))
            {
                // Something changed or no OCCs yet, try again in 10s.
                // Note on the first pass prevOCCSearch will be empty,
                // so there will be at least one delay to give things
                // a chance to settle.
                static bool tracedOCCWait = false;
                if (!tracedOCCWait ||
                    (prevOCCSearch.size() != occDevices.size()))
                {
                    lg2::info(
                        "Manager::findAndCreateObjects(): Waiting for OCC devices in /dev (currently {QTY})",
                        "QTY", occDevices.size());
                    tracedOCCWait = true;
                }
                prevOCCSearch = occDevices;

                discoverTimer->restartOnce(10s);
            }
            else
            {
                // All OCCs appear to be available, create OCC objects

                // createObjects requires OCC0 first.
                std::sort(occDevices.begin(), occDevices.end());

                lg2::info(
                    "Manager::findAndCreateObjects(): Creating {QTY} occObjects",
                    "QTY", occDevices.size());
                for (auto id : occDevices)
                {
                    createObjects(id);
                }
                occObjCreated = true;
                waitingForAllOccActiveSensors = true;

                // Find/update the processor path associated with each OCC
                for (auto& [cid, chassisObj] : chassisObjects)
                {
                    chassisObj->updateProcAssociations();
                }
            }
        }

        if (occObjCreated && waitingForAllOccActiveSensors)
        {
            static bool tracedHostWait = false;
            if (utils::isHostRunning())
            {
                if (tracedHostWait)
                {
                    lg2::info(
                        "Manager::findAndCreateObjects(): Host is running");
                    tracedHostWait = false;
                }
                checkAllActiveSensors();
            }
            else
            {
                if (!tracedHostWait)
                {
                    lg2::info(
                        "Manager::findAndCreateObjects(): Waiting for host to start");
                    tracedHostWait = true;
                }
                discoverTimer->restartOnce(30s);

                if (throttlePldmTraceTimer->isEnabled())
                {
                    // Host is no longer running, disable throttle timer and
                    // make sure traces are not throttled
                    lg2::info("findAndCreateObjects(): disabling sensor timer");
                    throttlePldmTraceTimer->setEnabled(false);
                    pldmHandle->setTraceThrottle(false);
                }
            }
        }
    }
    else
    {
        lg2::info(
            "Manager::findAndCreateObjects(): Waiting for {FILE} to complete...",
            "FILE", HOST_ON_FILE);
        discoverTimer->restartOnce(10s);
    }
}

// Check if all occActive sensors are available
void Manager::checkAllActiveSensors()
{
    static bool allActiveSensorAvailable = false;
    static bool tracedSensorWait = false;
    static bool waitingForHost = false;

    if (open_power::occ::utils::isHostRunning())
    {
        if (waitingForHost)
        {
            waitingForHost = false;
            lg2::info("checkAllActiveSensors(): Host is now running");
        }

        // Start with the assumption that all sensors are available
        allActiveSensorAvailable = true;
        for (auto& [cid, chassisObj] : chassisObjects)
        {
            bool resolved = chassisObj->isSensorResolved(
                queuedActiveState, [this](instanceID inst) {
                    // Make sure PLDM traces are not throttled
                    // Ignore active sensor check if the OCCs are being reset
                    if (!resetInProgress)
                        pldmHandle->checkActiveSensor(inst);
                });

            if (!resolved)
            {
                allActiveSensorAvailable = false;
                if (!tracedSensorWait)
                {
                    lg2::info(
                        "checkAllActiveSensors(): Waiting on chassis{CID} Active sensor",
                        "CID", cid);
                    tracedSensorWait = true;
                    // Make sure PLDM traces are not throttled
                    pldmHandle->setTraceThrottle(false);
                    // Start timer to throttle PLDM traces when timer expires
                    onPldmTimeoutCreatePel = false;
                    throttlePldmTraceTimer->restartOnce(5min);
                }
                break;
            }
        }
    }
    else
    {
        if (!waitingForHost)
        {
            waitingForHost = true;
            lg2::info("checkAllActiveSensors(): Waiting for host to start");
            if (throttlePldmTraceTimer->isEnabled())
            {
                // Host is no longer running, disable throttle timer and
                // make sure traces are not throttled
                lg2::info("checkAllActiveSensors(): disabling sensor timer");
                throttlePldmTraceTimer->setEnabled(false);
                pldmHandle->setTraceThrottle(false);
            }
        }
    }

    if (allActiveSensorAvailable)
    {
        // All sensors were found, disable the discovery timer
        if (discoverTimer->isEnabled())
        {
            discoverTimer->setEnabled(false);
        }
        if (throttlePldmTraceTimer->isEnabled())
        {
            // Disable throttle timer and make sure traces are not throttled
            throttlePldmTraceTimer->setEnabled(false);
            pldmHandle->setTraceThrottle(false);
        }
        if (waitingForAllOccActiveSensors)
        {
            lg2::info(
                "checkAllActiveSensors(): OCC Active sensors are available");
            waitingForAllOccActiveSensors = false;

            // Retry processor association for any OCC that failed at
            // object-creation time (host D-Bus objects may not have existed
            // yet).
            for (auto& [cid, chassisObj] : chassisObjects)
            {
                chassisObj->updateProcAssociations();
            }

            if (resetRequired)
            {
                triggerOccReset();

                if (!waitForAllOccsTimer->isEnabled())
                {
                    lg2::warning(
                        "occsNotAllRunning: Restarting waitForAllOccTimer");
                    // restart occ wait timer to check status after reset
                    // completes
                    waitForAllOccsTimer->restartOnce(60s);
                }
            }
        }
        queuedActiveState.clear();
        tracedSensorWait = false;
    }
    else
    {
        // Not all sensors were available, so keep waiting
        if (!tracedSensorWait)
        {
            lg2::info(
                "checkAllActiveSensors(): Waiting for OCC Active sensors to become available");
            tracedSensorWait = true;
        }
        discoverTimer->restartOnce(10s);
    }
}

std::vector<int> Manager::findOCCsInDev()
{
    std::vector<int> occDevices;
    std::regex expr{R"(occ(\d+)$)"};

    for (auto& file : fs::directory_iterator("/dev"))
    {
        std::smatch match;
        std::string path{file.path().string()};
        if (std::regex_search(path, match, expr))
        {
            auto deviceNum = std::stoi(match[1].str());
            occDevices.push_back(deviceNum);
        }
    }

    return occDevices;
}

void Manager::createObjects(int deviceNum)
{
    // Build the D-Bus path for this /dev/occ# device:
    // e.g. device 0 -> OCC_CONTROL_ROOT/chassis1/occ0
    // Currently one OCC per chassis
    chassisID cid = static_cast<chassisID>(deviceNum + 1);
    auto path = fs::path(OCC_CONTROL_ROOT) /
                (std::string(CHASSIS_NAME) + std::to_string(cid)) /
                (std::string(OCC_NAME) + "0");

    // Create the ChassisObject for this chassis if it doesn't exist yet,
    // wiring its manager callback at construction time.
    if (chassisObjects.find(cid) == chassisObjects.end())
    {
        chassisObjects[cid] = std::make_unique<ChassisObject>(
            cid, std::bind(std::mem_fn(&Manager::chassisStatusCallback), this,
                           std::placeholders::_1, std::placeholders::_2));
        ++numChassis;
    }
    auto& chassisObj = chassisObjects[cid];

    // Build the OccObject that will be transferred to the ChassisObject
    auto occ = std::make_unique<OccObject>(
        event, path.c_str(), *this, pmode, cid,
        std::bind(std::mem_fn(&ChassisObject::onOccActiveChanged),
                  chassisObj.get(), std::placeholders::_2),
        // Callback will set flag indicating reset needs to be done
        // instead of immediately issuing a reset via PLDM.
        std::bind(std::mem_fn(&Manager::resetOccRequest), this,
                  std::placeholders::_1));

    // Create the power cap monitor object
    if (!pcap)
    {
        pcap = std::make_unique<open_power::occ::powercap::PowerCap>();
    }

    // Transfer ownership of the OccObject to the ChassisObject
    chassisObj->addOcc(std::move(occ));
    chassisObj->addPassThrough(
        std::make_unique<PassThrough>(path.c_str(), pmode));
}

// If a reset is not already outstanding, set a flag to indicate that a reset is
// needed.
void Manager::resetOccRequest(chassisID cid)
{
    if (!resetRequired)
    {
        resetRequired = true;
        chassisRequestingReset = cid;
        // TODO: pass the per-chassis PLDM instance once chassis-specific
        // reset support is added; use 0 for now.
        instanceRequestingReset = 0;
        lg2::error(
            "resetOccRequest: PM Complex reset was requested by chassis{CH}",
            "CH", cid);

        if (!std::any_of(chassisObjects.begin(), chassisObjects.end(),
                         [](const auto& kv) { return kv.second->active; }))
        {
            // The OCCs are all no longer active
            triggerOccReset();
        }
    }
    else if (cid != chassisRequestingReset)
    {
        lg2::warning(
            "resetOccRequest: Ignoring PM Complex reset request for chassis{CH}, "
            "because reset already outstanding for chassis{RCH}",
            "CH", cid, "RCH", chassisRequestingReset);
    }
}

// If a reset is not already in progress, trigger a PM Complex reset via PLDM
void Manager::triggerOccReset()
{
    if (!resetInProgress)
    {
        resetInProgress = true;
        lg2::error(
            "triggerOccReset: Initiating PM Complex reset for chassis{CH} (OCC{INST})",
            "CH", chassisRequestingReset, "INST", instanceRequestingReset);

        // Make sure ALL OCC comm stops to all OCCs before the reset
        for (auto& [chassis, chassisObj] : chassisObjects)
        {
            chassisObj->deactivateAll();
        }

        // TODO: pass the chassis-specific PLDM instance once per-chassis
        // reset support is added; instance is 0 until then.
        pldmHandle->resetOCC(instanceRequestingReset);
        resetRequired = false;
    }
    else
    {
        lg2::warning(
            "triggerOccReset: Ignoring PM Complex reset request for chassis{CH} "
            "(OCC{INST}), because reset already in process",
            "CH", chassisRequestingReset, "INST", instanceRequestingReset);
    }
}

void Manager::chassisStatusCallback(chassisID cid, bool status)
{
    if (status == true)
    {
        if (resetInProgress)
        {
            lg2::info(
                "chassisStatusCallback: Ignoring chassis{CH} activate because a reset has "
                "been initiated for chassis{RCH}",
                "CH", cid, "RCH", chassisRequestingReset);
            return;
        }

        // First chassis went active — allow time for all to come up
        if (std::count_if(chassisObjects.begin(), chassisObjects.end(),
                          [](const auto& kv) { return kv.second->active; }) ==
            1)
        {
            // Set the master OCC on the PowerMode object using the first
            // chassis to go active
            auto it = chassisObjects.find(cid);
            if (pmode && (it != chassisObjects.end()))
            {
                pmode->setMasterOcc(it->second->getMasterOcc().getPath());
            }

            waitForAllOccsTimer->restartOnce(60s);
        }

        auto allActive = [this]() {
            return !chassisObjects.empty() &&
                   std::all_of(chassisObjects.begin(), chassisObjects.end(),
                               [](const auto& kv) {
                                   return kv.second->active;
                               });
        };
        if (allActive())
        {
            // All chassis are now running
            if (waitForAllOccsTimer->isEnabled())
            {
                // stop occ wait timer
                waitForAllOccsTimer->setEnabled(false);
            }

            // All OCCs have been found, check if we need a reset
            if (resetRequired)
            {
                triggerOccReset();

                if (!waitForAllOccsTimer->isEnabled())
                {
                    lg2::warning(
                        "occsNotAllRunning: Restarting waitForAllOccTimer");
                    // restart occ wait timer
                    waitForAllOccsTimer->restartOnce(60s);
                }
            }
            else
            {
                // Verify master OCC and start presence monitor
                validateOccMaster();
            }
        }

        // Start poll timer if not already started (since at least one chassis
        // is active)
        if (!_pollTimer->isEnabled())
        {
            // A chassis just went active, PM Complex is just coming online so
            // clear any outstanding reset requests
            if (resetRequired)
            {
                resetRequired = false;
                lg2::error(
                    "chassisStatusCallback: clearing resetRequired (since chassis{CH} "
                    "went active, resetInProgress={RIP})",
                    "CH", cid, "RIP", resetInProgress);
            }

            lg2::info("Manager: OCCs will be polled every {TIME} seconds",
                      "TIME", pollInterval);

            // Send poll and start OCC poll timer
            pollerTimerExpired();
        }
    }
    else
    {
        auto anyActive = [this]() {
            return std::any_of(
                chassisObjects.begin(), chassisObjects.end(),
                [](const auto& kv) { return kv.second->active; });
        };

        if (!anyActive())
        {
            // No chassis are running
            if (resetInProgress)
            {
                // All OCC active sensors are clear (reset should be in
                // progress)
                lg2::info("chassisStatusCallback: Clearing resetInProgress "
                          "(chassis{CH}, status={STATUS})",
                          "CH", cid, "STATUS", status);
                resetInProgress = false;
                chassisRequestingReset = 0;
                instanceRequestingReset = 0xFF;
            }

            // Stop OCC poll timer
            if (_pollTimer->isEnabled())
            {
                lg2::info(
                    "Manager::chassisStatusCallback(): OCCs are not running, stopping poll timer");
                _pollTimer->setEnabled(false);
            }

            // stop wait timer
            if (waitForAllOccsTimer->isEnabled())
            {
                waitForAllOccsTimer->setEnabled(false);
            }
        }
        else if (resetInProgress)
        {
            lg2::info(
                "chassisStatusCallback: Skipping clear of resetInProgress "
                "(chassis{CH}, status={STATUS})",
                "CH", cid, "STATUS", status);
        }
    }

    if (waitingForAllOccActiveSensors)
    {
        if (utils::isHostRunning())
        {
            checkAllActiveSensors();
        }
    }
}

void Manager::sbeTimeout(unsigned int deviceInstance)
{
    chassisID cid = static_cast<chassisID>(deviceInstance + 1);
    auto it = chassisObjects.find(cid);
    if (it != chassisObjects.end())
    {
        lg2::info("SBE timeout, requesting HRESET (OCC{INST})", "INST",
                  deviceInstance);

#ifdef PHAL_SUPPORT
        setSBEState(deviceInstance, SBE_STATE_NOT_USABLE);
#endif

        it->second->setOccActive(deviceInstance, false);
        pldmHandle->sendHRESET(deviceInstance);
    }
}

bool Manager::updateOCCActive(instanceID instance, bool status)
{
    chassisID cid = static_cast<chassisID>(instance + 1);
    auto it = chassisObjects.find(cid);

    bool hostRunning = open_power::occ::utils::isHostRunning();
    if (it != chassisObjects.end())
    {
        if (!hostRunning && (status == true))
        {
            lg2::warning(
                "updateOCCActive: Host is not running yet (OCC{INST} active={STAT}), "
                "clearing sensor received",
                "INST", instance, "STAT", status);
            it->second->setPldmSensorReceived(instance, false);
            if (!waitingForAllOccActiveSensors)
            {
                lg2::info(
                    "updateOCCActive: Waiting for Host and all OCC Active Sensors");
                waitingForAllOccActiveSensors = true;
            }
            discoverTimer->restartOnce(30s);
            return false;
        }
        else
        {
            it->second->setPldmSensorReceived(instance, true);
            it->second->setOccActive(instance, status);
            return status;
        }
    }
    else
    {
        if (hostRunning)
        {
            lg2::warning(
                "updateOCCActive: No occObject to update for OCC{INST} (active={STAT})",
                "INST", instance, "STAT", status);
        }
        else
        {
            if (status == true)
            {
                lg2::warning(
                    "updateOCCActive: No occObjects and Host is not running yet (OCC{INST} "
                    "active={STAT})",
                    "INST", instance, "STAT", status);
            }
        }
        if (status == true)
        {
            // OCC went active
            queuedActiveState.insert(instance);
        }
        else
        {
            auto match = queuedActiveState.find(instance);
            if (match != queuedActiveState.end())
            {
                // OCC was disabled
                queuedActiveState.erase(match);
            }
        }
        return false;
    }
}

// Called upon pldm event To set powermode Safe Mode State for system.
void Manager::updateOccSafeMode(bool safeMode)
{
    pmode->updateDbusSafeMode(safeMode);
    // Update the processor throttle status on dbus for all chassis
    for (auto& [cid, chassisObj] : chassisObjects)
    {
        chassisObj->updateThrottle(safeMode, THROTTLED_SAFE);
    }
}

void Manager::sbeHRESETResult(instanceID instance, bool success)
{
    if (success)
    {
        lg2::info("HRESET succeeded (OCC{INST})", "INST", instance);

#ifdef PHAL_SUPPORT
        setSBEState(instance, SBE_STATE_BOOTED);
#endif

        // Re-enable communication with this OCC
        chassisID cid = static_cast<chassisID>(instance + 1);
        auto it = chassisObjects.find(cid);
        if (it != chassisObjects.end())
        {
            it->second->setOccActive(instance, true);
        }

        return;
    }

#ifdef PHAL_SUPPORT
    setSBEState(instance, SBE_STATE_FAILED);

    if (sbeCanDump(instance))
    {
        lg2::info("HRESET failed (OCC{INST}), triggering SBE dump", "INST",
                  instance);

        auto& bus = utils::getBus();
        uint32_t src6 = instance << 16;
        uint32_t logId =
            FFDC::createPEL("org.open_power.Processor.Error.SbeChipOpTimeout",
                            src6, "SBE command timeout");

        try
        {
            constexpr auto interface = "xyz.openbmc_project.Dump.Create";
            constexpr auto function = "CreateDump";

            std::string service =
                utils::getService(OP_DUMP_OBJ_PATH, interface);
            auto method = bus.new_method_call(service.c_str(), OP_DUMP_OBJ_PATH,
                                              interface, function);

            std::map<std::string, std::variant<std::string, uint64_t>>
                createParams{
                    {"com.ibm.Dump.Create.CreateParameters.ErrorLogId",
                     uint64_t(logId)},
                    {"com.ibm.Dump.Create.CreateParameters.DumpType",
                     "com.ibm.Dump.Create.DumpType.SBE"},
                    {"com.ibm.Dump.Create.CreateParameters.FailingUnitId",
                     uint64_t(instance)},
                };

            method.append(createParams);

            auto response = bus.call(method);
        }
        catch (const sdbusplus::exception_t& e)
        {
            constexpr auto ERROR_DUMP_DISABLED =
                "xyz.openbmc_project.Dump.Create.Error.Disabled";
            if (e.name() == ERROR_DUMP_DISABLED)
            {
                lg2::info("Dump is disabled, skipping");
            }
            else
            {
                lg2::error("Dump failed");
            }
        }
    }
#endif

    // SBE Reset failed, try PM Complex reset
    lg2::error("sbeHRESETResult: Forcing PM Complex reset");
    resetOccRequest(instance);
}

#ifdef PHAL_SUPPORT
bool Manager::sbeCanDump(unsigned int instance)
{
    struct pdbg_target* proc = getPdbgTarget(instance);

    if (!proc)
    {
        // allow the dump in the error case
        return true;
    }

    try
    {
        if (!openpower::phal::sbe::isDumpAllowed(proc))
        {
            return false;
        }

        if (openpower::phal::pdbg::isSbeVitalAttnActive(proc))
        {
            return false;
        }
    }
    catch (openpower::phal::exception::SbeError& e)
    {
        lg2::info("Failed to query SBE state");
    }

    // allow the dump in the error case
    return true;
}

void Manager::setSBEState(unsigned int instance, enum sbe_state state)
{
    struct pdbg_target* proc = getPdbgTarget(instance);

    if (!proc)
    {
        return;
    }

    try
    {
        openpower::phal::sbe::setState(proc, state);
    }
    catch (const openpower::phal::exception::SbeError& e)
    {
        lg2::error("Failed to set SBE state: {ERROR}", "ERROR", e.what());
    }
}

struct pdbg_target* Manager::getPdbgTarget(unsigned int instance)
{
    if (!pdbgInitialized)
    {
        try
        {
            openpower::phal::pdbg::init();
            pdbgInitialized = true;
        }
        catch (const openpower::phal::exception::PdbgError& e)
        {
            lg2::error("pdbg initialization failed");
            return nullptr;
        }
    }

    struct pdbg_target* proc = nullptr;
    pdbg_for_each_class_target("proc", proc)
    {
        if (pdbg_target_index(proc) == instance)
        {
            return proc;
        }
    }

    lg2::error("Failed to get pdbg target");
    return nullptr;
}
#endif

void Manager::pollerTimerExpired()
{
    if (!_pollTimer)
    {
        lg2::error("pollerTimerExpired() ERROR: Timer not defined");
        return;
    }

    if (resetRequired)
    {
        lg2::error("pollerTimerExpired() - Initiating PM Complex reset");
        triggerOccReset();

        if (!waitForAllOccsTimer->isEnabled())
        {
            lg2::warning("pollerTimerExpired: Restarting waitForAllOccTimer");
            // restart occ wait timer
            waitForAllOccsTimer->restartOnce(60s);
        }
        return;
    }
    for (auto& [cid, chassisObj] : chassisObjects)
    {
        chassisObj->poll();
    }

    if (std::any_of(chassisObjects.begin(), chassisObjects.end(),
                    [](const auto& kv) { return kv.second->active; }))
    {
        // Restart OCC poll timer
        _pollTimer->restartOnce(std::chrono::seconds(pollInterval));
    }
    else
    {
        // No OCCs running, so poll timer will not be restarted
        lg2::info(
            "Manager::pollerTimerExpired: poll timer will not be restarted");
    }
}

// Read the altitude from DBus
void Manager::readAltitude()
{
    static bool traceAltitudeErr = true;

    utils::PropertyValue altitudeProperty{};
    try
    {
        altitudeProperty = utils::getProperty(ALTITUDE_PATH, ALTITUDE_INTERFACE,
                                              ALTITUDE_PROP);
        auto sensorVal = std::get<double>(altitudeProperty);
        if (sensorVal < 0xFFFF)
        {
            if (sensorVal < 0)
            {
                altitude = 0;
            }
            else
            {
                // Round to nearest meter
                altitude = uint16_t(sensorVal + 0.5);
            }
            lg2::debug("readAltitude: sensor={VALUE} ({ALT}m)", "VALUE",
                       sensorVal, "ALT", altitude);
            traceAltitudeErr = true;
        }
        else
        {
            if (traceAltitudeErr)
            {
                traceAltitudeErr = false;
                lg2::debug("Invalid altitude value: {ALT}", "ALT", sensorVal);
            }
        }
    }
    catch (const sdbusplus::exception_t& e)
    {
        if (traceAltitudeErr)
        {
            traceAltitudeErr = false;
            lg2::info("Unable to read Altitude: {ERROR}", "ERROR", e.what());
        }
        altitude = 0xFFFF; // not available
    }
}

// Callback function when ambient temperature changes
void Manager::ambientCallback(sdbusplus::message_t& msg)
{
    double currentTemp = 0;
    uint8_t truncatedTemp = 0xFF;
    std::string msgSensor;
    std::map<std::string, std::variant<double>> msgData;
    msg.read(msgSensor, msgData);

    auto valPropMap = msgData.find(AMBIENT_PROP);
    if (valPropMap == msgData.end())
    {
        lg2::debug("ambientCallback: Unknown ambient property changed");
        return;
    }
    currentTemp = std::get<double>(valPropMap->second);
    if (std::isnan(currentTemp))
    {
        truncatedTemp = 0xFF;
    }
    else
    {
        if (currentTemp < 0)
        {
            truncatedTemp = 0;
        }
        else
        {
            // Round to nearest degree C
            truncatedTemp = uint8_t(currentTemp + 0.5);
        }
    }

    // If ambient changes, notify OCCs
    if (truncatedTemp != ambient)
    {
        lg2::debug("ambientCallback: Ambient change from {OLD} to {NEW}C",
                   "OLD", ambient, "NEW", currentTemp);

        ambient = truncatedTemp;
        if (altitude == 0xFFFF)
        {
            // No altitude yet, try reading again
            readAltitude();
        }

        lg2::debug("ambientCallback: Ambient: {TEMP}C, altitude: {ALT}m",
                   "TEMP", ambient, "ALT", altitude);

        // Send ambient and altitude to all OCCs
        for (auto& [cid, chassisObj] : chassisObjects)
        {
            chassisObj->sendAmbient(ambient, altitude);
        }
    }
}

// return the current ambient and altitude readings
void Manager::getAmbientData(bool& ambientValid, uint8_t& ambientTemp,
                             uint16_t& altitudeValue) const
{
    ambientValid = true;
    ambientTemp = ambient;
    altitudeValue = altitude;

    if (ambient == 0xFF)
    {
        ambientValid = false;
    }
}

// Called when waitForAllOccsTimer expires
// After the first OCC goes active, this timer will be started (60 seconds)
void Manager::occsNotAllRunning()
{
    if (resetInProgress)
    {
        lg2::warning(
            "occsNotAllRunning: Ignoring waitForAllOccsTimer because reset is in progress");
        return;
    }
    auto activeCnt =
        std::count_if(chassisObjects.begin(), chassisObjects.end(),
                      [](const auto& kv) { return kv.second->active; });
    if (static_cast<size_t>(activeCnt) != chassisObjects.size())
    {
        // Not all OCCs went active
        lg2::warning(
            "occsNotAllRunning: Active OCC count ({COUNT}) does not match expected count ({EXP})",
            "COUNT", activeCnt, "EXP", chassisObjects.size());
        // Procs may be garded, so may be expected
    }

    if (resetRequired)
    {
        triggerOccReset();

        if (!waitForAllOccsTimer->isEnabled())
        {
            lg2::warning("occsNotAllRunning: Restarting waitForAllOccTimer");
            // restart occ wait timer
            waitForAllOccsTimer->restartOnce(60s);
        }
    }
    else
    {
        validateOccMaster();
    }
}

// Called when throttlePldmTraceTimer expires.
// If this timer expires, that indicates there are no OCC active sensor PDRs
// found which will trigger pldm traces to be throttled.
// The second time this timer expires, a PEL will get created.
void Manager::throttlePldmTraceExpired()
{
    if (utils::isHostRunning())
    {
        if (!onPldmTimeoutCreatePel)
        {
            // Throttle traces
            pldmHandle->setTraceThrottle(true);
            // Restart timer to log a PEL when timer expires
            onPldmTimeoutCreatePel = true;
            throttlePldmTraceTimer->restartOnce(40min);
        }
        else
        {
            lg2::error(
                "throttlePldmTraceExpired(): OCC active sensors still not available!");
            // Create PEL
            createPldmSensorPEL();
        }
    }
    else
    {
        // Make sure traces are not throttled
        pldmHandle->setTraceThrottle(false);
        lg2::info(
            "throttlePldmTraceExpired(): host it not running ignoring sensor timer");
    }
}

void Manager::createPldmSensorPEL()
{
    Error::Descriptor d = Error::Descriptor(MISSING_OCC_SENSORS_PATH);
    std::map<std::string, std::string> additionalData;

    additionalData.emplace("_PID", std::to_string(getpid()));

    lg2::info(
        "createPldmSensorPEL(): Unable to find PLDM sensors for the OCCs");

    auto& bus = utils::getBus();

    try
    {
        FFDCFiles ffdc;
        // Add occ-control journal traces to PEL FFDC
        auto occJournalFile =
            FFDC::addJournalEntries(ffdc, "openpower-occ-control", 40);

        static constexpr auto loggingObjectPath =
            "/xyz/openbmc_project/logging";
        static constexpr auto opLoggingInterface = "org.open_power.Logging.PEL";
        std::string service =
            utils::getService(loggingObjectPath, opLoggingInterface);
        auto method =
            bus.new_method_call(service.c_str(), loggingObjectPath,
                                opLoggingInterface, "CreatePELWithFFDCFiles");

        // Set level to Warning (Predictive).
        auto level =
            sdbusplus::xyz::openbmc_project::Logging::server::convertForMessage(
                sdbusplus::xyz::openbmc_project::Logging::server::Entry::Level::
                    Warning);

        method.append(d.path, level, additionalData, ffdc);
        bus.call(method);
    }
    catch (const sdbusplus::exception_t& e)
    {
        lg2::error("Failed to create MISSING_OCC_SENSORS PEL: {ERROR}", "ERROR",
                   e.what());
    }
}

// Verify master OCC per chassis and start presence monitor
void Manager::validateOccMaster()
{
    for (auto& [cid, chassisObj] : chassisObjects)
    {
        chassisObj->isSensorResolved(queuedActiveState, [this, &chassisObj](
                                                            instanceID inst) {
            if (utils::isHostRunning())
            {
                pldmHandle->checkActiveSensor(inst);
                if (chassisObj->getPldmSensorReceived(inst))
                {
                    lg2::info(
                        "validateOccMaster: OCC{INST} is ACTIVE after reading sensor",
                        "INST", inst);
                }
            }
        });

        chassisObj->validateOccMaster(pmode);
    }
}

void Manager::updatePcapBounds(bool& parmsChanged, uint32_t& capSoftMin,
                               uint32_t& capHardMin, uint32_t& capMax) const
{
    if (pcap)
    {
        // Use the master OCC from the first (lowest-numbered) chassis
        auto it = chassisObjects.begin();
        if (it != chassisObjects.end())
        {
            OccObject& master = it->second->getMasterOcc();
            lg2::info("updatePcapBounds: setting OCC{INST} as pcap master",
                      "INST", master.getOccInstanceID());
            pcap->setMasterOccObj(master);
        }
        pcap->updatePcapBounds(parmsChanged, capSoftMin, capHardMin, capMax);
    }
}

// Clean up any variables since the OCC is no longer running.
// Called when pldm receives an event indicating host is powered off.
void Manager::hostPoweredOff()
{
    if (resetRequired)
    {
        lg2::info("hostPoweredOff: Clearing resetRequired for chassis{CH}",
                  "CH", chassisRequestingReset);
        resetRequired = false;
    }
    if (resetInProgress)
    {
        lg2::info("hostPoweredOff: Clearing resetInProgress for chassis{CH}",
                  "CH", chassisRequestingReset);
        resetInProgress = false;
    }
    chassisRequestingReset = 0;
    instanceRequestingReset = 0xFF;

    // Deactivate all OCCs — clears sensors, stops device, fires callbacks
    // that stop the poll timer and clear resetInProgress.
    for (auto& [cid, chassisObj] : chassisObjects)
    {
        chassisObj->deactivateAll();
    }
}

void Manager::collectDumpData(sdeventplus::source::Signal&,
                              const struct signalfd_siginfo*)
{
    json data;
    lg2::info("collectDumpData()");
    data["objectCount"] = std::to_string(numChassis) + " chassis";
    if (numChassis > 0)
    {
        try
        {
            for (auto& [cid, chassisObj] : chassisObjects)
            {
                chassisObj->collectDumpData(data);
            }
        }
        catch (const std::exception& e)
        {
            lg2::error("Failed to collect OCC dump data: {ERR}", "ERR",
                       e.what());
        }
    }

    std::ofstream file{Manager::dumpFile};
    if (!file)
    {
        lg2::error("Failed to open {FILE} for occ-control data", "FILE",
                   Manager::dumpFile);
        return;
    }

    file << std::setw(4) << data;
}

} // namespace occ
} // namespace open_power
