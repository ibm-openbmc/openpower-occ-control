#include "config.h"

#include "occ_manager.hpp"

#include "occ_dbus.hpp"
#include "occ_errors.hpp"
#include "utils.hpp"

#include <nlohmann/json.hpp>
#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

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
// (function is called everytime the discoverTimer expires)
// - create the PowerMode object to control OCC modes
// - create statusObjects for each OCC device found
// - waits for OCC Active sensors PDRs to become available
// - restart discoverTimer if all data is not available yet
void Manager::findAndCreateObjects()
{
    if (!pmode)
    {
        // Create the power mode object
        pmode = std::make_unique<powermode::PowerMode>(
            *this, powermode::PMODE_PATH, powermode::PIPS_PATH, event);
    }

    if (!fs::exists(HOST_ON_FILE))
    {
        static bool statusObjCreated = false;
        if (!statusObjCreated)
        {
            // Create the OCCs based on on the /dev/occX devices
            auto occs = findOCCsInDev();

            if (occs.empty() || (prevOCCSearch.size() != occs.size()))
            {
                // Something changed or no OCCs yet, try again in 10s.
                // Note on the first pass prevOCCSearch will be empty,
                // so there will be at least one delay to give things
                // a chance to settle.
                prevOCCSearch = occs;

                lg2::info(
                    "Manager::findAndCreateObjects(): Waiting for OCCs (currently {QTY})",
                    "QTY", occs.size());

                discoverTimer->restartOnce(10s);
            }
            else
            {
                // All OCCs appear to be available, create status objects

                // createObjects requires OCC0 first.
                std::sort(occs.begin(), occs.end());

                lg2::info(
                    "Manager::findAndCreateObjects(): Creating {QTY} OCC Status Objects",
                    "QTY", occs.size());
                for (auto id : occs)
                {
                    createObjects(std::string(OCC_NAME) + std::to_string(id));
                }
                statusObjCreated = true;
                waitingForAllOccActiveSensors = true;

                // Find/update the processor path associated with each OCC
                for (auto& obj : statusObjects)
                {
                    obj->updateProcAssociation();
                }
            }
        }

        if (statusObjCreated && waitingForAllOccActiveSensors)
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

        // Start with the assumption that all are available
        allActiveSensorAvailable = true;
        for (auto& obj : statusObjects)
        {
            if ((!obj->occActive()) && (!obj->getPldmSensorReceived()))
            {
                auto instance = obj->getOccInstanceID();
                // Check if sensor was queued while waiting for discovery
                auto match = queuedActiveState.find(instance);
                if (match != queuedActiveState.end())
                {
                    queuedActiveState.erase(match);
                    lg2::info(
                        "checkAllActiveSensors(): OCC{INST} is ACTIVE (queued)",
                        "INST", instance);
                    obj->occActive(true);
                }
                else
                {
                    allActiveSensorAvailable = false;
                    if (!tracedSensorWait)
                    {
                        lg2::info(
                            "checkAllActiveSensors(): Waiting on OCC{INST} Active sensor",
                            "INST", instance);
                        tracedSensorWait = true;
                        // Make sure PLDM traces are not throttled
                        pldmHandle->setTraceThrottle(false);
                        // Start timer to throttle PLDM traces when timer
                        // expires
                        onPldmTimeoutCreatePel = false;
                        throttlePldmTraceTimer->restartOnce(5min);
                    }
                    // Ignore active sensor check if the OCCs are being reset
                    if (!resetInProgress)
                    {
                        pldmHandle->checkActiveSensor(obj->getOccInstanceID());
                    }
                    break;
                }
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

            if (resetRequired)
            {
                initiateOccRequest(resetInstance);

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
    std::vector<int> occs;
    std::regex expr{R"(occ(\d+)$)"};

    for (auto& file : fs::directory_iterator("/dev"))
    {
        std::smatch match;
        std::string path{file.path().string()};
        if (std::regex_search(path, match, expr))
        {
            auto num = std::stoi(match[1].str());

            // /dev numbering starts at 1, ours starts at 0.
            occs.push_back(num - 1);
        }
    }

    return occs;
}

int Manager::cpuCreated(sdbusplus::message_t& msg)
{
    namespace fs = std::filesystem;

    auto o = msg.unpack<sdbusplus::message::object_path>();

    fs::path cpuPath(std::string(std::move(o)));

    auto name = cpuPath.filename().string();
    auto index = name.find(CPU_NAME);
    name.replace(index, std::strlen(CPU_NAME), OCC_NAME);

    createObjects(name);

    return 0;
}

void Manager::createObjects(const std::string& occ)
{
    auto path = fs::path(OCC_CONTROL_ROOT) / occ;

    statusObjects.emplace_back(std::make_unique<Status>(
        event, path.c_str(), *this, pmode,
        std::bind(std::mem_fn(&Manager::statusCallBack), this,
                  std::placeholders::_1, std::placeholders::_2),
        // Callback will set flag indicating reset needs to be done
        // instead of immediately issuing a reset via PLDM.
        std::bind(std::mem_fn(&Manager::resetOccRequest), this,
                  std::placeholders::_1)));

    // Create the power cap monitor object
    if (!pcap)
    {
        pcap = std::make_unique<open_power::occ::powercap::PowerCap>(
            *statusObjects.back());
    }

    if (statusObjects.back()->isMasterOcc())
    {
        lg2::info("Manager::createObjects(): OCC{INST} is the master", "INST",
                  statusObjects.back()->getOccInstanceID());
        _pollTimer->setEnabled(false);

        // Set the master OCC on the PowerMode object
        pmode->setMasterOcc(path);
    }

    passThroughObjects.emplace_back(
        std::make_unique<PassThrough>(path.c_str(), pmode));
}

// If a reset is not already outstanding, set a flag to indicate that a reset is
// needed.
void Manager::resetOccRequest(instanceID instance)
{
    if (!resetRequired)
    {
        resetRequired = true;
        resetInstance = instance;
        lg2::error(
            "resetOccRequest: PM Complex reset was requested due to OCC{INST}",
            "INST", instance);
    }
    else if (instance != resetInstance)
    {
        lg2::warning(
            "resetOccRequest: Ignoring PM Complex reset request for OCC{INST}, because reset already outstanding for OCC{RINST}",
            "INST", instance, "RINST", resetInstance);
    }
}

// If a reset has not been started, initiate an OCC reset via PLDM
void Manager::initiateOccRequest(instanceID instance)
{
    if (!resetInProgress)
    {
        resetInProgress = true;
        resetInstance = instance;
        lg2::error(
            "initiateOccRequest: Initiating PM Complex reset due to OCC{INST}",
            "INST", instance);

        // Make sure ALL OCC comm stops to all OCCs before the reset
        for (auto& obj : statusObjects)
        {
            if (obj->occActive())
            {
                obj->occActive(false);
            }
        }

        pldmHandle->resetOCC(instance);
        resetRequired = false;
    }
    else
    {
        lg2::warning(
            "initiateOccRequest: Ignoring PM Complex reset request for OCC{INST}, because reset already in process for OCC{RINST}",
            "INST", instance, "RINST", resetInstance);
    }
}

void Manager::statusCallBack(instanceID instance, bool status)
{
    if (status == true)
    {
        if (resetInProgress)
        {
            lg2::info(
                "statusCallBack: Ignoring OCC{INST} activate because a reset has been initiated due to OCC{RINST}",
                "INST", instance, "RINST", resetInstance);
            return;
        }

        // OCC went active
        ++activeCount;

        if (activeCount == 1)
        {
            // First OCC went active (allow some time for all OCCs to go active)
            waitForAllOccsTimer->restartOnce(60s);
        }

        if (activeCount == statusObjects.size())
        {
            // All OCCs are now running
            if (waitForAllOccsTimer->isEnabled())
            {
                // stop occ wait timer
                waitForAllOccsTimer->setEnabled(false);
            }

            // All OCCs have been found, check if we need a reset
            if (resetRequired)
            {
                initiateOccRequest(resetInstance);

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

        // Start poll timer if not already started (since at least one OCC is
        // running)
        if (!_pollTimer->isEnabled())
        {
            // An OCC just went active, PM Complex is just coming online so
            // clear any outstanding reset requests
            if (resetRequired)
            {
                resetRequired = false;
                lg2::error(
                    "statusCallBack: clearing resetRequired (since OCC{INST} went active, resetInProgress={RIP})",
                    "INST", instance, "RIP", resetInProgress);
            }

            lg2::info("Manager: OCCs will be polled every {TIME} seconds",
                      "TIME", pollInterval);

            // Send poll and start OCC poll timer
            pollerTimerExpired();
        }
    }
    else
    {
        // OCC went away
        if (activeCount > 0)
        {
            --activeCount;
        }
        else
        {
            lg2::info("OCC{INST} disabled, and no other OCCs are active",
                      "INST", instance);
        }

        if (activeCount == 0)
        {
            // No OCCs are running

            if (resetInProgress)
            {
                // All OCC active sensors are clear (reset should be in
                // progress)
                lg2::info(
                    "statusCallBack: Clearing resetInProgress (activeCount={COUNT}, OCC{INST}, status={STATUS})",
                    "COUNT", activeCount, "INST", instance, "STATUS", status);
                resetInProgress = false;
                resetInstance = 255;
            }

            // Stop OCC poll timer
            if (_pollTimer->isEnabled())
            {
                lg2::info(
                    "Manager::statusCallBack(): OCCs are not running, stopping poll timer");
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
                "statusCallBack: Skipping clear of resetInProgress (activeCount={COUNT}, OCC{INST}, status={STATUS})",
                "COUNT", activeCount, "INST", instance, "STATUS", status);
        }
        // Clear OCC sensors
        for (auto& obj : statusObjects)
        {
            if (instance == obj->getOccInstanceID())
            {
                obj->setSensorValueToNaN();
                break;
            }
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

void Manager::sbeTimeout(unsigned int instance)
{
    auto obj = std::find_if(statusObjects.begin(), statusObjects.end(),
                            [instance](const auto& obj) {
                                return instance == obj->getOccInstanceID();
                            });

    if (obj != statusObjects.end() && (*obj)->occActive())
    {
        lg2::info("SBE timeout, requesting HRESET (OCC{INST})", "INST",
                  instance);

#ifdef PHAL_SUPPORT
        setSBEState(instance, SBE_STATE_NOT_USABLE);
#endif

        // Stop communication with this OCC
        (*obj)->occActive(false);

        pldmHandle->sendHRESET(instance);
    }
}

bool Manager::updateOCCActive(instanceID instance, bool status)
{
    auto obj = std::find_if(statusObjects.begin(), statusObjects.end(),
                            [instance](const auto& obj) {
                                return instance == obj->getOccInstanceID();
                            });

    const bool hostRunning = open_power::occ::utils::isHostRunning();
    if (obj != statusObjects.end())
    {
        if (!hostRunning && (status == true))
        {
            lg2::warning(
                "updateOCCActive: Host is not running yet (OCC{INST} active={STAT}), clearing sensor received",
                "INST", instance, "STAT", status);
            (*obj)->setPldmSensorReceived(false);
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
            (*obj)->setPldmSensorReceived(true);
            return (*obj)->occActive(status);
        }
    }
    else
    {
        if (hostRunning)
        {
            lg2::warning(
                "updateOCCActive: No status object to update for OCC{INST} (active={STAT})",
                "INST", instance, "STAT", status);
        }
        else
        {
            if (status == true)
            {
                lg2::warning(
                    "updateOCCActive: No status objects and Host is not running yet (OCC{INST} active={STAT})",
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
    // Update the processor throttle status on dbus
    for (auto& obj : statusObjects)
    {
        obj->updateThrottle(safeMode, THROTTLED_SAFE);
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
        auto obj = std::find_if(statusObjects.begin(), statusObjects.end(),
                                [instance](const auto& obj) {
                                    return instance == obj->getOccInstanceID();
                                });
        if (obj != statusObjects.end() && (!(*obj)->occActive()))
        {
            (*obj)->occActive(true);
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
        initiateOccRequest(resetInstance);

        if (!waitForAllOccsTimer->isEnabled())
        {
            lg2::warning("pollerTimerExpired: Restarting waitForAllOccTimer");
            // restart occ wait timer
            waitForAllOccsTimer->restartOnce(60s);
        }
        return;
    }
    for (auto& obj : statusObjects)
    {
        if (!obj->occActive())
        {
            // OCC is not running yet
            obj->setSensorValueToNaN();
            continue;
        }

        // get OCC Poll Response and handle actions needed.
        obj->PollHandler();
    }

    if (activeCount > 0)
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
        for (auto& obj : statusObjects)
        {
            if (obj->occActive())
            {
                obj->sendAmbient(ambient, altitude);
            }
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
    if (activeCount != statusObjects.size())
    {
        // Not all OCCs went active
        lg2::warning(
            "occsNotAllRunning: Active OCC count ({COUNT}) does not match expected count ({EXP})",
            "COUNT", activeCount, "EXP", statusObjects.size());
        // Procs may be garded, so may be expected
    }

    if (resetRequired)
    {
        initiateOccRequest(resetInstance);

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

// Verify single master OCC and start presence monitor
void Manager::validateOccMaster()
{
    int masterInstance = -1;
    for (auto& obj : statusObjects)
    {
        auto instance = obj->getOccInstanceID();

        if (!obj->occActive())
        {
            if (utils::isHostRunning())
            {
                // Check if sensor was queued while waiting for discovery
                auto match = queuedActiveState.find(instance);
                if (match != queuedActiveState.end())
                {
                    queuedActiveState.erase(match);
                    lg2::info("validateOccMaster: OCC{INST} is ACTIVE (queued)",
                              "INST", instance);
                    obj->occActive(true);
                }
                else
                {
                    // OCC does not appear to be active yet, check active sensor
                    pldmHandle->checkActiveSensor(instance);
                    if (obj->occActive())
                    {
                        lg2::info(
                            "validateOccMaster: OCC{INST} is ACTIVE after reading sensor",
                            "INST", instance);
                    }
                }
            }
            else
            {
                lg2::warning(
                    "validateOccMaster: HOST is not running (OCC{INST})",
                    "INST", instance);
                return;
            }
        }

        if (obj->isMasterOcc())
        {
            obj->addPresenceWatchMaster();

            if (masterInstance == -1)
            {
                masterInstance = instance;
            }
            else
            {
                lg2::error(
                    "validateOccMaster: Multiple OCC masters! ({MAST1} and {MAST2})",
                    "MAST1", masterInstance, "MAST2", instance);
                // request reset
                obj->deviceError(Error::Descriptor(PRESENCE_ERROR_PATH));
            }
        }
    }

    if (masterInstance < 0)
    {
        lg2::error("validateOccMaster: Master OCC not found! (of {NUM} OCCs)",
                   "NUM", statusObjects.size());
        // request reset
        statusObjects.front()->deviceError(
            Error::Descriptor(PRESENCE_ERROR_PATH));
    }
    else
    {
        lg2::info("validateOccMaster: OCC{INST} is master of {COUNT} OCCs",
                  "INST", masterInstance, "COUNT", activeCount);

        pmode->updateDbusSafeMode(false);
    }
}

void Manager::updatePcapBounds(bool& parmsChanged, uint32_t& capSoftMin,
                               uint32_t& capHardMin, uint32_t& capMax) const
{
    if (pcap)
    {
        pcap->updatePcapBounds(parmsChanged, capSoftMin, capHardMin, capMax);
    }
}

// Clean up any variables since the OCC is no longer running.
// Called when pldm receives an event indicating host is powered off.
void Manager::hostPoweredOff()
{
    if (resetRequired)
    {
        lg2::info("hostPoweredOff: Clearing resetRequired for OCC{INST}",
                  "INST", resetInstance);
        resetRequired = false;
    }
    if (resetInProgress)
    {
        lg2::info("hostPoweredOff: Clearing resetInProgress for OCC{INST}",
                  "INST", resetInstance);
        resetInProgress = false;
    }
    resetInstance = 255;
}

void Manager::collectDumpData(sdeventplus::source::Signal&,
                              const struct signalfd_siginfo*)
{
    json data;
    lg2::info("collectDumpData()");
    data["objectCount"] = std::to_string(statusObjects.size()) + " OCC objects";
    if (statusObjects.size() > 0)
    {
        try
        {
            for (auto& occ : statusObjects)
            {
                json occData;
                auto instance = occ->getOccInstanceID();
                std::string occName = "occ" + std::to_string(instance);

                if (occ->occActive())
                {
                    // OCC General Info
                    occData["occState"] = "ACTIVE";
                    occData["occRole"] =
                        occ->isMasterOcc() ? "MASTER" : "SECONDARY";
                    occData["occHwmonPath"] =
                        occ->getHwmonPath().generic_string();

                    // OCC Poll Response
                    std::vector<std::uint8_t> cmd = {0x00, 0x00, 0x01, 0x20};
                    std::vector<std::uint8_t> rsp;
                    std::vector<std::string> rspHex;
                    rsp = passThroughObjects[instance]->send(cmd);
                    if (rsp.size() > 5)
                    {
                        rsp.erase(rsp.begin(),
                                  rsp.begin() + 5); // Strip rsp header
                        rspHex = utils::hex_dump(rsp);
                        occData["pollResponse"] = rspHex;
                    }

                    // Debug Data: WOF Dynamic Data
                    cmd = {0x40, 0x00, 0x01, 0x01};
                    rsp = passThroughObjects[instance]->send(cmd);
                    if (rsp.size() > 5)
                    {
                        rsp.erase(rsp.begin(),
                                  rsp.begin() + 5); // Strip rsp header
                        rspHex = utils::hex_dump(rsp);
                        occData["wofDataDynamic"] = rspHex;
                    }

                    // Debug Data: WOF Dynamic Data
                    cmd = {0x40, 0x00, 0x01, 0x0A};
                    rsp = passThroughObjects[instance]->send(cmd);
                    if (rsp.size() > 5)
                    {
                        rsp.erase(rsp.begin(),
                                  rsp.begin() + 5); // Strip rsp header
                        rspHex = utils::hex_dump(rsp);
                        occData["wofDataStatic"] = rspHex;
                    }
                }
                else
                {
                    occData["occState"] = "NOT ACTIVE";
                }

                data[occName] = occData;
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
