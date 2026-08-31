
#include "occ_poll_handler.hpp"

#include "occ_command.hpp"
#include "occ_dbus.hpp"
#include "occ_object.hpp"

#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/lg2.hpp>

namespace open_power
{
namespace occ
{

using namespace phosphor::logging;
using namespace std::literals::chrono_literals;

constexpr uint32_t fruTypeNotAvailable = 0xFF;

bool OccPollHandler::BuildTempDbusPaths(
    std::string& sensorPath, std::string& dvfsTempPath,
    std::string& tcontrolTempPath, const uint32_t SensorID,
    const uint32_t fruTypeValue, const uint32_t occInstance,
    const bool isHottest)
{
    bool returnSensorFound = true;

    // P12 multi-chassis naming: chassis_procY_<suffix>
    // "chassis" is a fixed prefix until per-chassis detection is available.
    const std::string procPrefix =
        "chassis_proc" + std::to_string(occInstance) + "_";

    sensorPath = OCC_SENSORS_ROOT + std::string("/temperature/");

    if (fruTypeValue == VRMVdd)
    {
        sensorPath.append(procPrefix + "vrm_vdd_temp");
    }
    else if (fruTypeValue == processorIoRing)
    {
        sensorPath.append(procPrefix + "ioring_temp");
        dvfsTempPath = std::string{OCC_SENSORS_ROOT} + "/temperature/" +
                       procPrefix + "ioring_dvfs_temp";
    }
    else
    {
        uint16_t type = (SensorID & 0xFF000000) >> 24;
        uint16_t instanceID = SensorID & 0x0000FFFF;

        if (type == SID_TYPE_DIMM)
        {
            if (fruTypeValue == fruTypeNotAvailable)
            {
                // Not all DIMM related temps are available to read
                returnSensorFound = false;
            }
            else
            {
                auto iter = dimmTempSensorName.find(fruTypeValue);
                if (iter == dimmTempSensorName.end())
                {
                    returnSensorFound = false;
                }
                else
                {
                    if (isHottest)
                    {
                        sensorPath.append(
                            procPrefix + "dimm" + iter->second + "_hottest");
                    }
                    else
                    {
                        sensorPath.append(
                            procPrefix + "dimm" + std::to_string(instanceID) +
                            iter->second);
                    }
                    dvfsTempPath = std::string{OCC_SENSORS_ROOT} +
                                   "/temperature/" + procPrefix +
                                   dimmDVFSSensorName.at(fruTypeValue);
                }
            }
        }
        else if (type == SID_TYPE_CORE)
        {
            // P12 tap group layout (8 taps, 8 cores each, 64 cores total per
            // proc):
            //   tap0 = cores  0-7
            //   tap1 = cores  8-15
            //   ...
            // OCC reports core which is the instanceID (0-63).
            // tapGroup  = instanceID / 8  (0-7)
            // coreInTap = instanceID % 8  (0-7)
            //
            // Full path (per-core): chassis_procY_coregroupZ_coreW_temp
            //
            // MMA tap-level sentinel: instanceID = 0xFF00 | tapGroup
            //   The OCC uses this to send one aggregate MMA value per tap.
            //   Path: chassis_procY_coregroupZ_mma_temp
            uint16_t tapGroup = instanceID / 8;
            uint16_t coreInTap = instanceID % 8;
            if (fruTypeValue == processorCore)
            {
                if (isHottest)
                {
                    // The OCC determines the tap temperature — publish it as
                    // the tap-level sensor (chassis_procY_coregroupZ_temp).
                    sensorPath.append(procPrefix + "coregroup" +
                                      std::to_string(tapGroup) + "_temp");
                }
                else
                {
                    sensorPath.append(
                        procPrefix + "coregroup" + std::to_string(tapGroup) +
                        "_core" + std::to_string(coreInTap) + "_temp");
                }
                dvfsTempPath = std::string{OCC_SENSORS_ROOT} + "/temperature/" +
                               procPrefix + "coregroup" +
                               std::to_string(tapGroup) + "_dvfs_temp";
                tcontrolTempPath = std::string{OCC_SENSORS_ROOT} +
                                   "/temperature/" + procPrefix + "coregroup" +
                                   std::to_string(tapGroup) + "_tcontrol_temp";
            }
            else if (fruTypeValue == processorMMA)
            {
                // Tap-level MMA only: OCC signals with instanceID = 0xFF00 |
                // tapGroup MMA has tcontrol but no dvfs threshold.
                if ((instanceID & 0xFF00) == 0xFF00)
                {
                    uint16_t tapNum = instanceID & 0x00FF;
                    sensorPath.append(procPrefix + "coregroup" +
                                      std::to_string(tapNum) + "_mma_temp");
                    tcontrolTempPath =
                        std::string{OCC_SENSORS_ROOT} + "/temperature/" +
                        procPrefix + "coregroup" + std::to_string(tapNum) +
                        "_mma_tcontrol_temp";
                }
                else
                {
                    returnSensorFound = false;
                }
            }
            else
            {
                returnSensorFound = false;
            }
        }
        else
        {
            returnSensorFound = false;
        }
    }

    return returnSensorFound;
} // end BuildTempDbusPaths

} // namespace occ
} // namespace open_power
