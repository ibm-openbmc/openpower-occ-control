
#include "occ_poll_handler.hpp"
#include "occ_dbus.hpp"
#include "occ_command.hpp"
#include "occ_status.hpp"

#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/lg2.hpp>


namespace open_power
{
namespace occ
{

using namespace phosphor::logging;
using namespace std::literals::chrono_literals;

constexpr uint32_t fruTypeNotAvailable = 0xFF;


bool OccPollHandler::BuildTempDbusPaths(std::string& sensorPath,
                                     std::string& dvfsTempPath,
                                     const uint32_t SensorID,
                                     const uint32_t fruTypeValue,
                                     const uint32_t occInstance)
{
    bool returnSensorFound = true;
    sensorPath = OCC_SENSORS_ROOT + std::string("/temperature/");

    if (fruTypeValue == VRMVdd)
    {
        sensorPath.append(
            "vrm_vdd" + std::to_string(occInstance) + "_temp");
    }
    else if (fruTypeValue == processorIoRing)
    {
        sensorPath.append(
            "proc" + std::to_string(occInstance) + "_ioring_temp");
        dvfsTempPath = std::string{OCC_SENSORS_ROOT} + "/temperature/proc" +
                        std::to_string(occInstance) + "_ioring_dvfs_temp";
    }
    else
    {
        uint16_t type = (SensorID & 0xFF000000) >> 24;
        uint16_t instanceID = SensorID & 0x0000FFFF;

        if (type == OCC_DIMM_TEMP_SENSOR_TYPE)
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
                    sensorPath.append(
                        "dimm" + std::to_string(instanceID) + iter->second);
                    dvfsTempPath = std::string{OCC_SENSORS_ROOT} + "/temperature/" +
                                    dimmDVFSSensorName.at(fruTypeValue);
                }
            }
        }
        else if (type == OCC_CPU_TEMP_SENSOR_TYPE)
        {
            if (fruTypeValue == processorCore)
            {
                // The OCC reports small core temps, of which there are
                // two per big core.  All current P10 systems are in big
                // core mode, so use a big core name.
                uint16_t coreNum = instanceID / 2;
                uint16_t tempNum = instanceID % 2;
                sensorPath.append("proc" + std::to_string(occInstance) +
                                    "_core" + std::to_string(coreNum) + "_" +
                                    std::to_string(tempNum) + "_temp");
                dvfsTempPath =
                    std::string{OCC_SENSORS_ROOT} + "/temperature/proc" +
                    std::to_string(occInstance) + "_core_dvfs_temp";
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
}//end BuildTempDbusPaths

} // namespace occ
} // namespace open_power
