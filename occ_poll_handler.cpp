

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
constexpr auto fruTypeSuffix = "fru_type";
constexpr auto faultSuffix = "fault";
constexpr auto inputSuffix = "input";
constexpr auto maxSuffix = "max";


template <typename T>
T readFile(const std::string& path)
{
    std::ifstream ifs;
    ifs.exceptions(std::ifstream::failbit | std::ifstream::badbit |
                   std::ifstream::eofbit);
    T data;

    try
    {
        ifs.open(path);
        ifs >> data;
        ifs.close();
    }
    catch (const std::exception& e)
    {
        auto err = errno;
        throw std::system_error(err, std::generic_category());
    }

    return data;
}


//##############################################################################
// COMMON Functions.
//##############################################################################
OccPollHandler::OccPollHandler(Status& status) :
      statusObject(status),
      occInstanceID(status.getOccInstanceID())
{

}

std::optional<std::string> OccPollHandler::getPowerLabelFunctionID(
    const std::string& value)
{
    // If the value is "system", then the FunctionID is "system".
    if (value == "system")
    {
        return value;
    }

    // If the value is not "system", then the label value have 3 numbers, of
    // which we only care about the middle one:
    // <sensor id>_<function id>_<apss channel>
    // eg: The value is "0_10_5" , then the FunctionID is "10".
    if (value.find("_") == std::string::npos)
    {
        return std::nullopt;
    }

    auto powerLabelValue = value.substr((value.find("_") + 1));

    if (powerLabelValue.find("_") == std::string::npos)
    {
        return std::nullopt;
    }

    return powerLabelValue.substr(0, powerLabelValue.find("_"));
}//end getPowerLabelFunctionID

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

//##############################################################################
// OCC-CONTROL POLL Functions.
//##############################################################################
#ifdef ENABLE_APP_POLL_SUPPORT


void OccPollHandler::sendOccPollCmd()
{
    // New Poll data, clear out storage vector.
    if (!PollRspData.empty())
    {
        PollRspData.clear();
    }

    OccCommand occCmd(occInstanceID, (fs::path(OCC_CONTROL_ROOT) /
                        (std::string(OCC_NAME) + std::to_string(occInstanceID)))
                            .c_str());

    std::vector<std::uint8_t> cmd = {0x00, 0x00, 0x01, 0x20};


    occCmd.send(cmd, PollRspData);

}//end OCCC sendOccPollCmd

bool OccPollHandler::pollReadStateStatus(unsigned int& state, int& lastOccReadStatus)
{
    bool stateWasRead = true;

    lastOccReadStatus = 0;
    state = PollRspStatus;

    return stateWasRead;

}//end OCCC pollReadStateStatus

void OccPollHandler::HandlePollAction()
{
    sendOccPollCmd();

    if (PollRspData.size() >= OCC_RSP_HDR_LENGTH)
    {
        uint16_t index = 0;
        const uint8_t sizeSensorLabel = SensorLabel[0].size();

        // Add offset to jump over Data we do not need.
        index += 5;

        // Data format OCC_RSP_HDR_LENGTH 32 bytes
        //----------------------
        //  ExtStatus:    2 byte
        //  OccsPresent:  1 byte
        //  ConfigNeed:   1 byte
        //  State:        1 byte <-Index + 4
        PollRspStatus = PollRspData[index+4];
        //  Mode:         1 byte
        //  IPS:          1 byte
        //----------------------
        //  ElogID:       1 byte
        //  ElogAddr:     4 byte
        //  Elog Len:     2 byte
        //  Elog Source:  1 byte
        //  GPU config:   1 byte
        //  Code Level:   16 byte
        //----------------------
        index += 32;
        //----------------------
        //  Sensor Tag:   6 byte
        //  Sensor Blocks:1 byte
        //  Sensor Vers:  1 byte
        //----------------------
        // Total Header is 40 bytes. -> OCC_RSP_HDR_LENGTH

        std::vector<uint8_t> dataLabel = {0x53, 0x45, 0x4E, 0x53, 0x4F, 0x52}; // SENSOR TAG
        if (std::equal(PollRspData.begin()+index, PollRspData.begin()+index+6, dataLabel.begin()))
        {
            index += 6; //Add offset of SENSOR TAG

            uint8_t numBlocks = PollRspData[index++]; // SENSOR Blocks
            uint8_t pollVersion = PollRspData[index++]; // SENSOR Vers

            if (pollVersion == POLL_VERSION_FORMAT_1)
            {
                for (uint16_t i = 0; i < numBlocks; i++)
                {
                    if (std::equal(PollRspData.begin()+index, PollRspData.begin()+index+sizeSensorLabel, SensorLabel[TEMP].begin()))
                    {
                        index += sizeSensorLabel;
                        PushTempSensorsToDbus(index);
                    }
                    else if (std::equal(PollRspData.begin()+index, PollRspData.begin()+index+sizeSensorLabel, SensorLabel[FREQ].begin()))
                    {
                        index += sizeSensorLabel;
                        PushFreqSensorsToDbus(index);
                    }
                    else if (std::equal(PollRspData.begin()+index, PollRspData.begin()+index+sizeSensorLabel, SensorLabel[POWR].begin()))
                    {
                        index += sizeSensorLabel;
                        PushPowrSensorsToDbus(index);
                    }
                    else if (std::equal(PollRspData.begin()+index, PollRspData.begin()+index+sizeSensorLabel, SensorLabel[CAPS].begin()))
                    {
                        index += sizeSensorLabel;
                        PushCapsSensorsToDbus(index);
                    }
                    else if (std::equal(PollRspData.begin()+index, PollRspData.begin()+index+sizeSensorLabel, SensorLabel[EXTN].begin()))
                    {
                        index += sizeSensorLabel;
                        PushExtnSensorsToDbus(index);
                    }
                    else if (std::equal(PollRspData.begin()+index, PollRspData.begin()+index+sizeSensorLabel, SensorLabel[EXTT].begin()))
                    {
                        index += sizeSensorLabel;
                        PushExttSensorsToDbus(index);
                    }
                }
            }
            else
            {
                if (TraceOncePollHeader)
                {
                    lg2::error("OccPollHandler::HandlePollAction: unsuported poll format:{BYTE}",
                        "BYTE", lg2::hex, pollVersion);
                    TraceOncePollHeader = false;
                }
            }
        }
        else
        {
            if (TraceOncePollHeader)
            {
                lg2::error("OccPollHandler::HandlePollAction: unsuported header:");
                dump_hex(dataLabel);
                TraceOncePollHeader = false;
            }
        }
    }
}//end OCCC HandlePollAction

void OccPollHandler::PushTempSensorsToDbus(uint16_t& index )
{
    // Temp Sensor Header Data format for POLL
    //  SensorFormat:   1 byte
    //  bytesPerSensor: 1 byte
    //  NumberSensors:  1 byte
    //----------------------

    uint8_t SensorFormat = PollRspData[index++];
    uint8_t bytesPerSensor = PollRspData[index++];
    uint8_t NumberSensors = PollRspData[index++];

    if (SensorFormat == TEMP_SENSOR_FORMAT_16)
    {
        for (uint16_t i = 0; i < NumberSensors; i++)
        {
            // Temp Sensor Record Data format for POLL
            //  SensorID:     4 byte
            //  fruTypeValue: 1 byte
            //  tempValue:    1 byte
            //----------------------

            uint32_t SensorID = (static_cast<uint32_t>(PollRspData[index]) << 24) |
                    (static_cast<uint32_t>(PollRspData[index + 1]) << 16) |
                    (static_cast<uint32_t>(PollRspData[index + 2]) << 8) |
                    (static_cast<uint32_t>(PollRspData[index + 3]));

            auto tempValue = PollRspData[index+5];

            uint32_t fruTypeValue = PollRspData[index+4];

            std::string sensorPath = "";
            std::string dvfsTempPath = "";
            // if Dbus sensor found , and good FruType and value not 0, then continue
            if ((BuildTempDbusPaths(sensorPath, dvfsTempPath, SensorID, fruTypeValue, occInstanceID)))
            {

                dbus::OccDBusSensors::getOccDBus().setValue(sensorPath, tempValue );
                dbus::OccDBusSensors::getOccDBus().setOperationalStatus(sensorPath, !std::isnan(tempValue));
                if (!dvfsTempPath.empty() && !dbus::OccDBusSensors::getOccDBus().hasDvfsTemp(dvfsTempPath))
                {
                    auto dvfsValue = PollRspData[index+5];
                    dbus::OccDBusSensors::getOccDBus().setDvfsTemp(dvfsTempPath, dvfsValue );
                }
                if (statusObject.existingSensors.find(sensorPath) == statusObject.existingSensors.end())
                {
                    dbus::OccDBusSensors::getOccDBus().setChassisAssociation(
                        sensorPath, {"all_sensors"});
                }
                statusObject.existingSensors[sensorPath] = occInstanceID;

            }
            index += bytesPerSensor;
        }
    }
    else
    {
        if (TraceOncePollSensor)
        {
            lg2::error("OccPollHandler::PushTempSensorsToDbus: unsuported format:{BYTE}",
                "BYTE", lg2::hex, SensorFormat);
            TraceOncePollSensor = false;
        }

        index += bytesPerSensor * NumberSensors;
    }


}//end OCCC PushTempSensorsToDbus

void OccPollHandler::PushFreqSensorsToDbus(uint16_t& index )
{
    // Freq Sensor Header Data format for POLL
    //  SensorFormat:   1 byte
    //  bytesPerSensor: 1 byte
    //  NumberSensors:  1 byte
    //----------------------

    uint8_t SensorFormat = PollRspData[index++];
    uint8_t bytesPerSensor = PollRspData[index++];
    uint8_t NumberSensors = PollRspData[index++];

    if (SensorFormat == FREQ_SENSOR_FORMAT)
    {
        for (uint16_t i = 0; i < NumberSensors; i++)
        {
            // Freq Sensor Record Data format for POLL
            //  SensorID:     4 byte
            //  Frequency:    2 byte
            //----------------------

            index += bytesPerSensor;
        }
    }
    else
    {
        if (TraceOncePollSensor)
        {
            lg2::error("OccPollHandler::PushFreqSensorsToDbus: unsuported format :{BYTE}",
                "BYTE", lg2::hex, SensorFormat);
            TraceOncePollSensor = false;
        }
        index += bytesPerSensor * NumberSensors;
    }
}//end OCCC PushFreqSensorsToDbus

void OccPollHandler::PushPowrSensorsToDbus(uint16_t& index )
{
    // Powr Sensor Header Data format for POLL
    //  SensorFormat:   1 byte
    //  bytesPerSensor: 1 byte
    //  NumberSensors:  1 byte
    //----------------------

    uint8_t SensorFormat = PollRspData[index++];
    uint8_t bytesPerSensor = PollRspData[index++];
    uint8_t NumberSensors = PollRspData[index++];

    if (SensorFormat == POWR_SENSOR_FORMAT)
    {
        for (uint16_t i = 0; i < NumberSensors; i++)
        {
            // Powr Sensor Record Data format for POLL
            //  SensorID:      4 byte
            //  functionalID:  1 byte
            //  ApsChannel:    2 byte
            //  UpdateTag:     4 byte
            //  Accumulator:   8 byte
            //  SensorCurrent: 2 byte
            //----------------------

            uint8_t functionalID = (static_cast<uint8_t>(PollRspData[index + 4]));
            uint16_t SensorCurrent = (static_cast<uint32_t>(PollRspData[index + 20]) << 8) |
                    (static_cast<uint32_t>(PollRspData[index + 21]));

            std::string sensorPath = OCC_SENSORS_ROOT + std::string("/power/");

            std::stringstream ss;
            ss << static_cast<int>(functionalID);
            std::string functionID = ss.str();

            auto iter = powerSensorName.find(functionID);
            if (iter != powerSensorName.end())
            {
                sensorPath.append(iter->second);

                dbus::OccDBusSensors::getOccDBus().setUnit(sensorPath, "xyz.openbmc_project.Sensor.Value.Unit.Watts");
                dbus::OccDBusSensors::getOccDBus().setValue(sensorPath, SensorCurrent);
                dbus::OccDBusSensors::getOccDBus().setOperationalStatus(sensorPath, true);
                if (statusObject.existingSensors.find(sensorPath) == statusObject.existingSensors.end())
                {
                    dbus::OccDBusSensors::getOccDBus().setChassisAssociation(
                        sensorPath, {"all_sensors"});
                }
                statusObject.existingSensors[sensorPath] = occInstanceID;
            }
            else
            {
                if ((functionalID != 0) && (TraceOncePwrFuncId))
                {
                    lg2::error("OccPollHandler::PushPowrSensorsToDbus: functionID({FXID}) unsuported power sensor name", "FXID", functionID);
                    TraceOncePwrFuncId = false;
                }
            }
            index += bytesPerSensor;
        }
    }
    else
    {
        if (TraceOncePollSensor)
        {
            lg2::error("OccPollHandler::PushPowrSensorsToDbus: unsuported format:{BYTE}",
                "BYTE", lg2::hex, SensorFormat);
            TraceOncePollSensor = false;
        }

        index += bytesPerSensor * NumberSensors;
    }

}//end OCCC PushPowrSensorsToDbus

void OccPollHandler::PushCapsSensorsToDbus(uint16_t& index )
{
    // Caps Sensor Header Data format for POLL
    //  SensorFormat:   1 byte
    //  bytesPerSensor: 1 byte
    //  NumberSensors:  1 byte
    //----------------------

    uint8_t SensorFormat = PollRspData[index++];
    uint8_t bytesPerSensor = PollRspData[index++];
    uint8_t NumberSensors = PollRspData[index++];

    if (SensorFormat == CAPS_SENSOR_FORMAT)
    {
        for (uint16_t i = 0; i < NumberSensors; i++)
        {
            // Caps Sensor Record Data format for POLL
            //  SensorCap:       2 byte
            //  PollRspMaxPower: 2 byte
            //  NCap:            2 byte
            //  MaxCap:          2 byte
            //  HardMinCap:      2 byte
            //  SoftMinCap:      2 byte
            //  UserCap:         2 byte
            //  UserSourse:      1 byte
            //----------------------

            PollRspMaxPower = (static_cast<uint16_t>(PollRspData[index + 2]) << 8) |
                    (static_cast<uint16_t>(PollRspData[index + 3]));

            std::string sensorPath = OCC_SENSORS_ROOT + std::string("/power/");
            sensorPath.append("total_power");

            dbus::OccDBusSensors::getOccDBus().setUnit(sensorPath, "xyz.openbmc_project.Sensor.Value.Unit.Watts");
            dbus::OccDBusSensors::getOccDBus().setValue(sensorPath, PollRspMaxPower);
            dbus::OccDBusSensors::getOccDBus().setOperationalStatus(sensorPath, true);
            if (statusObject.existingSensors.find(sensorPath) == statusObject.existingSensors.end())
            {
                dbus::OccDBusSensors::getOccDBus().setPurpose(sensorPath,"xyz.openbmc_project.Sensor.Purpose.SensorPurpose.TotalPower");
                dbus::OccDBusSensors::getOccDBus().setChassisAssociation(
                    sensorPath, {"all_sensors"});
            }
            statusObject.existingSensors[sensorPath] = occInstanceID;


            PollRspHardMin = (static_cast<uint32_t>(PollRspData[index + 8]) << 8) |
                    (static_cast<uint32_t>(PollRspData[index + 9]));

            PollRspSoftMin = (static_cast<uint32_t>(PollRspData[index + 10]) << 8) |
                    (static_cast<uint32_t>(PollRspData[index + 11]));

            index += bytesPerSensor;
        }
    }
    else
    {
        if (TraceOncePollSensor)
        {
            lg2::error("OccPollHandler::PushCapsSensorFormat: unsuported format :{BYTE}",
                "BYTE", lg2::hex, SensorFormat);
            TraceOncePollSensor = false;
        }

            index += bytesPerSensor * NumberSensors;
    }



}//end OCCC PushCapsSensorsToDbus

void OccPollHandler::PushExtnSensorsToDbus(uint16_t& index )
{
    // Extn Sensor Header Data format for POLL
    //  SensorFormat: 1 byte
    //  bytesPerSensor: 1 byte
    //  NumberSensors: 1 byte
    //----------------------
    uint8_t SensorFormat = PollRspData[index++];
    uint8_t bytesPerSensor = PollRspData[index++];
    uint8_t NumberSensors = PollRspData[index++];

    if (SensorFormat == EXTN_SENSOR_FORMAT)
    {
        for (uint16_t i = 0; i < NumberSensors; i++)
        {
            // Extn Sensor Record Data format for POLL
            //  SensorID:  4 byte
            //  Flags:     1 byte
            //  Reserved:  1 byte
            //  Value:     6 byte
            //                 PWRM and PWRP last 2 bytes.
            //----------------------

            const uint32_t SensorName = (static_cast<uint32_t>(PollRspData[index]) << 24) |
                    (static_cast<uint32_t>(PollRspData[index + 1]) << 16) |
                    (static_cast<uint32_t>(PollRspData[index + 2]) << 8) |
                    (static_cast<uint32_t>(PollRspData[index + 3]));

            bool push_power_to_dbus = false;
            std::string sensorPath = OCC_SENSORS_ROOT;
            uint32_t SensorValue = 0;

            switch (SensorName)
            {
                case EXTN_LABEL_FMIN: break;
                case EXTN_LABEL_FDIS: break;
                case EXTN_LABEL_FBAS: break;
                case EXTN_LABEL_FUT:  break;
                case EXTN_LABEL_FMAX: break;
                case EXTN_LABEL_CLIP: break;
                case EXTN_LABEL_MODE: break;
                case EXTN_LABEL_WOFC: break;
                case EXTN_LABEL_WOFI: break;
                case EXTN_LABEL_PWRM:
                    push_power_to_dbus = true;
                    sensorPath.append("/power/chiplet" + std::to_string(occInstanceID) + "_mem_power");
                    break;
                case EXTN_LABEL_PWRP:
                    push_power_to_dbus = true;
                    sensorPath.append("/power/chiplet" + std::to_string(occInstanceID) + "_power");
                    break;
                case EXTN_LABEL_ERRH: break;
                default:
                    lg2::error("OccPollHandler::PushExtnSensorsToDbus: EXTN label name FAILED:{BYTE}",
                        "BYTE", lg2::hex, SensorName);
                    break;
            }

            if (push_power_to_dbus == true)
            {
                // For Power field, Convert last 4 bytes into number value
                SensorValue = (static_cast<uint32_t>(PollRspData[index + 10]) << 8) |
                    (static_cast<uint32_t>(PollRspData[index + 11]));

                // Convert output/DC power to input/AC power in Watts (round up)
                uint16_t extnSensorValue =
                    std::round(((SensorValue / (PS_DERATING_FACTOR / 100.0))));

                dbus::OccDBusSensors::getOccDBus().setUnit(sensorPath, "xyz.openbmc_project.Sensor.Value.Unit.Watts");
                dbus::OccDBusSensors::getOccDBus().setValue(sensorPath, extnSensorValue );
                dbus::OccDBusSensors::getOccDBus().setOperationalStatus(sensorPath, !std::isnan(extnSensorValue));
                if (statusObject.existingSensors.find(sensorPath) == statusObject.existingSensors.end())
                {
                    dbus::OccDBusSensors::getOccDBus().setChassisAssociation(
                        sensorPath, {"all_sensors"});
                }
                statusObject.existingSensors[sensorPath] = occInstanceID;
            }
            index += bytesPerSensor;
        }

    }
    else
    {
        if (TraceOncePollSensor)
        {
            lg2::error("OccPollHandler::PushExtnSensorsToDbus: unsuported format:{BYTE}",
                "BYTE", lg2::hex, SensorFormat);
            TraceOncePollSensor = false;
        }

        index += bytesPerSensor * NumberSensors;

    }


}//end OCCC PushExtnSensorsToDbus

void OccPollHandler::PushExttSensorsToDbus(uint16_t& index )
{
    // Extn Sensor Header Data format for POLL
    //  SensorFormat: 1 byte
    //  bytesPerSensor: 1 byte
    //  NumberSensors: 1 byte
    //----------------------

    uint8_t SensorFormat = PollRspData[index++];
    uint8_t bytesPerSensor = PollRspData[index++];
    uint8_t NumberSensors = PollRspData[index++];


    if (TraceOncePollSensor)
    {
        lg2::error("OccPollHandler::PushExttSensorsToDbus: unsuported format:{BYTE}",
            "BYTE", lg2::hex, SensorFormat);
        TraceOncePollSensor = false;
    }

    //Temp to jump over section until format data put in.
    index += bytesPerSensor * NumberSensors;

    // Extt PROPOSED Sensor Record Data format for POLL
    //  SensorID:  4 byte
    //  FruType:   1 byte
    //  Value:     1 byte
    //----------------------

}//end OCCC PushExttSensorsToDbus

bool OccPollHandler::pollReadPcapBounds(uint32_t& capSoftMin, uint32_t& capHardMin, uint32_t& capMax)
{
    bool parmsChanged = true;

    HandlePollAction();

    capSoftMin = PollRspSoftMin;
    capHardMin = PollRspHardMin;
    capMax = PollRspMaxPower;

    return parmsChanged;
} //end OCCC pollReadPcapBounds


#else
//##############################################################################
// KERNEL POLL Functions.
//##############################################################################
bool OccPollHandler::pollReadStateStatus(unsigned int& state, int& lastOccReadStatus)
{
    bool stateWasRead = false;

    const fs::path filename =
    fs::path(DEV_PATH) /
    fs::path(sysfsName + "." + std::to_string(occInstanceID + 1)) / "occ_state";

    std::ifstream file;

    // open file.
    file.open(filename, std::ios::in);
    const int openErrno = errno;

    // File is open and state can be used.
    if (file.is_open() && file.good())
    {
        stateWasRead = true;
        file >> state;
        // Read the error code (if any) to check status of the read
        std::ios_base::iostate readState = file.rdstate();
        if (readState)
        {
            // There was a failure reading the file
            if (lastOccReadStatus != -1)
            {
                // Trace error bits
                std::string errorBits = "";
                if (readState & std::ios_base::eofbit)
                {
                    errorBits += " EOF";
                }
                if (readState & std::ios_base::failbit)
                {
                    errorBits += " failbit";
                }
                if (readState & std::ios_base::badbit)
                {
                    errorBits += " badbit";
                }
                lg2::error(
                    "readOccState: Failed to read OCC{INST} state: Read error on I/O operation - {ERROR}",
                    "INST", occInstanceID, "ERROR", errorBits);
                lastOccReadStatus = -1;
            }
            stateWasRead = false;
        }

        // If not able to read, OCC may be offline
        if ((!stateWasRead) && (openErrno != lastOccReadStatus))
        {
            lg2::error(
                "OccPollHandler::pollReadStateStatus: open/read failed trying to read OCC{INST} state (open errno={ERROR})",
                "INST", occInstanceID, "ERROR", openErrno);
            lastOccReadStatus = openErrno;
        }

    }

    file.close();


    return stateWasRead;
}//end Kernel pollReadStateStatus

void OccPollHandler::HandlePollAction()
{
    static bool tracedError[8] = {0};
    const fs::path sensorPath = statusObject.getHwmonPath();


    if (fs::exists(sensorPath))
    {
        // Read temperature sensors and push to dbus
        pushTempSensorsToDbus(sensorPath, occInstanceID);

        // Read Extended sensors and push to dbus
        pushExtnSensorsToDbus(sensorPath, occInstanceID);

        if (statusObject.isMasterOcc())
        {
            // Read power sensors and push to dbus
            pushPowrSensorsToDbus(sensorPath, occInstanceID);

        }
        tracedError[occInstanceID] = false;

    }
    else
    {
        if (!tracedError[occInstanceID])
        {
            lg2::error(
                "OccPollHandler::getSensorValues: OCC{INST} sensor path missing: {PATH}",
                "INST", occInstanceID, "PATH", sensorPath);
            tracedError[occInstanceID] = true;
        }
    }

    return;
}//end Kernel HandlePollAction

bool OccPollHandler::pollReadPcapBounds(uint32_t& capSoftMin, uint32_t& capHardMin, uint32_t& capMax)
{

    // Build the hwmon string to write the power cap bounds
    fs::path minName = getPcapFilename(std::regex{"power\\d+_cap_min$"});
    fs::path softMinName =
        getPcapFilename(std::regex{"power\\d+_cap_min_soft$"});
    fs::path maxName = getPcapFilename(std::regex{"power\\d+_cap_max$"});

    // Read the power cap bounds from sysfs files (from OCC)
    uint64_t cap;
    bool parmsChanged = false;
    std::ifstream softMinFile(softMinName, std::ios::in);
    if (softMinFile)
    {
        softMinFile >> cap;
        softMinFile.close();
        // Convert to input/AC Power in Watts (round up)
        capSoftMin = ((cap / (PS_DERATING_FACTOR / 100.0) / 1000000) + 0.9);
        parmsChanged = true;
    }
    else
    {
        lg2::error(
            "updatePcapBounds: unable to find pcap_min_soft file: {FILE} (errno={ERR})",
            "FILE", pcapBasePathname, "ERR", errno);
    }

    std::ifstream minFile(minName, std::ios::in);
    if (minFile)
    {
        minFile >> cap;
        minFile.close();
        // Convert to input/AC Power in Watts (round up)
        capHardMin = ((cap / (PS_DERATING_FACTOR / 100.0) / 1000000) + 0.9);
        parmsChanged = true;
    }
    else
    {
        lg2::error(
            "updatePcapBounds: unable to find cap_min file: {FILE} (errno={ERR})",
            "FILE", pcapBasePathname, "ERR", errno);
    }

    std::ifstream maxFile(maxName, std::ios::in);
    if (maxFile)
    {
        maxFile >> cap;
        maxFile.close();
        // Convert to input/AC Power in Watts (truncate remainder)
        capMax = cap / (PS_DERATING_FACTOR / 100.0) / 1000000;
        parmsChanged = true;
    }
    else
    {
        lg2::error(
            "updatePcapBounds: unable to find cap_max file: {FILE} (errno={ERR})",
            "FILE", pcapBasePathname, "ERR", errno);
    }

    return parmsChanged;
} //end Kernel pollReadPcapBounds

fs::path OccPollHandler::getPcapFilename(const std::regex& expr)
{
    if (pcapBasePathname.empty())
    {
        pcapBasePathname = statusObject.getHwmonPath();
    }

    if (fs::exists(pcapBasePathname))
    {
        // Search for pcap file based on the supplied expr
        for (auto& file : fs::directory_iterator(pcapBasePathname))
        {
            if (std::regex_search(file.path().string(), expr))
            {
                // Found match
                return file;
            }
        }
    }
    else
    {
        lg2::error("Power Cap base filename not found: {FILE}", "FILE",
                   pcapBasePathname);
    }

    // return empty path
    return fs::path{};
} //end kernel getPcapFilename

void OccPollHandler::pushTempSensorsToDbus(const fs::path& path, uint32_t occInstance)
{
    // There may be more than one sensor with the same FRU type
    // and label so make two passes: the first to read the temps
    // from sysfs, and the second to put them on D-Bus after
    // resolving any conflicts.
    std::map<std::string, double> sensorData;

    std::regex expr{"temp\\d+_label$"}; // Example: temp5_label
    for (auto& file : fs::directory_iterator(path))
    {
        if (!std::regex_search(file.path().string(), expr))
        {
            continue;
        }

        uint32_t labelValue{0};

        try
        {
            labelValue = readFile<uint32_t>(file.path());
        }
        catch (const std::system_error& e)
        {
            lg2::debug(
                "pushTempSensorsToDbus: Failed reading {PATH}, errno = {ERROR}",
                "PATH", file.path().string(), "ERROR", e.code().value());
            continue;
        }

        const std::string& tempLabel = "label";
        const std::string filePathString = file.path().string().substr(
            0, file.path().string().length() - tempLabel.length());

        uint32_t fruTypeValue{0};
        try
        {
            fruTypeValue = readFile<uint32_t>(filePathString + fruTypeSuffix);
        }
        catch (const std::system_error& e)
        {
            lg2::debug(
                "pushTempSensorsToDbus: Failed reading {PATH}, errno = {ERROR}",
                "PATH", filePathString + fruTypeSuffix, "ERROR",
                e.code().value());
            continue;
        }


        std::string sensorPath = "";
        std::string dvfsTempPath = "";
        // if no Dbus sensor found then continue
        if ( !BuildTempDbusPaths(sensorPath, dvfsTempPath, labelValue, fruTypeValue, occInstance))
        {
            continue;
        }

        // The dvfs temp file only needs to be read once per chip per type.
        if (!dvfsTempPath.empty() &&
            !dbus::OccDBusSensors::getOccDBus().hasDvfsTemp(dvfsTempPath))
        {
            try
            {
                auto dvfsValue = readFile<double>(filePathString + maxSuffix);
                dbus::OccDBusSensors::getOccDBus().setDvfsTemp(dvfsTempPath, dvfsValue * std::pow(10, -3));
            }
            catch (const std::system_error& e)
            {
                lg2::debug(
                    "pushTempSensorsToDbus: Failed reading {PATH}, errno = {ERROR}",
                    "PATH", filePathString + maxSuffix, "ERROR",
                    e.code().value());
            }
        }

        uint32_t faultValue{0};
        try
        {
            faultValue = readFile<uint32_t>(filePathString + faultSuffix);
        }
        catch (const std::system_error& e)
        {
            lg2::debug(
                "pushTempSensorsToDbus: Failed reading {PATH}, errno = {ERROR}",
                "PATH", filePathString + faultSuffix, "ERROR",
                e.code().value());
            continue;
        }

        double tempValue{0};
        // NOTE: if OCC sends back 0xFF, kernal sets this fault value to 1.
        if (faultValue != 0)
        {
            tempValue = std::numeric_limits<double>::quiet_NaN();
        }
        else
        {
            // Read the temperature
            try
            {
                tempValue = readFile<double>(filePathString + inputSuffix);
            }
            catch (const std::system_error& e)
            {
                lg2::debug(
                    "pushTempSensorsToDbus: Failed reading {PATH}, errno = {ERROR}",
                    "PATH", filePathString + inputSuffix, "ERROR",
                    e.code().value());

                // if errno == EAGAIN(Resource temporarily unavailable) then set
                // temp to 0, to avoid using old temp, and affecting FAN
                // Control.
                if (e.code().value() == EAGAIN)
                {
                    tempValue = 0;
                }
                // else the errno would be something like
                //     EBADF(Bad file descriptor)
                // or ENOENT(No such file or directory)
                else
                {
                    continue;
                }
            }
        }

        // If this object path already has a value, only overwite
        // it if the previous one was an NaN or a smaller value.
        auto existing = sensorData.find(sensorPath);
        if (existing != sensorData.end())
        {
            // Multiple sensors found for this FRU type
            if ((std::isnan(existing->second) && (tempValue == 0)) ||
                ((existing->second == 0) && std::isnan(tempValue)))
            {
                // One of the redundant sensors has failed (0xFF/nan), and the
                // other sensor has no reading (0), so set the FRU to NaN to
                // force fan increase
                tempValue = std::numeric_limits<double>::quiet_NaN();
                existing->second = tempValue;
            }
            if (std::isnan(existing->second) || (tempValue > existing->second))
            {
                existing->second = tempValue;
            }
        }
        else
        {
            // First sensor for this FRU type
            sensorData[sensorPath] = tempValue;
        }
    }

    // Now publish the values on D-Bus.
    for (const auto& [objectPath, value] : sensorData)
    {

        dbus::OccDBusSensors::getOccDBus().setValue(objectPath, value * std::pow(10, -3));
        dbus::OccDBusSensors::getOccDBus().setOperationalStatus(objectPath, !std::isnan(value));
        if (statusObject.existingSensors.find(objectPath) == statusObject.existingSensors.end())
        {
            try
            {
                dbus::OccDBusSensors::getOccDBus().setChassisAssociation(
                    objectPath, {"all_sensors"});
            }
            catch(const std::exception& e)
            {
                lg2::error("OccPollHandler::pushTempSensorsToDbus:setChassisAssociation FAILED OCC:{INST} watch:{ERROR} PATH:{PATH}",
                    "INST", occInstanceID, "ERROR", e.what(), "PATH", objectPath );
            }
        }
        statusObject.existingSensors[objectPath] = occInstance;

    }

    return;
} //end kernel pushTempSensorsToDbus

void OccPollHandler::pushExtnSensorsToDbus(const fs::path& path, uint32_t occInstanceID)
{

    std::regex expr{"extn\\d+_label$"}; // Example: extn5_label
    for (auto& file : fs::directory_iterator(path))
    {
        if (!std::regex_search(file.path().string(), expr))
        {
            continue;
        }

        // Read in Label value of the sensor from file.
        std::string labelValue;
        uint32_t SensorName = 0;
        try
        {
            labelValue = readFile<std::string>(file.path());

            std::stringstream ssData;
            ssData << std::hex << labelValue.substr(labelValue.length() - 8);
            ssData >> SensorName;
        }
        catch (const std::system_error& e)
        {
            lg2::debug(
                "pushExtnSensorsToDbus:label Failed reading {PATH}, errno = {ERROR}",
                "PATH", file.path().string(), "ERROR", e.code().value());
            continue;
        }
        const std::string& tempLabel = "label";
        const std::string filePathString = file.path().string().substr(
            0, file.path().string().length() - tempLabel.length());

        std::string sensorPath = OCC_SENSORS_ROOT + std::string("/power/");

        // Labels of EXTN sections from OCC interface Document
        //     have different formats.
        if ((SensorName == EXTN_LABEL_PWRM) ||
            (SensorName == EXTN_LABEL_PWRP))
        {
            // Label indicating byte 5 and 6 is the current (mem,proc) power in
            //      Watts.

            // Build the dbus String for this chiplet power asset.
            if (SensorName == EXTN_LABEL_PWRP)
            {
                labelValue = "_power";
            }
            else // else EXTN_LABEL_PWRM_MEMORY_POWER
            {
                labelValue = "_mem_power";
            }
            sensorPath.append("chiplet" + std::to_string(occInstanceID) + labelValue);

            // Read in data value of the sensor from file.
            // Read in as string due to different format of data in sensors.
            std::string extnValue;
            try
            {
                extnValue = readFile<std::string>(filePathString + inputSuffix);
            }
            catch (const std::system_error& e)
            {
                lg2::debug(
                    "pushExtnSensorsToDbus:value Failed reading {PATH}, errno = {ERROR}",
                    "PATH", filePathString + inputSuffix, "ERROR",
                    e.code().value());
                continue;
            }

            // For Power field, Convert last 4 bytes of hex string into number
            //   value.
            std::stringstream ssData;
            ssData << std::hex << extnValue.substr(extnValue.length() - 4);
            uint16_t extnSensorValue;
            ssData >> extnSensorValue;

            // Convert output/DC power to input/AC power in Watts (round up)
            extnSensorValue =
                std::round(((extnSensorValue / (PS_DERATING_FACTOR / 100.0))));

            dbus::OccDBusSensors::getOccDBus().setUnit(sensorPath, "xyz.openbmc_project.Sensor.Value.Unit.Watts");
            dbus::OccDBusSensors::getOccDBus().setValue(sensorPath, extnSensorValue);
            dbus::OccDBusSensors::getOccDBus().setOperationalStatus(sensorPath, true);
            if (statusObject.existingSensors.find(sensorPath) == statusObject.existingSensors.end())
            {
                dbus::OccDBusSensors::getOccDBus().setChassisAssociation(
                    sensorPath, {"all_sensors"});
            }
            statusObject.existingSensors[sensorPath] = occInstanceID;

        } // End Extended Power Sensors.
    } // End For loop on files for Extended Sensors.

    return;
} //end kernel pushExtnSensorsToDbus

void OccPollHandler::pushPowrSensorsToDbus(const fs::path& path, uint32_t occInstanceID)
{

    std::regex expr{"power\\d+_label$"}; // Example: power5_label
    for (auto& file : fs::directory_iterator(path))
    {
        if (!std::regex_search(file.path().string(), expr))
        {
            continue;
        }

        std::string labelValue;
        try
        {
            labelValue = readFile<std::string>(file.path());
        }
        catch (const std::system_error& e)
        {
            lg2::debug(
                "pushPowrSensorsToDbus: Failed reading {PATH}, errno = {ERROR}",
                "PATH", file.path().string(), "ERROR", e.code().value());
            continue;
        }

        auto functionID = getPowerLabelFunctionID(labelValue);
        if (functionID == std::nullopt)
        {
            continue;
        }

        const std::string& tempLabel = "label";
        const std::string filePathString = file.path().string().substr(
            0, file.path().string().length() - tempLabel.length());

        std::string sensorPath = OCC_SENSORS_ROOT + std::string("/power/");

        auto iter = powerSensorName.find(*functionID);
        if (iter == powerSensorName.end())
        {
            continue;
        }
        sensorPath.append(iter->second);

        double tempValue{0};

        try
        {
            tempValue = readFile<double>(filePathString + inputSuffix);
        }
        catch (const std::system_error& e)
        {
            lg2::debug(
                "pushPowrSensorsToDbus: Failed reading {PATH}, errno = {ERROR}",
                "PATH", filePathString + inputSuffix, "ERROR",
                e.code().value());
            continue;
        }

        dbus::OccDBusSensors::getOccDBus().setUnit(sensorPath, "xyz.openbmc_project.Sensor.Value.Unit.Watts");
        dbus::OccDBusSensors::getOccDBus().setValue(sensorPath, tempValue * std::pow(10, -3) * std::pow(10, -3));
        dbus::OccDBusSensors::getOccDBus().setOperationalStatus(sensorPath, true);
        if (statusObject.existingSensors.find(sensorPath) == statusObject.existingSensors.end())
        {
            if (iter->second == "total_power")
            {
                dbus::OccDBusSensors::getOccDBus().setPurpose(sensorPath,"xyz.openbmc_project.Sensor.Purpose.SensorPurpose.TotalPower");
            }
            dbus::OccDBusSensors::getOccDBus().setChassisAssociation(
                sensorPath, {"all_sensors"});
        }
        statusObject.existingSensors[sensorPath] = occInstanceID;

    }

    return;
} //end kernel pushPowrSensorsToDbus

#endif
//##############################################################################
// END
//##############################################################################


} // namespace occ
} // namespace open_power
