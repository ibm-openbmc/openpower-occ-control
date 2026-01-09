#include "occ_poll_app_handler.hpp"

#include "occ_command.hpp"
#include "occ_dbus.hpp"
#include "occ_poll_handler.hpp"
#include "occ_status.hpp"

#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/lg2.hpp>

namespace open_power
{
namespace occ
{

using namespace phosphor::logging;
using namespace std::literals::chrono_literals;

inline uint32_t UINT32_GET(const uint8_t* i_ptr)
{
    return (((*(i_ptr + 0)) << 24) | ((*(i_ptr + 1)) << 16) |
            ((*(i_ptr + 2)) << 8) | (*(i_ptr + 3)));
}

OccPollAppHandler::OccPollAppHandler(Status& status, unsigned int instance) :
    statusObject(status), occInstanceID(instance)
{}

void OccPollAppHandler::sendOccPollCmd()
{
    // New Poll data, clear out storage vector.
    if (!PollRspData.empty())
    {
        PollRspData.clear();
    }

    OccCommand occCmd(occInstanceID,
                      (fs::path(OCC_CONTROL_ROOT) /
                       (std::string(OCC_NAME) + std::to_string(occInstanceID)))
                          .c_str());

    std::vector<std::uint8_t> cmd = {0x00, 0x00, 0x01, 0x20};

    occCmd.send(cmd, PollRspData);

} // end sendOccPollCmd

bool OccPollAppHandler::pollReadStateStatus(unsigned int& state,
                                            int& lastOccReadStatus)
{
    if (ValidPollRspData)
    {
        lastOccReadStatus = 0;
        state = PollRspStatus;
    }
    else
    {
        // In the case of getting status of all OCCs if status not valid read.
        HandlePollAction();

        if (ValidPollRspData)
        {
            lastOccReadStatus = 0;
            state = PollRspStatus;
        }
        else
        {
            lastOccReadStatus = 0;
            state = 0;
        }
    }

    return ValidPollRspData;

} // end pollReadStateStatus

void OccPollAppHandler::HandlePollAction()
{
    sendOccPollCmd();

    if (PollRspData.size() >= OCC_RSP_HDR_LENGTH)
    {
        uint16_t index = 0;

        // Add offset to jump over Data we do not need.
        index += 5;

        // Data format OCC_RSP_HDR_LENGTH 32 bytes
        //----------------------
        //  ExtStatus:    2 byte
        //  OccsPresent:  1 byte
        //  ConfigNeed:   1 byte
        //  State:        1 byte <-Index + 4
        PollRspStatus = PollRspData[index + 4];
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

        std::vector<uint8_t> dataLabel = {'S', 'E', 'N', 'S', 'O', 'R'};
        if (std::equal(PollRspData.begin() + index,
                       PollRspData.begin() + index + 6, dataLabel.begin()))
        {
            index += 6; // Add offset of SENSOR TAG
            ValidPollRspData = true;

            uint8_t numBlocks = PollRspData[index++]; // SENSOR Blocks
            uint8_t sensorBlockVersion =
                PollRspData[index++];                 // SENSOR Block Vers

            if (sensorBlockVersion == SENSOR_BLOCK_VERSION_1)
            {
                for (uint16_t i = 0; i < numBlocks; i++)
                {
                    unsigned char* rspPtr = &PollRspData[index];
                    if (std::equal(rspPtr, rspPtr + size_label,
                                   TEMP_label.begin()))
                    {
                        index += size_label;
                        PushTempSensorsToDbus(index);
                    }
                    else if (std::equal(rspPtr, rspPtr + size_label,
                                        FREQ_label.begin()))
                    {
                        index += size_label;
                        PushFreqSensorsToDbus(index);
                    }
                    else if (std::equal(rspPtr, rspPtr + size_label,
                                        POWR_label.begin()))
                    {
                        index += size_label;
                        PushPowrSensorsToDbus(index);
                    }
                    else if (std::equal(rspPtr, rspPtr + size_label,
                                        CAPS_label.begin()))
                    {
                        index += size_label;
                        PushCapsSensorsToDbus(index);
                    }
                    else if (std::equal(rspPtr, rspPtr + size_label,
                                        EXTN_label.begin()))
                    {
                        index += size_label;
                        PushExtnSensorsToDbus(index);
                    }
                    else if (std::equal(rspPtr, rspPtr + size_label,
                                        EXTT_label.begin()))
                    {
                        index += size_label;
                        PushExttSensorsToDbus(index);
                    }
                }
            }
            else
            {
                if (TraceOncePollHeader)
                {
                    lg2::error(
                        "OccPollAppHandler::HandlePollAction: unsuported sensor block version:{BYTE}",
                        "BYTE", lg2::hex, sensorBlockVersion);
                    TraceOncePollHeader = false;
                }
            }
        }
        else
        {
            if (TraceOncePollHeader)
            {
                lg2::error(
                    "OccPollAppHandler::HandlePollAction: unsuported header:");
                dump_hex(dataLabel);
                TraceOncePollHeader = false;
            }
            ValidPollRspData = false;
        }
    }
    else
    {
        lg2::error(
            "OccPollAppHandler::HandlePollAction: invalid header size:{VALUE}",
            "VALUE", PollRspData.size());
        ValidPollRspData = false;
    }
} // end HandlePollAction

void OccPollAppHandler::PushTempSensorsToDbus(uint16_t& index)
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
        // Find hottest temperature for each fru type
        struct fru_temp_t
        {
            uint32_t sensorId;
            uint8_t temp;
            uint8_t dvfsTemp;
        };
        struct fru_temp_t hottest[MAX_FRU_TYPES] = {{0, 0, 0}};
        for (uint16_t i = 0; i < NumberSensors; i++)
        {
            // Temp Sensor Record Data format for POLL
            //  SensorID:     4 byte
            //  fruTypeValue: 1 byte
            //  tempValue:    1 byte
            //  throttleTemp: 1 byte
            //  errorTemp:    1 byte
            //----------------------
            const uint32_t SensorID = UINT32_GET(&PollRspData[index]);
            const uint8_t fruTypeValue = PollRspData[index + 4];
            if (fruTypeValue < MAX_FRU_TYPES)
            {
                const uint8_t tempValue = PollRspData[index + 5];
                const uint8_t dvfsValue = PollRspData[index + 6];
                if (tempValue > hottest[fruTypeValue].temp)
                {
                    hottest[fruTypeValue].sensorId = SensorID;
                    hottest[fruTypeValue].temp = tempValue;
                    hottest[fruTypeValue].dvfsTemp = dvfsValue;
                }
            }
            else if (fruTypeValue != FRU_UNAVAILABLE)
            {
                if (TraceOncePollHeader)
                {
                    lg2::error(
                        "PushTempSensorsToDbus: invalid FRU type in POLL response:{FRU}",
                        "FRU", fruTypeValue);
                    TraceOncePollHeader = false;
                }
            }
            index += bytesPerSensor;
        }

        for (size_t fruType = 0; fruType < MAX_FRU_TYPES; fruType++)
        {
            if (hottest[fruType].sensorId != 0)
            {
                std::string sensorPath = "";
                std::string dvfsTempPath = "";
                // if Dbus sensor found and good FruType and value not 0, then
                // update hottest temp on dbus
                if ((BuildTempDbusPaths(sensorPath, dvfsTempPath,
                                        hottest[fruType].sensorId, fruType,
                                        occInstanceID, true)))
                {
                    dbus::OccDBusSensors::getOccDBus().setValue(
                        sensorPath, hottest[fruType].temp);
                    dbus::OccDBusSensors::getOccDBus().setOperationalStatus(
                        sensorPath, !std::isnan(hottest[fruType].temp));
                    if (!dvfsTempPath.empty() &&
                        !dbus::OccDBusSensors::getOccDBus().hasDvfsTemp(
                            dvfsTempPath))
                    {
                        dbus::OccDBusSensors::getOccDBus().setDvfsTemp(
                            dvfsTempPath, hottest[fruType].dvfsTemp);
                    }
                    // Chassis Association will be set in EXTT section (since
                    // that contains all sensors)
                }
            }
        }
    }
    else
    {
        if (TraceOncePollSensor)
        {
            lg2::error(
                "OccPollAppHandler::PushTempSensorsToDbus: unsuported format:{BYTE}",
                "BYTE", lg2::hex, SensorFormat);
            TraceOncePollSensor = false;
        }

        index += bytesPerSensor * NumberSensors;
    }

} // end PushTempSensorsToDbus

void OccPollAppHandler::PushFreqSensorsToDbus(uint16_t& index)
{
    // Freq Sensor Header Data format for POLL
    //  SensorFormat:   1 byte
    //  bytesPerSensor: 1 byte
    //  NumberSensors:  1 byte
    //----------------------

    uint8_t SensorFormat = PollRspData[index++];
    uint8_t bytesPerSensor = PollRspData[index++];
    uint8_t NumberSensors = PollRspData[index++];

    if (SensorFormat == FREQ_SENSOR_FORMAT_2)
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
            lg2::error(
                "OccPollAppHandler::PushFreqSensorsToDbus: unsuported format :{BYTE}",
                "BYTE", lg2::hex, SensorFormat);
            TraceOncePollSensor = false;
        }
        index += bytesPerSensor * NumberSensors;
    }
} // end PushFreqSensorsToDbus

void OccPollAppHandler::PushPowrSensorsToDbus(uint16_t& index)
{
    // Powr Sensor Header Data format for POLL
    //  SensorFormat:   1 byte
    //  bytesPerSensor: 1 byte
    //  NumberSensors:  1 byte
    //----------------------

    uint8_t SensorFormat = PollRspData[index++];
    uint8_t bytesPerSensor = PollRspData[index++];
    uint8_t NumberSensors = PollRspData[index++];

    if (SensorFormat == POWR_SENSOR_FORMAT_2)
    {
        for (uint16_t i = 0; i < NumberSensors; i++)
        {
            // Powr Sensor Record Data format for POLL
            //  SensorID:      4 byte
            //  functionalID:  1 byte
            //  ApsChannel:    2 byte
            //  UpdateTag:     4 byte
            //  Accumulator:   8 byte
            //  SensorValue:   2 byte
            //----------------------
            uint8_t functionalID = (PollRspData[index + 4]);
            std::string functionID = std::to_string(functionalID);

            uint16_t SensorValue = ((PollRspData[index + 20]) << 8) |
                                   (PollRspData[index + 21]);

            std::string sensorPath = OCC_SENSORS_ROOT + std::string("/power/");

            auto iter = powerSensorName.find(functionID);
            if (iter != powerSensorName.end())
            {
                sensorPath.append(iter->second);

                dbus::OccDBusSensors::getOccDBus().setUnit(
                    sensorPath, "xyz.openbmc_project.Sensor.Value.Unit.Watts");
                dbus::OccDBusSensors::getOccDBus().setValue(sensorPath,
                                                            SensorValue);
                dbus::OccDBusSensors::getOccDBus().setOperationalStatus(
                    sensorPath, true);
                if (statusObject.existingSensors.find(sensorPath) ==
                    statusObject.existingSensors.end())
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
                    lg2::error(
                        "OccPollAppHandler::PushPowrSensorsToDbus: functionID({FXID}) unsuported power sensor name",
                        "FXID", functionID);
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
            lg2::error(
                "OccPollAppHandler::PushPowrSensorsToDbus: unsuported format:{BYTE}",
                "BYTE", lg2::hex, SensorFormat);
            TraceOncePollSensor = false;
        }

        index += bytesPerSensor * NumberSensors;
    }

} // end PushPowrSensorsToDbus

void OccPollAppHandler::PushCapsSensorsToDbus(uint16_t& index)
{
    // Caps Sensor Header Data format for POLL
    //  SensorFormat:   1 byte
    //  bytesPerSensor: 1 byte
    //  NumberSensors:  1 byte
    //----------------------

    uint8_t SensorFormat = PollRspData[index++];
    uint8_t bytesPerSensor = PollRspData[index++];
    uint8_t NumberSensors = PollRspData[index++];

    if (SensorFormat == CAPS_SENSOR_FORMAT_3)
    {
        for (uint16_t i = 0; i < NumberSensors; i++)
        {
            // Caps Sensor Record Data format for POLL
            //  CurrentPowerCap:     2 byte
            //  CurrentPowerReading: 2 byte
            //  NCap:                2 byte
            //  MaxCap:              2 byte
            //  HardMinCap:          2 byte
            //  SoftMinCap:          2 byte
            //  UserCap:             2 byte
            //  UserSourse:          1 byte
            //-----------------------------
            uint16_t CurrentPowerReading =
                ((PollRspData[index + 2]) << 8) | (PollRspData[index + 3]);

            std::string sensorPath = OCC_SENSORS_ROOT + std::string("/power/");
            sensorPath.append("total_power");

            dbus::OccDBusSensors::getOccDBus().setUnit(
                sensorPath, "xyz.openbmc_project.Sensor.Value.Unit.Watts");
            dbus::OccDBusSensors::getOccDBus().setValue(sensorPath,
                                                        CurrentPowerReading);
            dbus::OccDBusSensors::getOccDBus().setOperationalStatus(
                sensorPath, true);
            if (statusObject.existingSensors.find(sensorPath) ==
                statusObject.existingSensors.end())
            {
                dbus::OccDBusSensors::getOccDBus().setPurpose(
                    sensorPath,
                    "xyz.openbmc_project.Sensor.Purpose.SensorPurpose.TotalPower");
                dbus::OccDBusSensors::getOccDBus().setChassisAssociation(
                    sensorPath, {"all_sensors"});
            }
            statusObject.existingSensors[sensorPath] = occInstanceID;

            PollRspMaxCap = ((PollRspData[index + 6]) << 8) |
                            (PollRspData[index + 7]);

            PollRspHardMin = ((PollRspData[index + 8]) << 8) |
                             (PollRspData[index + 9]);

            PollRspSoftMin = ((PollRspData[index + 10]) << 8) |
                             (PollRspData[index + 11]);

            index += bytesPerSensor;
        }
    }
    else
    {
        if (TraceOncePollSensor)
        {
            lg2::error(
                "OccPollAppHandler::PushCapsSensorFormat: unsuported format :{BYTE}",
                "BYTE", lg2::hex, SensorFormat);
            TraceOncePollSensor = false;
        }

        index += bytesPerSensor * NumberSensors;
    }

} // end PushCapsSensorsToDbus

void OccPollAppHandler::PushExtnSensorsToDbus(uint16_t& index)
{
    // Extn Sensor Header Data format for POLL
    //  SensorFormat: 1 byte
    //  bytesPerSensor: 1 byte
    //  NumberSensors: 1 byte
    //----------------------
    uint8_t SensorFormat = PollRspData[index++];
    uint8_t bytesPerSensor = PollRspData[index++];
    uint8_t NumberSensors = PollRspData[index++];

    if (SensorFormat == EXTN_SENSOR_FORMAT_1)
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
            const uint32_t SensorID = UINT32_GET(&PollRspData[index]);

            bool push_power_to_dbus = false;
            std::string sensorPath = OCC_SENSORS_ROOT;
            uint32_t SensorValue = 0;

            switch (SensorID)
            {
                case EXTN_LABEL_FMIN:
                    break;
                case EXTN_LABEL_FDIS:
                    break;
                case EXTN_LABEL_FBAS:
                    break;
                case EXTN_LABEL_FUT:
                    break;
                case EXTN_LABEL_FMAX:
                    break;
                case EXTN_LABEL_CLIP:
                    break;
                case EXTN_LABEL_MODE:
                    break;
                case EXTN_LABEL_WOFC:
                    break;
                case EXTN_LABEL_WOFI:
                    break;
                case EXTN_LABEL_PWRM:
                    push_power_to_dbus = true;
                    sensorPath.append(
                        "/power/chiplet" + std::to_string(occInstanceID) +
                        "_mem_power");
                    break;
                case EXTN_LABEL_PWRP:
                    push_power_to_dbus = true;
                    sensorPath.append("/power/chiplet" +
                                      std::to_string(occInstanceID) + "_power");
                    break;
                case EXTN_LABEL_ERRH:
                    break;
                default:
                    break;
            }

            if (push_power_to_dbus == true)
            {
                // For Power field, Convert last 4 bytes into number value
                SensorValue = ((PollRspData[index + 10]) << 8) |
                              (PollRspData[index + 11]);

                // Convert output/DC power to input/AC power in Watts (round up)
                uint16_t extnSensorValue =
                    std::round(((SensorValue / (PS_DERATING_FACTOR / 100.0))));

                dbus::OccDBusSensors::getOccDBus().setUnit(
                    sensorPath, "xyz.openbmc_project.Sensor.Value.Unit.Watts");
                dbus::OccDBusSensors::getOccDBus().setValue(sensorPath,
                                                            extnSensorValue);
                dbus::OccDBusSensors::getOccDBus().setOperationalStatus(
                    sensorPath, !std::isnan(extnSensorValue));
                if (statusObject.existingSensors.find(sensorPath) ==
                    statusObject.existingSensors.end())
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
            lg2::error(
                "OccPollAppHandler::PushExtnSensorsToDbus: unsuported format:{BYTE}",
                "BYTE", lg2::hex, SensorFormat);
            TraceOncePollSensor = false;
        }

        index += bytesPerSensor * NumberSensors;
    }

} // end PushExtnSensorsToDbus

void OccPollAppHandler::PushExttSensorsToDbus(uint16_t& index)
{
    // Extn Sensor Header Data format for POLL
    //  SensorFormat: 1 byte
    //  bytesPerSensor: 1 byte
    //  NumberSensors: 1 byte
    //----------------------

    uint8_t SensorFormat = PollRspData[index++];
    uint8_t bytesPerSensor = PollRspData[index++];
    uint8_t NumberSensors = PollRspData[index++];

    if (SensorFormat == 16)
    {
        for (uint16_t i = 0; i < NumberSensors; i++)
        {
            // Extended Temp Sensor Record Data format for POLL
            //  SensorID:  4 byte
            //  FruType:   1 byte
            //  Value:     1 byte
            //----------------------
            const uint32_t SensorID = UINT32_GET(&PollRspData[index]);
            const auto fruType = PollRspData[index + 4];
            const auto temperature = PollRspData[index + 5];

            std::string sensorPath = "";
            std::string dvfsTempPath = "";
            // if Dbus sensor found, and good FruType and value not 0, then
            // continue
            if ((BuildTempDbusPaths(sensorPath, dvfsTempPath, SensorID, fruType,
                                    occInstanceID)))
            {
                dbus::OccDBusSensors::getOccDBus().setValue(sensorPath,
                                                            temperature);
                dbus::OccDBusSensors::getOccDBus().setOperationalStatus(
                    sensorPath, !std::isnan(temperature));
                if (statusObject.existingSensors.find(sensorPath) ==
                    statusObject.existingSensors.end())
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
            lg2::error(
                "OccPollAppHandler::PushExttSensorsToDbus: unsuported format:{BYTE}",
                "BYTE", lg2::hex, SensorFormat);
            TraceOncePollSensor = false;
        }
        index += bytesPerSensor * NumberSensors;
    }

} // end PushExttSensorsToDbus

// Called once when the master OCC goes active. This means parms always changed.
bool OccPollAppHandler::pollReadPcapBounds(
    uint32_t& capSoftMin, uint32_t& capHardMin, uint32_t& capMax)
{
    bool parmsChanged = true;

    HandlePollAction();

    capSoftMin = PollRspSoftMin;
    capHardMin = PollRspHardMin;
    capMax = PollRspMaxCap;

    return parmsChanged;
} // end pollReadPcapBounds

} // namespace occ
} // namespace open_power
