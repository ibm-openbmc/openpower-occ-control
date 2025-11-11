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



OccPollAppHandler::OccPollAppHandler(Status& status, unsigned int instance) :
      statusObject(status),
      occInstanceID(instance)
{

}


void OccPollAppHandler::sendOccPollCmd()
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

}//end sendOccPollCmd

bool OccPollAppHandler::pollReadStateStatus(unsigned int& state, int& lastOccReadStatus)
{
    bool stateWasRead = true;

    lastOccReadStatus = 0;
    state = PollRspStatus;

    return stateWasRead;

}//end pollReadStateStatus

void OccPollAppHandler::HandlePollAction()
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
                    lg2::error("OccPollAppHandler::HandlePollAction: unsuported poll format:{BYTE}",
                        "BYTE", lg2::hex, pollVersion);
                    TraceOncePollHeader = false;
                }
            }
        }
        else
        {
            if (TraceOncePollHeader)
            {
                lg2::error("OccPollAppHandler::HandlePollAction: unsuported header:");
                dump_hex(dataLabel);
                TraceOncePollHeader = false;
            }
        }
    }
}//end HandlePollAction

void OccPollAppHandler::PushTempSensorsToDbus(uint16_t& index )
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
            lg2::error("OccPollAppHandler::PushTempSensorsToDbus: unsuported format:{BYTE}",
                "BYTE", lg2::hex, SensorFormat);
            TraceOncePollSensor = false;
        }

        index += bytesPerSensor * NumberSensors;
    }


}//end PushTempSensorsToDbus

void OccPollAppHandler::PushFreqSensorsToDbus(uint16_t& index )
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
            lg2::error("OccPollAppHandler::PushFreqSensorsToDbus: unsuported format :{BYTE}",
                "BYTE", lg2::hex, SensorFormat);
            TraceOncePollSensor = false;
        }
        index += bytesPerSensor * NumberSensors;
    }
}//end PushFreqSensorsToDbus

void OccPollAppHandler::PushPowrSensorsToDbus(uint16_t& index )
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
                    lg2::error("OccPollAppHandler::PushPowrSensorsToDbus: functionID({FXID}) unsuported power sensor name", "FXID", functionID);
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
            lg2::error("OccPollAppHandler::PushPowrSensorsToDbus: unsuported format:{BYTE}",
                "BYTE", lg2::hex, SensorFormat);
            TraceOncePollSensor = false;
        }

        index += bytesPerSensor * NumberSensors;
    }

}//end PushPowrSensorsToDbus

void OccPollAppHandler::PushCapsSensorsToDbus(uint16_t& index )
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
            lg2::error("OccPollAppHandler::PushCapsSensorFormat: unsuported format :{BYTE}",
                "BYTE", lg2::hex, SensorFormat);
            TraceOncePollSensor = false;
        }

            index += bytesPerSensor * NumberSensors;
    }



}//end PushCapsSensorsToDbus

void OccPollAppHandler::PushExtnSensorsToDbus(uint16_t& index )
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
                    lg2::error("OccPollAppHandler::PushExtnSensorsToDbus: EXTN label name FAILED:{BYTE}",
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
            lg2::error("OccPollAppHandler::PushExtnSensorsToDbus: unsuported format:{BYTE}",
                "BYTE", lg2::hex, SensorFormat);
            TraceOncePollSensor = false;
        }

        index += bytesPerSensor * NumberSensors;

    }


}//end PushExtnSensorsToDbus

void OccPollAppHandler::PushExttSensorsToDbus(uint16_t& index )
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
        lg2::error("OccPollAppHandler::PushExttSensorsToDbus: unsuported format:{BYTE}",
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

}//end PushExttSensorsToDbus

// Called once when the master OCC goes active. This means parms always changed.
bool OccPollAppHandler::pollReadPcapBounds(uint32_t& capSoftMin, uint32_t& capHardMin, uint32_t& capMax)
{
    bool parmsChanged = true;

    HandlePollAction();

    capSoftMin = PollRspSoftMin;
    capHardMin = PollRspHardMin;
    capMax = PollRspMaxPower;

    return parmsChanged;
} //end pollReadPcapBounds

} // namespace occ
} // namespace open_power