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


OccPollKernelHandler::OccPollKernelHandler(Status& status, unsigned int instance):
      statusObject(status),
      occInstanceID(instance)
{

}

std::optional<std::string> OccPollKernelHandler::getPowerLabelFunctionID(
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

bool OccPollKernelHandler::pollReadStateStatus(unsigned int& state, int& lastOccReadStatus)
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
                "OccPollKernelHandler::pollReadStateStatus: open/read failed trying to read OCC{INST} state (open errno={ERROR})",
                "INST", occInstanceID, "ERROR", openErrno);
            lastOccReadStatus = openErrno;
        }

    }

    file.close();


    return stateWasRead;
}//end pollReadStateStatus

void OccPollKernelHandler::HandlePollAction()
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
                "OccPollKernelHandler::getSensorValues: OCC{INST} sensor path missing: {PATH}",
                "INST", occInstanceID, "PATH", sensorPath);
            tracedError[occInstanceID] = true;
        }
    }

    return;
}//end HandlePollAction

// Called once when the master OCC goes active. This means parms always changed.
bool OccPollKernelHandler::pollReadPcapBounds(uint32_t& capSoftMin, uint32_t& capHardMin, uint32_t& capMax)
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
            "pollReadPcapBounds: unable to find pcap_min_soft file: {FILE} (errno={ERR})",
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
            "pollReadPcapBounds: unable to find cap_min file: {FILE} (errno={ERR})",
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
            "pollReadPcapBounds: unable to find cap_max file: {FILE} (errno={ERR})",
            "FILE", pcapBasePathname, "ERR", errno);
    }

    return parmsChanged;
} //end pollReadPcapBounds

fs::path OccPollKernelHandler::getPcapFilename(const std::regex& expr)
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
} //end getPcapFilename

void OccPollKernelHandler::pushTempSensorsToDbus(const fs::path& path, uint32_t occInstance)
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
                lg2::error("OccPollKernelHandler::pushTempSensorsToDbus:setChassisAssociation FAILED OCC:{INST} watch:{ERROR} PATH:{PATH}",
                    "INST", occInstanceID, "ERROR", e.what(), "PATH", objectPath );
            }
        }
        statusObject.existingSensors[objectPath] = occInstance;

    }

    return;
} //end pushTempSensorsToDbus

void OccPollKernelHandler::pushExtnSensorsToDbus(const fs::path& path, uint32_t occInstanceID)
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
} //end pushExtnSensorsToDbus

void OccPollKernelHandler::pushPowrSensorsToDbus(const fs::path& path, uint32_t occInstanceID)
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
} //end pushPowrSensorsToDbus

} // namespace occ
} // namespace open_power