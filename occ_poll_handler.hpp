#pragma once

#include "config.h"

#include <chrono>
#include <filesystem>
#include <regex>
#include <vector>

#include "occ_errors.hpp"
#include "utils.hpp"


namespace open_power
{
namespace occ
{

using namespace std::literals::chrono_literals;

class Status;

constexpr auto POLL_VERSION_FORMAT_1 = 1;
constexpr auto TEMP_SENSOR_FORMAT_16 = 16;
constexpr auto FREQ_SENSOR_FORMAT = 2;
constexpr auto POWR_SENSOR_FORMAT = 2;
constexpr auto CAPS_SENSOR_FORMAT = 3;
constexpr auto EXTN_SENSOR_FORMAT = 1;
constexpr auto EXTT_SENSOR_FORMAT = 10;

constexpr auto EXTN_LABEL_FMIN = 0x464D494E;
constexpr auto EXTN_LABEL_FDIS = 0x46444953;
constexpr auto EXTN_LABEL_FBAS = 0x46424153;
constexpr auto EXTN_LABEL_FUT  = 0x46555400;
constexpr auto EXTN_LABEL_FMAX = 0x464D4158;
constexpr auto EXTN_LABEL_CLIP = 0x434C4950;
constexpr auto EXTN_LABEL_MODE = 0x4D4F4445;
constexpr auto EXTN_LABEL_WOFC = 0x574F4643;
constexpr auto EXTN_LABEL_WOFI = 0x574F4649;
constexpr auto EXTN_LABEL_PWRM = 0x5057524d;
constexpr auto EXTN_LABEL_PWRP = 0x50575250;
constexpr auto EXTN_LABEL_ERRH = 0x45525248;


/** @class OccPollHandler
 *  @brief Implements POLLing OCCs
 */
class OccPollHandler
{
  public:
    OccPollHandler() = delete;
    OccPollHandler(const OccPollHandler&) = delete;
    OccPollHandler& operator=(const OccPollHandler&) = delete;
    OccPollHandler(OccPollHandler&&) = default;
    OccPollHandler& operator=(OccPollHandler&&) = default;

    OccPollHandler(Status& status);

    /**  Store the associated Status instance */
    Status& statusObject;

    const uint32_t occInstanceID;


//##############################################################################
// COMMON PUBLIC INTERFACES
//##############################################################################
    /** @brief Done every 5 Sec.: From OCC Poll push data needed to the dbus.
     *
     */
    void HandlePollAction();

    /**
     * @brief Done when OCC goes active: From OCC Poll data returns OCC state
     * @param[out] state  - Returned State of OCC
     * @param[out] lastOccReadStatus - Returned (-1) if invalid (0) if valid)
     *
     *  @returns true if data gathering was success
     * */
    bool pollReadStateStatus(unsigned int& state, int& lastOccReadStatus);

    /** @brief Done One time: From OCC Poll data returns Pcap Information.
     * @param[out] capSoftMin  - Returned soft Minimum cap
     * @param[out] capHardMin  - Returned hard Minimum cap
     * @param[out] capMax.     - Returned max cap
     *
     *  @returns true if data gathering was success
     */
    bool pollReadPcapBounds(uint32_t& capSoftMin, uint32_t& capHardMin, uint32_t& capMax);


    /** @brief clear OCC flags to allow trace of POLL parsing errors.
     *
     */
    void clearOccPollTraceFlags()
    {
        TraceOncePollHeader = false;
        TraceOncePollSensor = false;
        TraceOncePwrFuncId = false;
    }


  private:

//##############################################################################
// OCC-CONTROL Private POLL Functions.
//##############################################################################
#ifdef ENABLE_APP_POLL_SUPPORT

    // Length of POLL Header Section.
    const uint16_t OCC_RSP_HDR_LENGTH = 40;

    // Object variable to store Response data.
    std::vector<std::uint8_t> PollRspData;

    // Data stored for quick return of information after a POLL and Handling the POLL.
    uint16_t PollRspStatus = 0;

    uint16_t PollRspSoftMin = 0;
    uint16_t PollRspHardMin = 0;
    uint16_t PollRspMaxPower = 0;

    /** @brief Create CMD to Poll OCC and send. Store POLL response data for use.
     *
     */
    void sendOccPollCmd();

    /** @brief ????
     *
     */
    void pollOccNow();

    /**
     * @brief .
     * @param[in] index - index into PollRspData.
     * */
    void PushTempSensorsToDbus(uint16_t& index );

    /**
     * @brief .
     * @param[in] index - index into PollRspData.
     * */
    void PushFreqSensorsToDbus(uint16_t& index );

    /**
     * @brief .
     * @param[in] index - index into PollRspData.
     * */
    void PushPowrSensorsToDbus(uint16_t& index );

    /**
     * @brief .
     * @param[in] index - index into PollRspData.
     * */
    void PushCapsSensorsToDbus(uint16_t& index );

    /**
     * @brief .
     * @param[in] index - index into PollRspData.
     * */
    void PushExtnSensorsToDbus(uint16_t& index );

    /**
     * @brief .
     * @param[in] index - index into PollRspData.
     * */
    void PushExttSensorsToDbus(uint16_t& index );

    // /** @brief The POLL sensor labels map */
    enum
    {
        TEMP = 0,
        FREQ,
        POWR,
        CAPS,
        EXTN,
        EXTT,
        sizeSensorLabelList
    };
    const std::vector<uint8_t> SensorLabel[sizeSensorLabelList] = {
        {0x54, 0x45, 0x4D, 0x50, 0x00}, // TEMP
        {0x46, 0x52, 0x45, 0x51, 0x00}, // FREQ
        {0x50, 0x4F, 0x57, 0x52, 0x00}, // POWR
        {0x43, 0x41, 0x50, 0x53, 0x00}, // CAPS
        {0x45, 0x58, 0x54, 0x4E, 0x00}, // EXTN
        {0x45, 0x58, 0x54, 0x54, 0x00}, // EXTT
    };

#else
//##############################################################################
// KERNEL Private POLL Functions.
//##############################################################################
    /**
     * @brief Trigger OCC driver to read the temperature sensors and push to dbus.
     * @param[in] path - path of the OCC sensors.
     * @param[in] id - Id of the OCC.
     * */
    void pushTempSensorsToDbus(const fs::path& path, uint32_t occInstance);

    /**
     * @brief Trigger OCC driver to read the extended sensors and push to dbus.
     * @param[in] path - path of the OCC sensors.
     * @param[in] id - Id of the OCC.
     * */
    void pushExtnSensorsToDbus(const fs::path& path, uint32_t id);

    /**
     * @brief Trigger OCC driver to read the power sensors and push to dbus.
     * @param[in] path - path of the OCC sensors.
     * @param[in] id - Id of the OCC.
     * */
    void pushPowrSensorsToDbus(const fs::path& path, uint32_t id);
    /**
     * @brief Returns the filename to use for the user power cap
     *
     * The file is of the form "powerX_cap_user", where X is any
     * number.
     *
     * @param[in] expr - Regular expression of file to find
     *
     * @return full path/filename, or empty path if not found.
     */
    fs::path getPcapFilename(const std::regex& expr);

    /** @brief Path to the sysfs files holding the cap properties **/
    fs::path pcapBasePathname;


#endif


//##############################################################################
// COMMON Private.
//##############################################################################

    // Flags to prevent flooding traces every 5 Sec.
    bool TraceOncePollHeader = true;
    bool TraceOncePollSensor = true;
    bool TraceOncePwrFuncId = true;

    enum occFruType
    {
        processorCore = 0,
        internalMemCtlr = 1,
        dimm = 2,
        memCtrlAndDimm = 3,
        VRMVdd = 6,
        PMIC = 7,
        memCtlrExSensor = 8,
        processorIoRing = 9
    };


    /** @brief Get FunctionID from the `powerX_label` file.
     *  @param[in] value - the value of the `powerX_label` file.
     *  @returns FunctionID of the power sensors.
     */
    std::optional<std::string> getPowerLabelFunctionID(const std::string& value);


    /** @brief Get ????
     *  @param[in] ????
     *  @returns bool ????
     */
    bool BuildTempDbusPaths(std::string& sensorPath,
                            std::string& dvfsTempPath,
                            const uint32_t SensorID,
                            const uint32_t fruTypeValue,
                            const uint32_t occInstance);

    /** @brief The dimm temperature sensor names map  used by */
    const std::map<uint32_t, std::string> dimmTempSensorName = {
        {internalMemCtlr, "_intmb_temp"},
        {dimm, "_dram_temp"},
        {memCtrlAndDimm, "_dram_extmb_temp"},
        {PMIC, "_pmic_temp"},
        {memCtlrExSensor, "_extmb_temp"}};

    /** @brief The dimm DVFS temperature sensor names map  */
    const std::map<uint32_t, std::string> dimmDVFSSensorName = {
        {internalMemCtlr, "dimm_intmb_dvfs_temp"},
        {dimm, "dimm_dram_dvfs_temp"},
        {memCtrlAndDimm, "dimm_dram_extmb_dvfs_temp"},
        {PMIC, "dimm_pmic_dvfs_temp"},
        {memCtlrExSensor, "dimm_extmb_dvfs_temp"}};

    /** @brief The power sensor names map */
    const std::map<std::string, std::string> powerSensorName = {
        {"system", "total_power"}, {"1", "p0_mem_power"},
        {"2", "p1_mem_power"},     {"3", "p2_mem_power"},
        {"4", "p3_mem_power"},     {"5", "p0_power"},
        {"6", "p1_power"},         {"7", "p2_power"},
        {"8", "p3_power"},         {"9", "p0_cache_power"},
        {"10", "p1_cache_power"},  {"11", "p2_cache_power"},
        {"12", "p3_cache_power"},  {"13", "io_a_power"},
        {"14", "io_b_power"},      {"15", "io_c_power"},
        {"16", "fans_a_power"},    {"17", "fans_b_power"},
        {"18", "storage_a_power"}, {"19", "storage_b_power"},
        {"23", "mem_cache_power"}, {"25", "p0_mem_0_power"},
        {"26", "p0_mem_1_power"},  {"27", "p0_mem_2_power"},
        {"34", "pcie_power"},
        {"35", "pcie_dcm0_power"}, {"36", "pcie_dcm1_power"},
        {"37", "pcie_dcm2_power"}, {"38", "pcie_dcm3_power"},
        {"39", "io_dcm0_power"},   {"40", "io_dcm1_power"},
        {"41", "io_dcm2_power"},   {"42", "io_dcm3_power"},
        {"43", "avdd_total_power"}};

//##############################################################################
// END
//##############################################################################

}; //

} // namespace occ
} // namespace open_power