#pragma once

#include "config.h"

#include "occ_errors.hpp"
#include "utils.hpp"

#include <chrono>
#include <filesystem>
#include <regex>
#include <vector>

namespace open_power
{
namespace occ
{

using namespace std::literals::chrono_literals;

class OccObject;

constexpr auto EXTN_LABEL_FMIN = 0x464D494E;
constexpr auto EXTN_LABEL_FDIS = 0x46444953;
constexpr auto EXTN_LABEL_FBAS = 0x46424153;
constexpr auto EXTN_LABEL_FUT = 0x46555400;
constexpr auto EXTN_LABEL_FMAX = 0x464D4158;
constexpr auto EXTN_LABEL_CLIP = 0x434C4950;
constexpr auto EXTN_LABEL_MODE = 0x4D4F4445;
constexpr auto EXTN_LABEL_WOFC = 0x574F4643;
constexpr auto EXTN_LABEL_WOFI = 0x574F4649;
constexpr auto EXTN_LABEL_PWRM = 0x5057524d;
constexpr auto EXTN_LABEL_PWRP = 0x50575250;
constexpr auto EXTN_LABEL_ERRH = 0x45525248;

constexpr auto SID_TYPE_CORE = 0xC0;
constexpr auto SID_TYPE_DIMM = 0xD0;

/** @class OccPollHandler
 *  @brief Implements POLLing OCCs
 */
class OccPollHandler
{
  public:
    /** @brief Done every 5 Sec.: From OCC Poll push data needed to the dbus.
     *
     */
    virtual void HandlePollAction() = 0;

    /**
     * @brief Done when OCC goes active: From OCC Poll data returns OCC state
     * @param[out] state  - Returned State of OCC
     * @param[out] lastOccReadStatus - Returned (-1) if invalid (0) if valid)
     *
     *  @returns true if data gathering was success
     * */
    virtual bool pollReadStateStatus(unsigned int& state,
                                     int& lastOccReadStatus) = 0;

    /** @brief Done One time: From OCC Poll data returns Pcap Information.
     * @param[out] capSoftMin  - Returned soft Minimum cap
     * @param[out] capHardMin  - Returned hard Minimum cap
     * @param[out] capMax.     - Returned max cap
     *
     *  @returns true if data gathering was success
     */
    virtual bool pollReadPcapBounds(uint32_t& capSoftMin, uint32_t& capHardMin,
                                    uint32_t& capMax) = 0;

    /** @brief clear OCC flags to allow trace of POLL parsing errors.
     *
     */
    void clearOccPollTraceFlags()
    {
        TraceOncePollHeader = false;
        TraceOncePollSensor = false;
        TraceOncePwrFuncId = false;
    }

  protected:
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
        processorIoRing = 9,
        processorMMA = 10, // Matrix Multiply Accelerator
        max = 11,
        MAX_FRU_TYPES,
        FRU_UNAVAILABLE = 0xFF

    };

    /** @brief Get the sensor path, dvfs path, and tcontrol path.
     *  @param[in] sensorPath - return path of the OCC sensor.
     *  @param[in] dvfsTempPath - return path of the DVFS sensor.
     *  @param[in] tcontrolTempPath - return path of the tcontrol sensor.
     *  @param[in] SensorID - Input Id of the Sensor.
     *  @param[in] fruTypeValue - Input fru type.
     *  @param[in] occInstance - Input Id of the OCC.
     *  @param[in] isHottest - true if this is hottest of this fru type.
     *  @returns bool If Sensor Found
     */
    bool BuildTempDbusPaths(
        std::string& sensorPath, std::string& dvfsTempPath,
        std::string& tcontrolTempPath, const uint32_t SensorID,
        const uint32_t fruTypeValue, const uint32_t occInstance,
        const bool isHottest = false);

    // P12 naming: suffix appended after "chassis_procY_dimmN" in
    // BuildTempDbusPaths(); DVFS suffix appended after "chassis_procY_".
    /** @brief The dimm temperature sensor names map */
    const std::map<uint32_t, std::string> dimmTempSensorName = {
        {internalMemCtlr, "_intmb_temp"},
        {dimm, "_dram_temp"},
        {memCtrlAndDimm, "_dram_extmb_temp"},
        {PMIC, "_pmic_temp"},
        {memCtlrExSensor, "_extmb_temp"}};

    /** @brief The dimm DVFS temperature sensor names map */
    const std::map<uint32_t, std::string> dimmDVFSSensorName = {
        {internalMemCtlr, "dimm_intmb_dvfs_temp"},
        {dimm, "dimm_dram_dvfs_temp"},
        {memCtrlAndDimm, "dimm_dram_extmb_dvfs_temp"},
        {PMIC, "dimm_pmic_dvfs_temp"},
        {memCtlrExSensor, "dimm_extmb_dvfs_temp"}};

    // P12 naming: values are leaf names appended to "chassis_procY_" in the
    // power push functions (except "system" which has no proc prefix).
    //
    // funcIDs 1-4   : per-proc memory domain power (aggregated by that OCC)
    // funcIDs 5-8   : per-proc total processor power (aggregated by that OCC)
    // funcIDs 9-12  : per-proc cache power
    // funcIDs 13-15 : IO subsystem power
    // funcIDs 16-17 : fan power
    // funcIDs 18-19 : storage power
    // funcID  23    : memory cache power
    // funcIDs 25-27 : per-proc memory channel power
    // funcID  34    : PCIe total power
    // funcIDs 35-38 : PCIe per-DCM power
    // funcIDs 39-42 : IO per-DCM power
    // funcID  43    : AVDD total power
    //
    // EXTN PWRP/PWRM are raw per-chiplet DC power (derated to AC) and use
    // separate paths: chassis_procY_chiplet_power /
    // chassis_procY_chiplet_mem_power
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
        {"34", "pcie_power"},      {"35", "pcie_dcm0_power"},
        {"36", "pcie_dcm1_power"}, {"37", "pcie_dcm2_power"},
        {"38", "pcie_dcm3_power"}, {"39", "io_dcm0_power"},
        {"40", "io_dcm1_power"},   {"41", "io_dcm2_power"},
        {"42", "io_dcm3_power"},   {"43", "avdd_total_power"}};

  private:
};

} // namespace occ
} // namespace open_power
