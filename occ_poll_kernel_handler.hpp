#pragma once

#include "occ_poll_handler.hpp"
#include "occ_status.hpp"

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

/** @class OccPollKernelHandler
 *  @brief Implements POLLing OCCs
 */
class OccPollKernelHandler : public OccPollHandler
{
  public:
    OccPollKernelHandler() = delete;
    OccPollKernelHandler(const OccPollKernelHandler&) = delete;
    OccPollKernelHandler& operator=(const OccPollKernelHandler&) = delete;
    OccPollKernelHandler(OccPollKernelHandler&&) = default;
    OccPollKernelHandler& operator=(OccPollKernelHandler&&) = default;

    OccPollKernelHandler(Status& status, unsigned int instance);


    /** @brief Done every 5 Sec.: From OCC Poll push data needed to the dbus.
     *
     */
    void HandlePollAction() override;

    /**
     * @brief Done when OCC goes active: From OCC Poll data returns OCC state
     * @param[out] state  - Returned State of OCC
     * @param[out] lastOccReadStatus - Returned (-1) if invalid (0) if valid)
     *
     *  @returns true if data gathering was success
     * */
    bool pollReadStateStatus(unsigned int& state, int& lastOccReadStatus) override;

    /** @brief Done One time: From OCC Poll data returns Pcap Information.
     * @param[out] capSoftMin  - Returned soft Minimum cap
     * @param[out] capHardMin  - Returned hard Minimum cap
     * @param[out] capMax.     - Returned max cap
     *
     *  @returns true if data gathering was success
     */
    bool pollReadPcapBounds(uint32_t& capSoftMin, uint32_t& capHardMin, uint32_t& capMax) override;


  private:

    /**  Store the associated Status instance */
    Status& statusObject;

    const unsigned int occInstanceID;

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

    /** @brief Get FunctionID from the `powerX_label` file.
     *  @param[in] value - the value of the `powerX_label` file.
     *  @returns FunctionID of the power sensors.
     */
    std::optional<std::string> getPowerLabelFunctionID(const std::string& value);

}; //

} // namespace occ
} // namespace open_power