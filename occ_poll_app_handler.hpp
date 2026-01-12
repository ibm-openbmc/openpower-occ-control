#pragma once

#include "config.h"

#include "occ_errors.hpp"
#include "occ_poll_handler.hpp"
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

class Status;

constexpr auto SENSOR_BLOCK_VERSION_1 = 1;
constexpr auto TEMP_SENSOR_FORMAT_16 = 16;
constexpr auto FREQ_SENSOR_FORMAT_2 = 2;
constexpr auto POWR_SENSOR_FORMAT_2 = 2;
constexpr auto CAPS_SENSOR_FORMAT_3 = 3;
constexpr auto EXTN_SENSOR_FORMAT_1 = 1;

/** @class OccPollAppHandler
 *  @brief Implements POLLing OCCs
 */
class OccPollAppHandler : public OccPollHandler
{
  public:
    OccPollAppHandler() = delete;
    OccPollAppHandler(const OccPollAppHandler&) = delete;
    OccPollAppHandler& operator=(const OccPollAppHandler&) = delete;
    OccPollAppHandler(OccPollAppHandler&&) = default;

    OccPollAppHandler(Status& status, unsigned int instance);

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
    bool pollReadStateStatus(unsigned int& state,
                             int& lastOccReadStatus) override;

    /** @brief Done One time: From OCC Poll data returns Pcap Information.
     * @param[out] capSoftMin  - Returned soft Minimum cap
     * @param[out] capHardMin  - Returned hard Minimum cap
     * @param[out] capMax.     - Returned max cap
     *
     *  @returns true if data gathering was success
     */
    bool pollReadPcapBounds(uint32_t& capSoftMin, uint32_t& capHardMin,
                            uint32_t& capMax) override;

  private:
    /**  Store the associated Status instance */
    Status& statusObject;

    const unsigned int occInstanceID;

    // Length of POLL Header Section.
    const uint16_t OCC_RSP_HDR_LENGTH = 40;

    // Object variable to store Response data.
    std::vector<std::uint8_t> PollRspData;

    // Data stored for quick return of information after a POLL and Handling the
    // POLL.
    uint16_t PollRspStatus = 0;
    bool ValidPollRspData = false;

    uint16_t PollRspSoftMin = 0;
    uint16_t PollRspHardMin = 0;
    uint16_t PollRspMaxCap = 0;

    /** @brief Create CMD to Poll OCC and send. Store POLL response data for
     * use.
     *
     */
    void sendOccPollCmd();

    /**
     * @brief Take Poll response data and push Temp Sensors onto dbus.
     * @param[in] index - index into PollRspData.
     * */
    void PushTempSensorsToDbus(uint16_t& index);

    /**
     * @brief Take Poll response data and push select Frequencies onto dbus.
     * @param[in] index - index into PollRspData.
     * */
    void PushFreqSensorsToDbus(uint16_t& index);

    /**
     * @brief Take Poll response data and push select Power Values onto dbus.
     * @param[in] index - index into PollRspData.
     * */
    void PushPowrSensorsToDbus(uint16_t& index);

    /**
     * @brief Take Poll response data and push select Power and Power Caps onto
     * dbus.
     * @param[in] index - index into PollRspData.
     * */
    void PushCapsSensorsToDbus(uint16_t& index);

    /**
     * @brief Take Poll response data and push select Power and extra data onto
     * dbus.
     * @param[in] index - index into PollRspData.
     * */
    void PushExtnSensorsToDbus(uint16_t& index);

    /**
     * @brief Take Poll response data and push select data onto dbus.
     * @param[in] index - index into PollRspData.
     * */
    void PushExttSensorsToDbus(uint16_t& index);

    const uint8_t size_label = 5;
    const std::string TEMP_label = "TEMP\0";
    const std::string FREQ_label = "FREQ\0";
    const std::string POWR_label = "POWR\0";
    const std::string CAPS_label = "CAPS\0";
    const std::string EXTN_label = "EXTN\0";
    const std::string EXTT_label = "EXTT\0";

}; //

} // namespace occ
} // namespace open_power
