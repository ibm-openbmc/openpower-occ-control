#include "config.h"

#include "occ_command.hpp"

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include <org/open_power/OCC/Device/error.hpp>
#include <phosphor-logging/lg2.hpp>

#include <algorithm>
#include <format>
#include <string>

// #define TRACE_PACKETS

namespace open_power
{
namespace occ
{

// Trace block of data in hex
void dump_hex(const std::vector<std::uint8_t>& data,
              const unsigned int data_len)
{
    unsigned int dump_length = data.size();
    if ((data_len > 0) && (data_len < dump_length))
    {
        dump_length = data_len;
    }
    std::string s;
    for (uint32_t i = 0; i < dump_length; i++)
    {
        if (i % 16 == 0)
        {
            s += std::format("{:04X}: ", i);
        }
        else if (i % 4 == 0)
        {
            s += " ";
        }

        s += std::format("{:02X}", data.at(i));

        if ((i % 16 == 15) || (i == (dump_length - 1)))
        {
            lg2::info("{STRING}", "STRING", s);
            s.clear();
        }
    }
}

OccCommand::OccCommand(const char* path) :
    path(path), occId([](const std::string& p) {
        // Extract last two segments: "chassis<N>/occ<M>"
        auto slash2 = p.rfind('/');
        auto slash1 = (slash2 == std::string::npos) ? std::string::npos
                                                    : p.rfind('/', slash2 - 1);
        return (slash1 == std::string::npos) ? p : p.substr(slash1 + 1);
    }(path)),
    // Kernel device index derived from last character of path (occ instance).
    devicePath(OCC_DEV_PATH + std::to_string(this->path.back() - '0')),
    activeStatusSignal(
        utils::getBus(),
        sdbusRule::propertiesChanged(path, "org.open_power.OCC.Status"),
        std::bind(std::mem_fn(&OccCommand::activeStatusEvent), this,
                  std::placeholders::_1))
{
#ifdef TRACE_PACKETS
    lg2::info("OccCommand::OccCommand(path={PATH}, devicePath={DEVICE})",
              "PATH", this->path, "DEVICE", devicePath);
#endif
}

void OccCommand::openDevice()
{
    using namespace sdbusplus::org::open_power::OCC::Device::Error;

#ifdef TRACE_PACKETS
    lg2::info("OccCommand::openDevice: calling open {PATH}", "PATH",
              devicePath);
#endif
    fd = open(devicePath.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0)
    {
        const int openErrno = errno;
        lg2::error(
            "OccCommand::openDevice: open failed (errno={ERR}, path={PATH})",
            "ERR", openErrno, "PATH", devicePath);
    }
#ifdef TRACE_PACKETS
    else
    {
        lg2::debug(
            "OccCommand::openDevice: open success (fd={FD}, path={PATH})", "FD",
            fd, "PATH", devicePath);
    }
#endif

    return;
}

void OccCommand::closeDevice()
{
    if (fd >= 0)
    {
#ifdef TRACE_PACKETS
        lg2::info("OccCommand::closeDevice: calling close()");
#endif
        close(fd);
        fd = -1;
    }
}

CmdStatus OccCommand::send(const std::vector<uint8_t>& command,
                           std::vector<uint8_t>& response)
{
    using namespace sdbusplus::org::open_power::OCC::Device::Error;
    CmdStatus status = CmdStatus::FAILURE;

    response.clear();

    openDevice();

    if (fd < 0)
    {
        // OCC is inactive; empty response
        return CmdStatus::COMM_FAILURE;
    }

    const uint8_t cmd_type = command[0];
#ifdef TRACE_PACKETS
    lg2::info("{OCC}: Sending {CMD} command (length={LEN}, {PATH})", "OCC",
              occId, "CMD", lg2::hex, cmd_type, "LEN", command.size(), "PATH",
              devicePath);
    dump_hex(command);
#endif

    int retries = 1; // Allow a retry if a command fails to get valid response
    do
    {
#ifdef TRACE_PACKETS
        lg2::info("OccCommand::send: calling write()");
#endif
        auto rc = write(fd, command.data(), command.size());
        const int writeErrno = errno;
        if ((rc < 0) || (rc != (int)command.size()))
        {
            lg2::error(
                "OccCommand::send: write({OCC}, command:{CMD}) failed with errno={ERR} (retries={RETRIES})",
                "OCC", occId, "CMD", lg2::hex, cmd_type, "ERR", writeErrno,
                "RETRIES", retries);
            status = CmdStatus::COMM_FAILURE;
            // retry if available
            continue;
        }
#ifdef TRACE_PACKETS
        else
        {
            lg2::debug("OccCommand::send: write succeeded");
        }
#endif

        // Now read the response. This would be the content of occ-sram
        while (1)
        {
            uint8_t data{};
#ifdef TRACE_PACKETS
            lg2::info("OccCommand::send: calling read()");
#endif
            auto len = read(fd, &data, sizeof(data));
            const int readErrno = errno;
            if (len > 0)
            {
                response.emplace_back(data);
            }
            else if (len < 0 && readErrno == EAGAIN)
            {
                // We may have data coming still.
                // This driver does not need a sleep for a retry.
                continue;
            }
            else if (len == 0)
            {
#ifdef TRACE_PACKETS
                lg2::info("OccCommand::send: read completed");
#endif
                // We have read all that we can.
                status = CmdStatus::SUCCESS;
                break;
            }
            else
            {
                lg2::error(
                    "OccCommand::send: read({OCC}, command:{CMD}) failed with errno={ERR} (rspSize={LEN}, retries={RETRIES})",
                    "OCC", occId, "CMD", lg2::hex, cmd_type, "ERR", readErrno,
                    "LEN", response.size(), "RETRIES", retries);
                status = CmdStatus::COMM_FAILURE;
                break;
            }
        }
        if (status != CmdStatus::SUCCESS)
        {
            // retry if available
            continue;
        }

        if (response.size() > 2)
        {
#ifdef TRACE_PACKETS
            lg2::info(
                "{OCC}: Received {CMD} response (length={LEN} w/checksum)",
                "OCC", occId, "CMD", lg2::hex, cmd_type, "LEN",
                response.size());
            dump_hex(response, 64);
#endif

            // Validate checksum (last 2 bytes of response)
            const unsigned int csumIndex = response.size() - 2;
            const uint32_t rspChecksum =
                (response[csumIndex] << 8) + response[csumIndex + 1];
            // OCC checksum is the 2 byte sum of data (ignoring overflow)
            uint16_t calcChecksum = 0;
            for (unsigned int index = 0; index < csumIndex; ++index)
            {
                calcChecksum += response[index];
            }
            if (calcChecksum != rspChecksum)
            {
                lg2::error(
                    "{OCC}: Checksum Mismatch: response {RCVD}, calculated {EXPECT}",
                    "OCC", occId, "RCVD", rspChecksum, "EXPECT", calcChecksum);
                dump_hex(response);
                status = CmdStatus::COMM_FAILURE;
            }
            else
            {
                // Validate response was for the specified command
                if (command[0] == response[1])
                {
                    // Valid response received

                    // Strip off 2 byte checksum
                    response.pop_back();
                    response.pop_back();
                    break;
                }
                else
                {
                    lg2::error("OccCommand::send: Response command mismatch "
                               "(sent: "
                               "{CMD}, rsp: {STATUS}, rsp seq#: {SEQ}",
                               "CMD", lg2::hex, command[0], "STATUS", lg2::hex,
                               response[1], "SEQ", lg2::hex, response[0]);
                    dump_hex(response, 64);
                }
            }
        }
        else
        {
            lg2::error("OccCommand::send: Invalid {OCC} response length: {LEN}",
                       "OCC", occId, "LEN", response.size());
            status = CmdStatus::FAILURE;
            dump_hex(response);
        }

        if (retries > 0)
        {
            lg2::error("OccCommand::send: Command will be retried");
            response.clear();
        }
    } while (retries-- > 0);

    closeDevice();

    return status;
}

// Called at OCC Status change signal
void OccCommand::activeStatusEvent(sdbusplus::message_t& msg)
{
    std::string statusInterface;
    std::map<std::string, std::variant<bool>> msgData;
    msg.read(statusInterface, msgData);

    auto propertyMap = msgData.find("OccActive");
    if (propertyMap != msgData.end())
    {
        // Extract the OccActive property
        if (!std::get<bool>(propertyMap->second))
        {
            this->closeDevice();
        }
    }
    return;
}

} // namespace occ
} // namespace open_power
