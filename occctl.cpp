#include "config.h"

#include "occctl.hpp"

#include "occ_command.hpp"
#include "utils.hpp"

#include <arpa/inet.h>
#include <sys/wait.h>
#include <unistd.h>

#include <nlohmann/json.hpp>
#include <phosphor-logging/lg2.hpp>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

namespace
{

bool verbose = false;
bool force = false;

// OCC command codes (from CmdType enum in occ_command.hpp)
constexpr uint8_t CMD_POLL =
    static_cast<uint8_t>(open_power::occ::CmdType::POLL);
constexpr uint8_t CMD_DEBUG_PT =
    static_cast<uint8_t>(open_power::occ::CmdType::DEBUG_PASS_THROUGH);
constexpr uint8_t CMD_MFG_TEST =
    static_cast<uint8_t>(open_power::occ::CmdType::MFG_TEST);

// ANSI color/style codes — return empty string when stdout is not a terminal
// so piped/redirected output is never polluted with escape sequences.
struct Ansi
{
    static bool enabled()
    {
        return isatty(STDOUT_FILENO) != 0;
    }

    static const char* bold()
    {
        return enabled() ? "\033[1m" : "";
    }
    static const char* reset()
    {
        return enabled() ? "\033[0m" : "";
    }
    static const char* red()
    {
        return enabled() ? "\033[31m" : "";
    }
    static const char* green()
    {
        return enabled() ? "\033[32m" : "";
    }
    static const char* yellow()
    {
        return enabled() ? "\033[33m" : "";
    }
};

int runSystemCmd(const std::string& cmd)
{
    if (verbose)
    {
        std::cout << "==> " << cmd << "\n";
    }
    std::fflush(stdout);
    return std::system(cmd.c_str());
}

// Run a command and capture its stdout. Returns exit code.
// Output lines are appended to 'output'.
int captureCommand(const std::string& cmd, std::string& output)
{
    if (verbose)
    {
        std::cout << "==> " << cmd << "\n";
    }
    std::fflush(stdout);

    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe)
    {
        return -1;
    }
    char buf[256];
    while (fgets(buf, sizeof(buf), pipe))
    {
        output += buf;
    }
    return pclose(pipe);
}

// Returns true if the host is running, false otherwise.
// Prints an error and returns false if the check itself fails.
bool checkHostRunning()
{
    try
    {
        return open_power::occ::utils::isHostRunning();
    }
    catch (const std::exception& e)
    {
        lg2::debug("Failed to check if host is running: {ERR}", "ERR",
                   e.what());
        return false;
    }
}

// Checks that the OCC is active and prints an error if not.
// Returns true if the OCC is active (or --force was given), false otherwise.
bool requireOccActive(int chassis, int occ)
{
    if (force)
    {
        return true;
    }

    std::string occLabel = std::string(CHASSIS_NAME) + std::to_string(chassis) +
                           "/" + OCC_NAME + std::to_string(occ);
    std::string path = std::string(OCC_CONTROL_ROOT) + "/" + occLabel;
    constexpr auto occStatusInterface = "org.open_power.OCC.Status";

    try
    {
        auto propVal = open_power::occ::utils::getProperty(
            path, occStatusInterface, "OccActive");
        if (!std::get<bool>(propVal))
        {
            std::cerr << "ERROR: OCC is not active (" << occLabel << ")\n";
            return false;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "ERROR: Failed to read OccActive for " << occLabel << ": "
                  << e.what() << "\n";
        return false;
    }

    return true;
}

// Checks that the host is running and prints an error if not.
// Returns true if the host is running (or --force was given), false otherwise.
bool requireHostRunning()
{
    if (force)
    {
        if (verbose)
        {
            std::cerr
                << "WARNING: --force specified; skipping BootProgress state check\n";
        }
        return true;
    }
    if (!checkHostRunning())
    {
        std::string bootProgress = "unknown";
        try
        {
            auto propVal = open_power::occ::utils::getProperty(
                "/xyz/openbmc_project/state/host0",
                "xyz.openbmc_project.State.Boot.Progress", "BootProgress");
            bootProgress = std::get<std::string>(propVal);
        }
        catch (const std::exception&)
        {}
        std::cerr << "ERROR: BootProgress state prevents OCC communication\n";
        std::cerr << "       BootProgress : " << bootProgress << "\n";
        return false;
    }
    return true;
}

#ifndef UNIT_TEST

std::vector<int> findOCCsInDev()
{
    std::vector<int> occs;

    if (fs::exists("/dev"))
    {
        constexpr std::string_view prefix = "occ";
        for (auto& file : fs::directory_iterator("/dev"))
        {
            // Match device names of the form "occ<digits>"
            const std::string name{file.path().filename().string()};
            if (name.size() <= prefix.size() ||
                name.compare(0, prefix.size(), prefix) != 0)
            {
                continue;
            }
            const char* first = name.data() + prefix.size();
            const char* last = name.data() + name.size();
            int num = 0;
            auto [ptr, ec] = std::from_chars(first, last, num);
            if (ec == std::errc{} && ptr == last)
            {
                occs.push_back(num);
            }
        }
    }

    std::sort(occs.begin(), occs.end());
    return occs;
}

int displayOccStatus()
{
    // Display the individual OCC status
    auto occDevs = findOCCsInDev();
    constexpr auto occStatusInterface = "org.open_power.OCC.Status";
    if (occDevs.empty())
    {
        std::cout << "No OCC devices found in /dev.\n";
    }
    else
    {
        std::cout << "OCC ActiveSensor status (" << occDevs.size()
                  << " OCC devices):\n";
        for (auto id : occDevs)
        {
            int chassis = id + 1;
            std::string occLabel =
                std::string(CHASSIS_NAME) + std::to_string(chassis) + "/" +
                OCC_NAME + "0";
            std::string path = std::string(OCC_CONTROL_ROOT) + "/" + occLabel;
            std::string devPath =
                std::string(OCC_DEV_PATH) + std::to_string(id);

            bool occActive = false;
            try
            {
                auto propVal = open_power::occ::utils::getProperty(
                    path, occStatusInterface, "OccActive");
                occActive = std::get<bool>(propVal);
            }
            catch (const std::exception& e)
            {
                lg2::debug("Failed to get OccActive for {PATH}: {ERR}", "PATH",
                           path, "ERR", e.what());
            }

            std::string state = occActive ? "active" : "unavailable";
            std::cout << "  " << occLabel << ": " << std::left << std::setw(10)
                      << state << " (" << devPath << ")\n";
        }
    }

    // Check if system is in Safe Mode
    constexpr auto powerModePath =
        "/xyz/openbmc_project/control/host0/power_mode";
    constexpr auto powerModeInterface =
        "xyz.openbmc_project.Control.Power.Mode";
    try
    {
        auto propVal = open_power::occ::utils::getProperty(
            powerModePath, powerModeInterface, "SafeMode");
        if (std::get<bool>(propVal))
        {
            std::cout
                << Ansi::red() << Ansi::bold()
                << "System is currently in SAFE MODE (OCCs are not running)"
                << Ansi::reset() << "\n";
        }
    }
    catch (const std::exception& e)
    {
        lg2::debug("Failed to read SafeMode property: {ERR}", "ERR", e.what());
    }

    if (!checkHostRunning())
    {
        std::cout
            << "OCCs are unavailable because of the BootProgress state:\n";
        std::fflush(stdout);
        auto rc = runSystemCmd("obmcutil status | sed 's/^/  /'");
        if (rc != 0)
        {
            std::cout << "Failed to run 'obmcutil status' (exit code: " << rc
                      << ")\n";
        }
    }

    // Check for OCC patch LID
    constexpr auto occPatchLid = "/usr/local/share/hostfw/running/81e00430.lid";
    if (fs::exists(occPatchLid))
    {
        // Read the OCC level string at offset 0x740 (up to 16 bytes,
        // NUL-terminated)
        std::string occPatchLevel;
        std::ifstream lidFile(occPatchLid, std::ios::binary);
        if (lidFile)
        {
            lidFile.seekg(0x740);
            if (lidFile)
            {
                char levelBuf[17] = {};
                lidFile.read(levelBuf, 16);
                occPatchLevel = levelBuf; // stops at embedded NUL
            }
        }

        std::cout << Ansi::yellow() << Ansi::bold() << "OCC Patch found ("
                  << occPatchLid << ")";
        if (!occPatchLevel.empty())
        {
            std::cout << " - Level: " << occPatchLevel;
        }
        std::cout << Ansi::reset() << "\n";
    }

    return 0;
}

int dumpTrace(const std::string& lines = {})
{
    std::string cmd =
        "journalctl -u org.open_power.OCC.Control.service --no-pager";
    if (!lines.empty())
    {
        cmd += " -n " + lines;
    }
    auto rc = runSystemCmd(cmd);
    if (rc != 0)
    {
        std::cerr << "Failed to run journalctl (exit code: " << rc << ")\n";
        return 1;
    }
    return 0;
}

// Known PowerMode short names → full D-Bus enum string
// Returns empty string if the name is not recognised.
struct ModeEntry
{
    int value;
    const char* modeName;
    bool
        isOem; // true = use SetMode busctl call; false = use D-Bus property set
};

const ModeEntry knownModes[] = {
    {0x01, "Static", true},
    {0x02, "NonDeterministic", true},
    {0x03, "SFP", true},
    {0x05, "PowerSaving", false},
    {0x06, "EfficiencyFavorPower", false},
    {0x07, "EfficiencyFavorPerformance", true},
    {0x09, "MaxFrequency", true},
    {0x0A, "BalancedPerformance", true},
    {0x0B, "FFO", true},
    {0x0C, "MaximumPerformance", false},
};

// Resolve a mode name or numeric string to a ModeEntry.
// Returns nullptr if not found.
const ModeEntry* resolvePowerMode(const std::string& name)
{
    // Try numeric (decimal or hex) first
    try
    {
        size_t pos = 0;
        int val = std::stoi(name, &pos, 0);
        if (pos == name.size())
        {
            for (auto& m : knownModes)
            {
                if (m.value == val)
                    return &m;
            }
            return nullptr;
        }
    }
    catch (const std::exception&)
    {}

    // Fall back to name match (case-insensitive)
    for (auto& m : knownModes)
    {
        if (strcasecmp(name.c_str(), m.modeName) == 0)
            return &m;
    }
    return nullptr;
}

constexpr const char* modeKnownModesMsg =
    "  Known modes (name or numeric value):\n"
    "    1 Static, 2 NonDeterministic, 3 SFP*,\n"
    "    5 PowerSaving, 6 EfficiencyFavorPower,\n"
    "    7 EfficiencyFavorPerformance, 9 MaxFrequency,\n"
    "    10 BalancedPerformance, 11 FFO*, 12 MaximumPerformance\n"
    "  * requires oemdata (non-zero frequency point)\n";

// setMode: set power mode via D-Bus.
// modeArg  - mode name or numeric value
// oemData  - frequency point for SFP/FFO (default 0)
int setMode(const std::string& modeArg, uint16_t oemData = 0)
{
    const ModeEntry* entry = resolvePowerMode(modeArg);
    if (!entry)
    {
        std::cerr << "ERROR: Unknown power mode '" << modeArg << "'\n";
        std::cerr << modeKnownModesMsg;
        return 1;
    }

    // SFP & FFO require a non-zero frequency point; all other modes must have 0
    if (entry->value == 0x03 || entry->value == 0x0B)
    {
        if (oemData == 0)
        {
            std::cerr << "ERROR: " << entry->modeName
                      << " requires a non-zero frequency point (oemdata)\n";
            return 1;
        }
    }
    else if (oemData != 0)
    {
        std::cerr << "ERROR: " << entry->modeName
                  << " does not accept a frequency point (oemdata must be 0)\n";
        return 1;
    }

    std::cout << "Setting PowerMode to: " << entry->modeName << " ("
              << entry->value << ")\n";

    if (entry->isOem)
    {
        // OEM modes are set via the PassThrough SetMode method:
        //   busctl call org.open_power.OCC.Control
        //     /org/open_power/control/chassis1/occ0
        //     org.open_power.OCC.PassThrough SetMode yq <value> <oemData>
        constexpr int defaultChassis = open_power::occ::DEFAULT_CHASSIS;
        std::ostringstream cmd;
        cmd << "busctl call org.open_power.OCC.Control"
            << " /org/open_power/control/chassis" << defaultChassis << "/occ0"
            << " org.open_power.OCC.PassThrough SetMode yq " << entry->value
            << " " << oemData;
        std::string ignored;
        int rc = captureCommand(cmd.str(), ignored);
        if (rc != 0)
        {
            std::cerr << "ERROR: Failed to set OEM power mode (exit code: "
                      << rc << ")\n";
            return 1;
        }
    }
    else
    {
        constexpr auto powerModeInterface =
            "xyz.openbmc_project.Control.Power.Mode";
        constexpr auto powerModePath =
            "/xyz/openbmc_project/control/host0/power_mode";
        constexpr auto prefix =
            "xyz.openbmc_project.Control.Power.Mode.PowerMode.";
        std::string fullMode = std::string(prefix) + entry->modeName;

        // Pre-check AllowedPowerModes to give a clear error without
        // triggering the noisy lg2 log inside setProperty on rejection.
        try
        {
            auto allowedProp = open_power::occ::utils::getProperty(
                powerModePath, powerModeInterface, "AllowedPowerModes");
            auto allowed = std::get<std::vector<std::string>>(allowedProp);
            if (!allowed.empty())
            {
                bool found = false;
                for (auto& a : allowed)
                {
                    if (a == fullMode)
                    {
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    std::cerr << "ERROR: Mode '" << entry->modeName
                              << "' is not in AllowedPowerModes:\n";
                    for (auto& a : allowed)
                    {
                        auto p = a.rfind('.');
                        std::cerr
                            << "  "
                            << (p != std::string::npos ? a.substr(p + 1) : a)
                            << "\n";
                    }
                    return 1;
                }
            }
        }
        catch (const std::exception&)
        {
            // Can't read AllowedPowerModes; proceed and let setProperty
            // report any error.
        }

        if (verbose)
        {
            std::cout << "==> busctl set-property"
                      << " xyz.openbmc_project.Control.Power.Mode"
                      << " " << powerModePath << " " << powerModeInterface
                      << " PowerMode s \"" << fullMode << "\"\n";
            std::fflush(stdout);
        }

        try
        {
            open_power::occ::utils::setProperty(
                powerModePath, powerModeInterface, "PowerMode", fullMode);
        }
        catch (const std::exception& e)
        {
            std::cerr << "ERROR: Failed to set PowerMode: " << e.what() << "\n";
            return 1;
        }
    }

    std::cout << "\n";
    return 0;
}

int dumpMode()
{
    constexpr auto powerModeInterface =
        "xyz.openbmc_project.Control.Power.Mode";
    constexpr auto powerModePath =
        "/xyz/openbmc_project/control/host0/power_mode";

    try
    {
        auto propVal = open_power::occ::utils::getProperty(
            powerModePath, powerModeInterface, "PowerMode");
        std::string mode = std::get<std::string>(propVal);

        auto pos = mode.rfind('.');
        std::string shortMode =
            (pos != std::string::npos) ? mode.substr(pos + 1) : mode;

        // If D-Bus reports OEM, look up the real mode from the persist file.
        // cereal JSON keys: value0=modeInitialized, value1=mode,
        // value2=oemModeData
        if (shortMode == "OEM")
        {
            std::string persistFile =
                std::string(OCC_CONTROL_PERSIST_PATH) + "/powerModeData";
            std::ifstream pf(persistFile);
            if (pf.is_open())
            {
                try
                {
                    nlohmann::json j;
                    pf >> j;
                    // cereal wraps the struct in value0; fields are then
                    // value0=modeInitialized, value1=mode, value2=oemModeData
                    auto& d = j.contains("value0") ? j["value0"] : j;
                    int modeVal = d.value("value1", -1);
                    int oemData = d.value("value2", 0);
                    const char* realName = nullptr;
                    for (auto& entry : knownModes)
                    {
                        if (entry.value == modeVal)
                        {
                            realName = entry.modeName;
                            break;
                        }
                    }
                    if (realName)
                    {
                        if (modeVal == 0x0B) // FFO
                        {
                            shortMode = "OEM: Fixed Frequency Override";
                            if (oemData != 0)
                            {
                                shortMode += " - ";
                                shortMode += std::to_string(oemData);
                                shortMode += "MHz";
                            }
                        }
                        else if (modeVal == 0x03) // SFP
                        {
                            std::ostringstream oss;
                            oss << "OEM: Static Frequency Point - 0x"
                                << std::hex << std::uppercase << std::setw(4)
                                << std::setfill('0') << oemData;
                            shortMode = oss.str();
                        }
                        else
                        {
                            shortMode = std::string("OEM: ") + realName;
                        }
                    }
                }
                catch (const std::exception&)
                {}
            }
        }

        std::cout << "Power Mode (D-Bus: " << powerModeInterface << "):\n";
        std::cout << "  PowerMode:         " << Ansi::bold() << shortMode
                  << Ansi::reset() << "\n";

        // SafeMode
        try
        {
            auto safeProp = open_power::occ::utils::getProperty(
                powerModePath, powerModeInterface, "SafeMode");
            bool safeMode = std::get<bool>(safeProp);
            std::cout << "  SafeMode:          "
                      << (safeMode ? Ansi::red() : "")
                      << (safeMode ? Ansi::bold() : "")
                      << (safeMode ? "true" : "false") << Ansi::reset() << "\n";
        }
        catch (const std::exception& e)
        {
            std::cout << "  SafeMode:          ERROR (" << e.what() << ")\n";
        }

        // AllowedPowerModes
        try
        {
            auto allowedProp = open_power::occ::utils::getProperty(
                powerModePath, powerModeInterface, "AllowedPowerModes");
            auto allowed = std::get<std::vector<std::string>>(allowedProp);
            std::cout << "  AllowedPowerModes: ";
            if (allowed.empty())
            {
                std::cout << "(any)\n";
            }
            else
            {
                for (size_t i = 0; i < allowed.size(); ++i)
                {
                    auto p = allowed[i].rfind('.');
                    std::string s = (p != std::string::npos)
                                        ? allowed[i].substr(p + 1)
                                        : allowed[i];
                    if (i > 0)
                        std::cout << "                     ";
                    std::cout << s << "\n";
                }
            }
        }
        catch (const std::exception& e)
        {
            std::cout << "  AllowedPowerModes: ERROR (" << e.what() << ")\n";
        }
    }
    catch (const std::exception& e)
    {
        std::cout << "Power Mode (D-Bus: " << powerModeInterface << "):\n";
        std::cout << "  PowerMode:         ERROR (" << e.what() << ")\n";
    }

    if (verbose)
    {
        std::string persistFile =
            std::string(OCC_CONTROL_PERSIST_PATH) + "/powerModeData";
        std::cout << "\nPersisted Power Mode Data (" << persistFile << "):\n";
        if (fs::exists(persistFile))
        {
            std::ifstream file(persistFile);
            if (file.is_open())
            {
                std::string line;
                while (std::getline(file, line))
                {
                    std::cout << "  " << line << "\n";
                }
            }
            else
            {
                std::cout << "  ERROR: Unable to open " << persistFile << "\n";
            }
        }
        else
        {
            std::cout << "  File does not exist\n";
        }
    }

    return 0;
}

int introspectPath(const std::string& path, const std::string& interface)
{
    std::cout << "Introspecting " << path << " for " << interface << ":\n";
    std::string service;
    try
    {
        service = open_power::occ::utils::getService(path, interface);
    }
    catch (const std::exception& e)
    {
        std::cerr << "  Failed to find service for " << path << " ("
                  << interface << "): " << e.what() << "\n\n";
        return 1;
    }

    std::string cmd =
        "busctl introspect --full " + service + " " + path + " " + interface;
    auto rc = runSystemCmd(cmd);
    if (rc != 0)
    {
        std::cerr << "  Failed to introspect " << path << " (exit code: " << rc
                  << ")\n";
    }
    std::cout << "\n";
    return rc;
}

int doIntrospect()
{
    int rc = 0;
    rc |= introspectPath("/org/open_power/control/chassis1/occ0",
                         "org.open_power.OCC.Status");
    rc |= introspectPath("/xyz/openbmc_project/control/host0/power_mode",
                         "xyz.openbmc_project.Control.Power.Mode");
    rc |= introspectPath("/xyz/openbmc_project/control/host0/power_ips",
                         "xyz.openbmc_project.Control.Power.IdlePowerSaver");
    rc |= introspectPath("/xyz/openbmc_project/control/host0/power_cap",
                         "xyz.openbmc_project.Control.Power.Cap");
    return rc == 0 ? 0 : 1;
}

int exitSafe()
{
    if (!requireHostRunning())
    {
        return 1;
    }

    std::cout << "Sending request to HTMGT to attempt to exit safe mode\n";
    std::fflush(stdout);

    auto rc = runSystemCmd("pldmtool raw -m 10 -d 0x80 0x3f 0x0f 5");
    if (rc != 0)
    {
        std::cerr << "Failed to send exit safe command (exit code: " << rc
                  << ")\n";
        return 1;
    }
    return 0;
}

int dumpSensors()
{
    const std::string cmd =
        "dbus-send --system --print-reply --dest=org.open_power.OCC.Control "
        "/xyz/openbmc_project/sensors org.freedesktop.DBus.ObjectManager.GetManagedObjects | "
        "grep -e \"object path\" -e variant | "
        "grep -v -e 'inf$' -e 'boolean true' -e string -e array | "
        "sed 's/ *object path *\\([^\\s]*\\)/\\1/' | "
        "sed 's/ *variant *double //g' | "
        "awk 'NR%2{printf \"%s \\t\",$0;next;}1' | "
        "sort -";

    auto rc = runSystemCmd(cmd);
    if (rc != 0)
    {
        std::cerr << "Failed to dump sensors (exit code: " << rc << ")\n";
        return 1;
    }
    return 0;
}

#endif // UNIT_TEST

} // namespace

namespace occctl
{

// Parse a busctl "ai <count> <v0> <v1> ..." response into a byte vector.
// Returns true on success.
bool parseBusctlArrayResponse(const std::string& busctlOutput,
                              std::vector<uint8_t>& bytes)
{
    bytes.clear();

    std::istringstream ss(busctlOutput);
    std::string token;

    // First token must be "ai"
    if (!(ss >> token) || token != "ai")
    {
        return false;
    }

    // Second token is the count
    int count = 0;
    if (!(ss >> count) || (count < 0) || (count > 65536))
    {
        return false;
    }

    bytes.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i)
    {
        int val = 0;
        if (!(ss >> val))
        {
            bytes.clear();
            return false;
        }
        bytes.push_back(static_cast<uint8_t>(val & 0xFF));
    }
    return true;
}

// ---- OCC poll response helpers ----

const char* getPollStatusStr(uint8_t flags)
{
    static thread_local char buf[256];
    buf[0] = '\0';
    if (flags & 0x80)
        strcat(buf, "Master ");
    if (flags & 0x10)
        strcat(buf, "OCCPmcrOwner ");
    if (flags & 0x08)
        strcat(buf, "SIMICS ");
    if (flags & 0x04)
        strcat(buf, "DDR5Workaround ");
    if (flags & 0x02)
        strcat(buf, "ObsReady ");
    if (flags & 0x01)
        strcat(buf, "ActReady ");
    if (flags & 0x60)
        strcat(buf, "UNKNOWN ");
    size_t len = strlen(buf);
    if (len > 0 && buf[len - 1] == ' ')
        buf[len - 1] = '\0';
    return buf;
}

const char* getPollExtStatusStr(uint8_t flags)
{
    static thread_local char buf[256];
    buf[0] = '\0';
    if (flags & 0x80)
        strcat(buf, "Throttle-ProcOverTemp ");
    if (flags & 0x40)
        strcat(buf, "Throttle-Power ");
    if (flags & 0x20)
        strcat(buf, "MemThrot-OverTemp ");
    if (flags & 0x10)
        strcat(buf, "QuickPowerDrop ");
    if (flags & 0x08)
        strcat(buf, "Throttle-VddOverTemp ");
    if (flags & 0x04)
        strcat(buf, "GPU2-Throttled ");
    if (flags & 0x02)
        strcat(buf, "GPU1-Throttled ");
    if (flags & 0x01)
        strcat(buf, "GPU0-Throttled ");
    size_t len = strlen(buf);
    if (len > 0 && buf[len - 1] == ' ')
        buf[len - 1] = '\0';
    return buf;
}

const char* getOccStateStr(uint8_t state)
{
    switch (state)
    {
        case 0x01:
            return "STANDBY";
        case 0x02:
            return "OBSERVATION";
        case 0x03:
            return "ACTIVE";
        case 0x04:
            return "SAFE";
        case 0x05:
            return "CHARACTERISTIC";
        case 0x85:
            return "RESET";
        case 0x87:
            return "TRANSITION";
        case 0x88:
            return "LOADING";
        default:
            return "UNKNOWN";
    }
}

const char* getOccModeStr(uint8_t mode)
{
    switch (mode)
    {
        case 0x01:
            return "OEM: Static";
        case 0x02:
            return "OEM: Non-Deterministic";
        case 0x03:
            return "OEM: StaticFreqPoint";
        case 0x04:
            return "SAFE";
        case 0x05:
            return "PowerSaving";
        case 0x06:
            return "EfficiencyFavorPower";
        case 0x07:
            return "OEM: EfficiencyFavorPerf";
        case 0x09:
            return "OEM: MaxFrequency";
        case 0x0A:
            return "OEM: BalancedPerformance";
        case 0x0B:
            return "OEM: FixedFrequency";
        case 0x0C:
            return "MaxPerformance";
        default:
            return "UNKNOWN";
    }
}

const char* getRspStatusStr(uint8_t status)
{
    switch (status)
    {
        case 0x00:
            return "SUCCESS";
        case 0x01:
            return "CONDITIONAL_SUCCESS";
        case 0x11:
            return "INVALID_COMMAND";
        case 0x12:
            return "INVALID_CMD_LENGTH";
        case 0x13:
            return "INVALID_DATA";
        case 0x14:
            return "CHECKSUM_FAIL";
        case 0x15:
            return "INTERNAL_FAIL";
        case 0x16:
            return "INVALID_STATE";
        case 0x17:
            return "NO_SUPPORT_IN_SMF_MODE";
        case 0xE0:
            return "EXCEPTION-PANIC";
        case 0xE1:
            return "EXCEPTION-INIT_CHECKPOINT";
        case 0xE2:
            return "EXCEPTION-WATCHDOG_TIMER";
        case 0xE3:
            return "EXCEPTION-OCB_TIMER";
        case 0xE5:
            return "EXCEPTION-INIT_FAILURE";
        case 0xFF:
            return "CMD_IN_PROGRESS";
        default:
            return "UNKNOWN";
    }
}

const char* getFruStr(uint8_t fru)
{
    switch (fru)
    {
        case 0:
            return "core";
        case 1:
            return "membuf";
        case 2:
            return "dimm";
        case 3:
            return "memctrl-dram";
        case 4:
            return "gpu";
        case 5:
            return "gpu-mem";
        case 6:
            return "vrm-vdd";
        case 7:
            return "pmic";
        case 8:
            return "memctrl-ext";
        case 9:
            return "proc-io";
        case 0xA:
            return "proc-mma";
        case 0xF0:
            return "proc-delta";
        case 0xF9:
            return "proc-io-delta";
        default:
            return "";
    }
}

uint16_t be16(const uint8_t* p)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

uint32_t be32(const uint8_t* p)
{
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
}

// Lowercase hex string of n bytes (no separators).
std::string hexBytes(const uint8_t* p, size_t n)
{
    std::ostringstream ss;
    ss << std::hex << std::setfill('0');
    for (size_t i = 0; i < n; ++i)
    {
        ss << std::setw(2) << static_cast<int>(p[i]);
    }
    return ss.str();
}

// Join the names of all set flags (space separated).
std::string flagNames(uint32_t flags,
                      const std::vector<std::pair<uint32_t, const char*>>& tbl)
{
    std::string result;
    for (const auto& [mask, name] : tbl)
    {
        if (flags & mask)
        {
            if (!result.empty())
            {
                result += " ";
            }
            result += name;
        }
    }
    return result;
}

// from src/occ_405/cmdh/cmdh_fsp_cmds.h
std::string getWofReason(uint32_t flags)
{
    return flagNames(
        flags,
        {{0x00000001, "NO_WOF_HEADER_MASK"},
         {0x00000002, "ILLEGAL_DIM_VALUE"},
         {0x00000004, "BAD_WFT_CEFFRT_BOUNDS"},
         {0x00000008, "INVALID_VDD_VDN"},
         {0x00000010, "PGPE_WOF_DISABLED"},
         {0x00000020, "PSTATE_PROTOCOL_OFF"},
         {0x00000040, "CEFFINFO_REQ_TIMEOUT"},
         {0x00000080, "CONTROL_REQ_TIMEOUT"},
         {0x00000100, "STATE_CHANGE"},
         {0x00000200, "THERMAL_WOF"},
         {0x00000400, "MODE_NO_SUPPORT_MASK"},
         {0x00000800, "DIVIDE_BY_ZERO_VDD"},
         {0x00001000, "CEFFINFO_REQ_FAILURE"},
         {0x00002000, "CONTROL_REQ_FAILURE"},
         {0x00004000, "VRT_ALIGNMENT_ERROR"},
         {0x00010000, "UTURBO_IS_ZERO"},
         {0x00020000, "OCC_WOF_DISABLED"},
         {0x00040000, "GPPB_WOF_DISABLED"},
         {0x00080000, "SYSTEM_WOF_DISABLE"},
         {0x00100000, "RESET_LIMIT_REACHED"},
         {0x00400000, "NO_CONFIGURED_CORES"},
         {0x00800000, "IPC_FAILURE"},
         {0x02000000, "RESET_DEBUG_CMD"},
         {0x40000000, "STALE_TAP_DATA"}});
}

// from src/occ_405/amec/amec_freq.h
std::string getClipReason(uint32_t flags)
{
    return flagNames(
        flags,
        {{0x00000001, "SOCKET_VDD_CAP"},
         {0x00000002, "TOT_SOCKET_CAP"},
         {0x00000040, "PROC_THRM"},
         {0x00000200, "OVER_CURRENT"},
         {0x00000400, "OVERRIDE"},
         {0x00002000, "PPB_HARD_CAP"},
         {0x00004000, "PPB_SOFT_CAP"},
         {0x00008000, "UTIL/DPM"},
         {0x00020000, "OVERRIDE_CORE"},
         {0x00040000, "IPS"},
         {0x00080000, "APSS_PMAX"},
         {0x00100000, "VDD_THRM"},
         {0x00200000, "EFF_MODE_IDLE_UTIL"},
         {0x00400000, "EFF_MODE_IDLE_CEFF"}});
}

// from src/occ_405/wof/wof.h
std::string getWofAdjustmentReason(uint8_t flags)
{
    return flagNames(flags, {{0x01, "CEFF_OVERCURRENT"},
                             {0x02, "CEFF_THROTTLE"},
                             {0x04, "CEFF_OVERVOLT"},
                             {0x08, "CEFF_EFFICIENCY_MODE"},
                             {0x20, "AMBIENT_NO_AMBIENT"},
                             {0x40, "AMBIENT_ALTITUDE"},
                             {0x80, "AMBIENT_DIMM"}});
}

std::string getDimmDisableReason(uint8_t reason)
{
    return flagNames(reason, {{0x01, "WOF_TABLE_SIZE"},
                              {0x02, "0_THERMAL_CONSTANT"},
                              {0x04, "0_MAX_PREHEAT_PWR"},
                              {0x08, "INTERPOLATION"},
                              {0x10, "OCMB_DATA_MISMATCH"},
                              {0x20, "OCMB_PRESENCE_MISMATCH"}});
}

// from src/occ_405/errl/errl.h
const char* getHistoryName(uint8_t id)
{
    switch (id)
    {
        case 0x01:
            return "NoOCMBdts";
        case 0x02:
            return "EmpathFail";
        case 0x03:
            return "NUTIL0Freq";
        case 0x07:
            return "VddReadFail";
        case 0x09:
            return "VddOverCurrent";
        case 0x0B:
            return "InvalidApssData";
        case 0x0C:
            return "ApssCompleteErr";
        case 0x0D:
            return "ApssTimeout";
        case 0x0E:
            return "DcomTxSlvInbox";
        case 0x0F:
            return "DcomRxSlvInbox";
        case 0x10:
            return "DcomTxSlvOutbox";
        case 0x11:
            return "DcomRxSlvOutbox";
        case 0x12:
            return "DcomMstPbaxSend";
        case 0x13:
            return "DcomSlvPbaxSend";
        case 0x14:
            return "DcomMstPbaxRead";
        case 0x15:
            return "DcomSlvPbaxRead";
        case 0x16:
            return "Gpe0NotIdle";
        case 0x17:
            return "Gpe1NotIdle";
        case 0x18:
            return "24x7Disabled";
        case 0x19:
            return "CeffRatioVdd";
        case 0x1A:
            return "VddTemp";
        case 0x1B:
            return "OverPcapIgnored";
        case 0x1C:
            return "VFRTTimeoutIgn";
        case 0x1D:
            return "WOFControlTimeoutIgn";
        case 0x1E:
            return "PstateChangeIgnored";
        case 0x1F:
            return "VddCurrentRolloverMax";
        case 0x20:
            return "CoreSmallDroop";
        case 0x21:
            return "CoreLargeDroop";
        case 0x22:
            return "OCSDirtyBlock";
        case 0x23:
            return "RtlTimeExceeded";
        case 0x24:
            return "DcomSlvInIncomplete";
        case 0x25:
            return "DcomSlvLostConn";
        case 0x26:
            return "24x7NotIdle";
        default:
            return "???";
    }
}

// Decode one 12-byte EXTN sensor entry (name[4], flags, byte5, data[6]).
// pollMode is the OCC mode from the poll header; foundMode is set when a MODE
// entry has been seen (used by WOFC).
void printExtnEntry(const uint8_t* s, uint8_t pollMode, bool& foundMode,
                    std::ostream& out)
{
    const uint32_t name = be32(s);
    const uint8_t flags = s[4];
    const std::string raw = hexBytes(&s[6], 6);

    out << "                   ";
    if (flags & 1)
    {
        out << " " << hexBytes(s, 4) << "  " << hexBytes(&s[4], 1) << " "
            << hexBytes(&s[5], 1) << " " << raw;
        return;
    }

    char nameStr[5];
    snprintf(nameStr, sizeof(nameStr), "%.4s",
             reinterpret_cast<const char*>(s));
    out << "\"" << std::left << std::setfill(' ') << std::setw(4) << nameStr
        << std::right << "\" " << hexBytes(&s[4], 1) << " "
        << hexBytes(&s[5], 1) << " ";

    char buf[256];
    switch (name)
    {
        case 0x464D494E: // FMIN
        case 0x46444953: // FDIS
        case 0x46424153: // FBAS
        case 0x46555400: // FUT
        case 0x464D4158: // FMAX
            snprintf(buf, sizeof(buf), "%s    pState: %3d / %d MHz",
                     raw.c_str(), s[6], be16(&s[7]));
            out << buf;
            break;

        case 0x434C4950: // CLIP
        {
            const uint8_t pstate = s[6];
            const uint8_t clipCount = s[7];
            const uint32_t reason = be32(&s[8]);
            const std::string reasonStr = getClipReason(reason);
            out << raw << "    ";
            if (pstate == 0 || clipCount == 0)
            {
                if (reason == 0)
                {
                    out << "OCC is NOT clipping the pstate";
                }
                else
                {
                    out << "OCC is NOT clipping (prior clip pState: "
                        << static_cast<int>(pstate) << ", reason:" << reasonStr
                        << ")";
                }
            }
            else
            {
                out << "OCC clip pState: " << static_cast<int>(pstate)
                    << ", count:" << static_cast<int>(clipCount)
                    << ", reason:" << reasonStr;
            }
            break;
        }

        case 0x574F4643: // WOFC
        {
            const uint8_t pstate = s[6];
            out << raw << "    ";
            if (pstate != 0xFF)
            {
                const uint16_t vratio = be16(&s[9]);
                snprintf(buf, sizeof(buf), "WOF pState: %3d, ", pstate);
                out << buf;
                if (foundMode)
                {
                    snprintf(buf, sizeof(buf),
                             "VddCeff: %.2f%%, vRatio: 0x%04X",
                             be16(&s[7]) / 100.0, vratio);
                    out << buf;
                    if (s[11] != 0)
                    {
                        out << ", Adjustments: "
                            << getWofAdjustmentReason(s[11]);
                    }
                }
                else
                {
                    snprintf(buf, sizeof(buf), "vRatio: 0x%04X", vratio);
                    out << buf;
                }
            }
            else
            {
                out << "WOF Disabled: " << getWofReason(be32(&s[8]));
            }
            break;
        }

        case 0x574F4649: // WOFI
        {
            const uint8_t vrt1 = s[6];
            const uint8_t vrt2 = s[7];
            const uint8_t vrt3 = s[8];
            const uint8_t dimmDisable = s[9];
            const int8_t ambAdjust = static_cast<int8_t>(s[10]);
            const int ioIdx = ((vrt1 & 0x0F) << 1) | ((vrt2 & 0x80) >> 7);
            const int ambIdx = (vrt2 & 0x7C) >> 2;
            const int vddIdx = vrt3 & 0x1F;
            out << raw << "    IO idx: " << ioIdx << ", Amb idx: " << ambIdx
                << ", Vdd idx: " << vddIdx << ", ";
            if (dimmDisable != 0)
            {
                out << "DIMM Disable: " << getDimmDisableReason(dimmDisable);
            }
            else
            {
                snprintf(buf, sizeof(buf), "dimm ambient adjust: %.1f",
                         ambAdjust / 10.0);
                out << buf;
            }
            break;
        }

        case 0x4D4F4445: // MODE
        {
            foundMode = true;
            const uint16_t freq = be16(&s[6]);
            out << raw << "    ";
            if (pollMode == 0x03 || pollMode == 0x0B) // SFP or FFO
            {
                out << "Frequency: " << freq << " MHz";
            }
            else
            {
                snprintf(buf, sizeof(buf),
                         "Freq limit: %d MHz, Vdd Ceff addr: %.2f%%, "
                         "Raw Ceff: %.2f%%",
                         freq, be16(&s[8]) / 100.0, be16(&s[10]) / 100.0);
                out << buf;
            }
            break;
        }

        case 0x45525248: // ERRH
            for (int i = 6; i <= 10; i += 2)
            {
                if (s[i])
                {
                    out << "0x" << hexBytes(&s[i], 1) << "="
                        << getHistoryName(s[i]) << ":"
                        << static_cast<int>(s[i + 1]) << "  ";
                }
            }
            break;

        case 0x5057524D: // PWRM
            snprintf(buf, sizeof(buf),
                     "%s    OCMB Status: 0x%04X, GSID: 0x%04X, "
                     "Total mem power: %dW",
                     raw.c_str(), be16(&s[6]), be16(&s[8]), be16(&s[10]));
            out << buf;
            break;

        case 0x50575250: // PWRP
            snprintf(buf, sizeof(buf),
                     "%s    reserved: 0x%04X,    GSID: 0x%04X, "
                     "Proc chip power: %dW",
                     raw.c_str(), be16(&s[6]), be16(&s[8]), be16(&s[10]));
            out << buf;
            break;

        default:
            out << raw;
            break;
    }
}

// Parse the OCC poll response body (response header seq/cmd/status/len already
// stripped). rsp points to at least 40 bytes of poll body data.
void parsePollResponse(const uint8_t* rsp, uint16_t rspLen, std::ostream& out)
{
    if (rspLen < 40)
    {
        std::cerr << "ERROR: Poll response too short (" << rspLen
                  << " bytes, expected at least 40)\n";
        return;
    }

    uint8_t status = rsp[0];
    uint8_t extStatus = rsp[1];
    uint8_t occPresMask = rsp[2];
    uint8_t configData = rsp[3];
    uint8_t state = rsp[4];
    uint8_t mode = rsp[5];
    uint8_t ipsStatus = rsp[6];
    uint8_t errlId = rsp[7];
    uint32_t errlAddr = be32(&rsp[8]);
    uint16_t errlLen = be16(&rsp[12]);
    uint8_t errlSrc = rsp[14];
    uint8_t gpuPresence = rsp[15];

    char occLevel[17];
    snprintf(occLevel, sizeof(occLevel), "%.16s",
             reinterpret_cast<const char*>(&rsp[16]));
    char sensorTag[7];
    snprintf(sensorTag, sizeof(sensorTag), "%.6s",
             reinterpret_cast<const char*>(&rsp[32]));

    uint8_t numBlocks = rsp[38];
    uint8_t sensorDblkVer = rsp[39];
    bool isMaster = (status & 0x80) != 0;

    out << "    Status: 0x" << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(status) << std::dec << "  "
        << getPollStatusStr(status) << "\n";
    out << "Ext Status: 0x" << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(extStatus) << std::dec << "  "
        << getPollExtStatusStr(extStatus) << "\n";
    out << "OCCs Prsnt: 0x" << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(occPresMask) << std::dec << "\n";
    out << "Confg Reqd: 0x" << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(configData) << std::dec << "\n";
    out << "     State: 0x" << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(state) << std::dec << "  " << getOccStateStr(state)
        << "\n";
    out << "      Mode: 0x" << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(mode) << std::dec << "  " << getOccModeStr(mode)
        << "\n";

    if (isMaster)
    {
        bool ipsEnabled = (ipsStatus & 0x01) != 0;
        bool ipsActive = (ipsStatus & 0x02) != 0;
        out << "IPS Status: 0x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(ipsStatus) << std::dec << "  "
            << (ipsEnabled ? "ENABLED" : "DISABLED")
            << (ipsActive ? " and ACTIVE" : "") << "\n";
    }
    else
    {
        out << "IPS Status: 0x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(ipsStatus) << std::dec << "  N/A\n";
    }

    out << "   Elog ID: 0x" << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(errlId) << std::dec
        << (errlId ? "" : "  (no error)") << "\n";
    out << " Elog Addr: 0x" << std::hex << std::setw(8) << std::setfill('0')
        << errlAddr << std::dec << "\n";
    out << "  Elog Len: 0x" << std::hex << std::setw(4) << std::setfill('0')
        << errlLen << std::dec << "\n";
    out << "  Elog Src: 0x" << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(errlSrc) << std::dec << "\n";
    out << "   GPU Cfg: 0x" << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(gpuPresence) << std::dec << "\n";
    out << "Code Level: \"" << occLevel << "\"\n";
    out << "Sensor Tag: \"" << sensorTag << "\"\n";
    out << "  # Blocks: 0x" << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(numBlocks) << std::dec << "\n";
    out << " Sens Vers: 0x" << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(sensorDblkVer) << std::dec << "\n";

    // Sensor data blocks start at byte 40
    const uint8_t* dblock = &rsp[40];
    uint16_t remaining = (rspLen > 40) ? static_cast<uint16_t>(rspLen - 40) : 0;
    uint16_t index = 0;
    uint8_t blocksLeft = numBlocks;
    bool foundMode = false;

    while (blocksLeft > 0 && index + 8 <= remaining)
    {
        const uint8_t* bh = &dblock[index];
        uint8_t sensorLen = bh[6];
        uint8_t numSens = bh[7];
        uint16_t sindex = index + 8;

        if (sensorLen == 0)
        {
            std::cerr << "WARNING: Sensor block has zero-length entries; "
                         "stopping decode\n";
            break;
        }

        char tag[5];
        snprintf(tag, sizeof(tag), "%.4s", reinterpret_cast<const char*>(bh));

        out << "    Sensor: " << tag << " - format:" << static_cast<int>(bh[4])
            << ", " << static_cast<int>(numSens) << " sensors ("
            << static_cast<int>(sensorLen) << " bytes/sensor)\n";

        if (strncmp(tag, "TEMP", 4) == 0)
        {
            out << "                   SSSSSSSS FF TT LL EE"
                   " (SSSS=Sensor ID, FF=FRU type, TT=temp in C,"
                   " LL=throttle, EE=error)\n";
            bool tracedLimits[16] = {};
            uint8_t n = numSens;
            while (n > 0 && sindex + sensorLen <= remaining)
            {
                const uint8_t* s = &dblock[sindex];
                uint8_t fruType = s[4];
                if (fruType != 0xFF)
                {
                    out << "                   " << std::hex
                        << std::setfill('0') << std::setw(2)
                        << static_cast<int>(s[0]) << std::setw(2)
                        << static_cast<int>(s[1]) << std::setw(2)
                        << static_cast<int>(s[2]) << std::setw(2)
                        << static_cast<int>(s[3]) << " " << std::setw(2)
                        << static_cast<int>(fruType) << " " << std::setw(2)
                        << static_cast<int>(s[5]) << " " << std::setw(2)
                        << static_cast<int>(s[6]) << " " << std::setw(2)
                        << static_cast<int>(s[7]) << std::dec;
                    if (s[5] != 0xFF)
                    {
                        if ((fruType < 16) && !tracedLimits[fruType])
                        {
                            tracedLimits[fruType] = true;
                            out << " (" << std::setfill(' ') << std::setw(3)
                                << static_cast<int>(s[5]) << "C " << std::left
                                << std::setw(12) << getFruStr(fruType)
                                << std::right << "  DVFS: " << std::setw(3)
                                << static_cast<int>(s[6])
                                << ", ERROR: " << std::setw(3)
                                << static_cast<int>(s[7]) << ")";
                        }
                        else
                        {
                            out << " (" << static_cast<int>(s[5]) << "C "
                                << getFruStr(fruType) << ")";
                        }
                    }
                    else
                    {
                        out << " (ERROR " << getFruStr(fruType) << ")";
                    }
                    out << "\n";
                }
                sindex += sensorLen;
                --n;
            }
        }
        else if (strncmp(tag, "EXTT", 4) == 0)
        {
            out << "                   SSSSSSSS FF TT"
                   " (SSSS=Sensor ID, FF=FRU type, TT=temp in C)\n";
            uint8_t n = numSens;
            while (n > 0 && sindex + sensorLen <= remaining)
            {
                const uint8_t* s = &dblock[sindex];
                uint8_t fruType = s[4];
                if (fruType != 0xFF)
                {
                    out << "                   " << std::hex
                        << std::setfill('0') << std::setw(2)
                        << static_cast<int>(s[0]) << std::setw(2)
                        << static_cast<int>(s[1]) << std::setw(2)
                        << static_cast<int>(s[2]) << std::setw(2)
                        << static_cast<int>(s[3]) << " " << std::setw(2)
                        << static_cast<int>(fruType) << " " << std::setw(2)
                        << static_cast<int>(s[5]) << std::dec;
                    if (s[5] != 0xFF)
                    {
                        out << " (" << std::setfill(' ') << std::setw(3)
                            << static_cast<int>(s[5]) << "C "
                            << getFruStr(fruType) << ")";
                    }
                    else
                    {
                        out << " (ERROR " << getFruStr(fruType) << ")";
                    }
                    out << "\n";
                }
                sindex += sensorLen;
                --n;
            }
        }
        else if (strncmp(tag, "FREQ", 4) == 0)
        {
            out << "                   SSSSSSSS FFFF"
                   "  (SSSS=Sensor ID, FFFF=freq in MHz)\n";
            uint8_t n = numSens;
            while (n > 0 && sindex + sensorLen <= remaining)
            {
                const uint8_t* s = &dblock[sindex];
                uint32_t sid = be32(s);
                uint16_t mhz = be16(&s[4]);
                out << "                   " << std::hex << std::setfill('0')
                    << std::setw(8) << sid << " " << std::setw(4) << mhz
                    << std::dec;
                if (mhz)
                    out << "  (" << mhz << " MHz)";
                out << "\n";
                sindex += sensorLen;
                --n;
            }
        }
        else if (strncmp(tag, "POWR", 4) == 0)
        {
            out << "                   SSSSSSSS FF CH rrrr TTTTTTTT"
                   " AAAAAAAAAAAAAAAA CCCC\n";
            out << "                               (SS=Sensor ID, FF=Function ID,"
                   " CH=APSS Channel, TT=Update Tag,\n";
            out << "                                AA=Accumulator,"
                   " CC=current reading in W)\n";
            uint8_t n = numSens;
            while (n > 0 && sindex + sensorLen <= remaining)
            {
                const uint8_t* s = &dblock[sindex];
                uint32_t sid = be32(s);
                uint16_t cur = be16(&s[20]);
                out << "                   " << std::hex << std::setfill('0')
                    << std::setw(8) << sid << " " << std::setw(2)
                    << static_cast<int>(s[4]) << " " << std::setw(2)
                    << static_cast<int>(s[5]) << " " << std::setw(4)
                    << be16(&s[6]) << " " << std::setw(8) << be32(&s[8]) << " "
                    << std::setw(8) << be32(&s[12]) << std::setw(8)
                    << be32(&s[16]) << " " << std::setw(4) << cur << std::dec
                    << "  (" << std::setw(5) << cur << " W)\n";
                sindex += sensorLen;
                --n;
            }
        }
        else if (strncmp(tag, "CAPS", 4) == 0 &&
                 sindex + sensorLen <= remaining)
        {
            const uint8_t* s = &dblock[sindex];
            uint16_t cap = be16(&s[0]);
            out << "                      Current Power Cap: " << std::hex
                << std::setw(4) << std::setfill('0') << cap << std::dec << "  ("
                << cap << " W)\n";
            cap = be16(&s[2]);
            out << "                   Current System Power: " << std::hex
                << std::setw(4) << std::setfill('0') << cap << std::dec << "  ("
                << cap << " W) (output power)\n";
            cap = be16(&s[4]);
            out << "                            N Power Cap: " << std::hex
                << std::setw(4) << std::setfill('0') << cap << std::dec << "  ("
                << cap << " W) (cap without redundant power)\n";
            cap = be16(&s[6]);
            out << "                   Max System Power Cap: " << std::hex
                << std::setw(4) << std::setfill('0') << cap << std::dec << "  ("
                << cap << " W)\n";
            cap = be16(&s[8]);
            out << "                     Hard Min Power Cap: " << std::hex
                << std::setw(4) << std::setfill('0') << cap << std::dec << "  ("
                << cap << " W)\n";
            if (sensorLen > 10)
            {
                cap = be16(&s[10]);
                out << "                     Soft Min Power Cap: " << std::hex
                    << std::setw(4) << std::setfill('0') << cap << std::dec
                    << "  (" << cap << " W)\n";
            }
            cap = be16(&s[12]);
            if (cap != 0)
            {
                out << "                       User Power Limit: " << std::hex
                    << std::setw(4) << std::setfill('0') << cap << std::dec
                    << "  (" << cap << " W)\n";
            }
            else
            {
                out << "                       User Power Limit: " << std::hex
                    << std::setw(4) << std::setfill('0') << cap << std::dec
                    << "  (DISABLED)\n";
            }
            uint8_t src = s[14];
            const char* srcName = (src == 1)   ? "(TMGT/BMC)"
                                  : (src == 2) ? "(OPAL)"
                                               : "";
            out << "                   User Power Limit Src: " << std::hex
                << std::setw(2) << std::setfill('0') << static_cast<int>(src)
                << std::dec << "  " << srcName << "\n";
            sindex += sensorLen;
        }
        else if (strncmp(tag, "EXTN", 4) == 0 && sensorLen >= 12)
        {
            out << "                   NNNNNNNN FF XX DDDDDDDDDDDD"
                   " (NNNN=Name/Sensor ID, FF=flags, XX=reserved,"
                   " DDDD=value)\n";
            uint8_t n = numSens;
            while (n > 0 && sindex + sensorLen <= remaining)
            {
                printExtnEntry(&dblock[sindex], mode, foundMode, out);
                out << "\n";
                sindex += sensorLen;
                --n;
            }
        }
        else
        {
            // Unknown block: hex dump
            uint16_t dumpLen = static_cast<uint16_t>(sensorLen * numSens);
            if (sindex + dumpLen > remaining)
                dumpLen = remaining - sindex;
            if (dumpLen > 0)
            {
                out << "                   (raw, " << dumpLen << " bytes):\n";
                for (uint16_t i = 0; i < dumpLen; ++i)
                {
                    if (i % 16 == 0)
                        out << "                   ";
                    out << std::hex << std::setfill('0') << std::setw(2)
                        << static_cast<int>(dblock[sindex + i]) << std::dec;
                    if (i % 16 == 15 || i + 1 == dumpLen)
                        out << "\n";
                    else
                        out << " ";
                }
            }
            sindex += dumpLen;
        }

        index = sindex;
        --blocksLeft;
    }

    if (blocksLeft > 0)
    {
        std::cerr << "WARNING: Response truncated; "
                  << static_cast<int>(blocksLeft)
                  << " sensor block(s) not decoded\n";
    }
}

// ---- Debug pass-through response parser (cmd 0x40) ----
//
// Sub-commands:
//   0x07 / 0x0C  - Get Multiple Sensor Data (basic: name[16]+gsid+cur+min+max,
//   24 bytes/sensor) 0xA7 / 0xAC  - Get Multiple Sensor Data with averages (30
//   bytes/sensor)

void parseDebugPassthruResponse(const uint8_t* cmdData, uint16_t cmdLen,
                                const uint8_t* rsp, uint16_t rspLen,
                                std::ostream& out)
{
    if (cmdLen == 0 || cmdData == nullptr)
    {
        std::cerr << "ERROR: No sub-command data for debug pass-through\n";
        return;
    }

    uint8_t subCmd = cmdData[0];

    switch (subCmd)
    {
        case 0x07: // Get Multiple Sensor Data
        case 0x0C: // Get Multiple Sensor Data (with clear)
        {
            if (rspLen < 2)
            {
                std::cerr << "ERROR: Debug PT response too short\n";
                return;
            }
            uint16_t numSensors = be16(rsp);
            out << "Number of sensors retrieved: " << numSensors << "\n";
            out << "Sensor              GSID  Current    Min     Max\n";
            out << "------------------------------------------------\n";

            // basic sensor: name[16] + gsid(2) + current(2) + min(2) + max(2) =
            // 24 bytes
            constexpr uint16_t SENSOR_SZ = 24;
            uint32_t offset = 2;
            uint16_t decoded = 0;
            while (decoded < numSensors && offset + SENSOR_SZ <= rspLen)
            {
                const uint8_t* s = &rsp[offset];
                char name[17];
                snprintf(name, sizeof(name), "%.16s",
                         reinterpret_cast<const char*>(s));
                uint16_t gsid = be16(&s[16]);
                uint16_t current = be16(&s[18]);
                uint16_t minVal = be16(&s[20]);
                uint16_t maxVal = be16(&s[22]);
                out << std::left << std::setw(16) << name << std::right
                    << "  0x" << std::hex << std::setw(4) << std::setfill('0')
                    << gsid << std::dec << "  " << std::setw(6) << current
                    << "  " << std::setw(6) << minVal << "  " << std::setw(6)
                    << maxVal << "\n";
                offset += SENSOR_SZ;
                ++decoded;
            }
            if (decoded < numSensors)
            {
                std::cerr << "WARNING: Response data too short for "
                          << numSensors << " sensors (decoded " << decoded
                          << ")\n";
            }
            break;
        }

        case 0xA7: // Get Multiple Sensor Data with averages
        case 0xAC: // Get Multiple Sensor Data with averages (with clear)
        {
            if (rspLen < 2)
            {
                std::cerr << "ERROR: Debug PT response too short\n";
                return;
            }
            uint16_t numSensors = be16(rsp);
            out << "Number of sensors retrieved: " << numSensors << "\n";
            out << "Sensor              GSID  Current    Min     Max     Avg    Samples\n";
            out << "------------------------------------------------------------------\n";

            // avg sensor: name[16] + gsid(2) + cur(2) + min(2) + max(2) +
            // avg(2) + count(4) = 30 bytes
            constexpr uint16_t SENSOR_SZ = 30;
            uint32_t offset = 2;
            uint16_t decoded = 0;
            while (decoded < numSensors && offset + SENSOR_SZ <= rspLen)
            {
                const uint8_t* s = &rsp[offset];
                char name[17];
                snprintf(name, sizeof(name), "%.16s",
                         reinterpret_cast<const char*>(s));
                uint16_t gsid = be16(&s[16]);
                uint16_t current = be16(&s[18]);
                uint16_t minVal = be16(&s[20]);
                uint16_t maxVal = be16(&s[22]);
                uint16_t avg = be16(&s[24]);
                uint32_t count = be32(&s[26]);
                out << std::left << std::setw(16) << name << std::right
                    << "  0x" << std::hex << std::setw(4) << std::setfill('0')
                    << gsid << std::dec << "  " << std::setw(6) << current
                    << "  " << std::setw(6) << minVal << "  " << std::setw(6)
                    << maxVal << "  " << std::setw(6) << avg << "  0x"
                    << std::hex << std::setw(8) << std::setfill('0') << count
                    << std::dec << "\n";
                offset += SENSOR_SZ;
                ++decoded;
            }
            if (decoded < numSensors)
            {
                std::cerr << "WARNING: Response data too short for "
                          << numSensors << " sensors (decoded " << decoded
                          << ")\n";
            }
            break;
        }

        default:
            out << "  Debug PT Sub Cmd: 0x" << std::hex << std::setw(2)
                << std::setfill('0') << static_cast<int>(subCmd) << std::dec
                << "  (no parser for this sub-command)\n";
            break;
    }
}

// ---- MFG test response parser (cmd 0x53) ----
//
// Sub-commands:
//   0x02  - Frequency Slew
//   0x06  - Get Sensor Details
//   0x09  - Memory Slew

void parseMfgTestResponse(const uint8_t* cmdData, uint16_t cmdLen,
                          const uint8_t* rsp, uint16_t rspLen,
                          std::ostream& out)
{
    if (cmdLen == 0 || cmdData == nullptr)
    {
        std::cerr << "ERROR: No sub-command data for MFG test\n";
        return;
    }

    uint8_t subCmd = cmdData[0];

    switch (subCmd)
    {
        case 0x02: // Frequency Slew
        {
            if (rspLen < 6)
            {
                std::cerr << "ERROR: Freq Slew response too short (" << rspLen
                          << " bytes)\n";
                return;
            }
            uint16_t slewCount = be16(&rsp[0]);
            uint16_t startPstate = be16(&rsp[2]);
            uint16_t stopPstate = be16(&rsp[4]);
            out << "  MFG Sub Cmd: 0x02  (Frequency Slew)\n\n";
            out << "   Slew Count: 0x" << std::hex << std::setw(4)
                << std::setfill('0') << slewCount << std::dec << " ("
                << std::setw(3) << slewCount << ")\n";
            out << " Start Pstate: 0x" << std::hex << std::setw(4)
                << std::setfill('0') << startPstate << std::dec << " ("
                << std::setw(3) << startPstate << ")  (lowest frequency)\n";
            out << "  Stop Pstate: 0x" << std::hex << std::setw(4)
                << std::setfill('0') << stopPstate << std::dec << " ("
                << std::setw(3) << stopPstate << ")  (highest frequency)\n";
            break;
        }

        case 0x06: // Get Sensor Details
        {
            // cmdh_mfg_get_sensor_resp_t (packed):
            //   gsid(2) + sample(2) + status(1) + accumulator(4) + min(2) +
            //   max(2)
            //   + name[16] + units[4] + freq(4) + scalefactor(4) + location(2)
            //   + type(2) + checksum(2)
            constexpr uint16_t MIN_SZ = 47;
            if (rspLen < MIN_SZ)
            {
                std::cerr << "ERROR: Get Sensor Details response too short ("
                          << rspLen << " bytes)\n";
                return;
            }
            const uint8_t* s = rsp;
            uint16_t gsid = be16(&s[0]);
            uint16_t sample = be16(&s[2]);
            uint8_t sensorStatus = s[4];
            uint32_t accumulator = be32(&s[5]);
            uint16_t minVal = be16(&s[9]);
            uint16_t maxVal = be16(&s[11]);
            char name[17];
            snprintf(name, sizeof(name), "%.16s",
                     reinterpret_cast<const char*>(&s[13]));
            char units[5];
            snprintf(units, sizeof(units), "%.4s",
                     reinterpret_cast<const char*>(&s[29]));
            uint32_t freq = be32(&s[33]);
            uint32_t scaleFactor = be32(&s[37]);
            uint16_t mantissa = static_cast<uint16_t>(scaleFactor >> 8);
            int16_t exponent = static_cast<int16_t>(scaleFactor & 0xFF);
            if (exponent > 128)
                exponent -= 256;
            uint16_t location = be16(&s[41]);
            uint16_t sensorType = be16(&s[43]);

            out << "  MFG Sub Cmd: 0x06  (Get Sensor Details)\n\n";
            out << "         GUID: 0x" << std::hex << std::setw(4)
                << std::setfill('0') << gsid << std::dec << "\n";
            out << "Latest sample: " << std::setw(6) << sample << " " << units
                << " (0x" << std::hex << std::setw(4) << std::setfill('0')
                << sample << std::dec << ")\n";
            out << "       Status: 0x" << std::hex << std::setw(2)
                << std::setfill('0') << static_cast<int>(sensorStatus)
                << std::dec << "\n";
            out << "  Accumulator: " << accumulator << " (0x" << std::hex
                << std::setw(8) << std::setfill('0') << accumulator << std::dec
                << ")\n";
            out << "   Min Sample: " << std::setw(6) << minVal << " " << units
                << " (0x" << std::hex << std::setw(4) << std::setfill('0')
                << minVal << std::dec << ")\n";
            out << "   Max Sample: " << std::setw(6) << maxVal << " " << units
                << " (0x" << std::hex << std::setw(4) << std::setfill('0')
                << maxVal << std::dec << ")\n";
            out << "         Name: " << name << "\n";
            out << "        Units: " << units << "\n";
            out << "  Update freq: " << freq << " (0x" << std::hex
                << std::setw(8) << std::setfill('0') << freq << std::dec
                << ")\n";
            out << " Scale Factor: 0x" << std::hex << std::setw(8)
                << std::setfill('0') << scaleFactor << std::dec << "  ("
                << mantissa << "x10^" << exponent << ")\n";
            out << " Sen Location: 0x" << std::hex << std::setw(4)
                << std::setfill('0') << location << std::dec << "\n";
            out << "  Sensor Type: 0x" << std::hex << std::setw(4)
                << std::setfill('0') << sensorType << std::dec << "\n";
            break;
        }

        case 0x09: // Memory Slew
        {
            if (rspLen < 2)
            {
                std::cerr << "ERROR: Memory Slew response too short (" << rspLen
                          << " bytes)\n";
                return;
            }
            uint16_t slewCount = be16(&rsp[0]);
            out << "  MFG Sub Cmd: 0x09  (Memory Slew)\n\n";
            out << "   Slew Count: 0x" << std::hex << std::setw(4)
                << std::setfill('0') << slewCount << std::dec << " ("
                << std::setw(3) << slewCount << ")\n";
            break;
        }

        default:
            out << "  MFG Sub Cmd: 0x" << std::hex << std::setw(2)
                << std::setfill('0') << static_cast<int>(subCmd) << std::dec
                << "  (no parser for this sub-command)\n";
            break;
    }
}

// ---- Common helper: send a busctl OCC command, capture response, strip header
// ----
//
// Sends the command, captures the busctl output, parses it into rspBytes
// (full array including 5-byte OCC response header), and prints the header
// unless printHeader is false.
// Returns false on any hard failure (transport error or OCC error status).
bool sendOccCommand(int chassis, int occ, uint8_t command,
                    const std::vector<uint8_t>& dataBytes,
                    std::vector<uint8_t>& rspBytes, bool printHeader = true)
{
    size_t totalElements = 3 + dataBytes.size();
    uint16_t dataLen = static_cast<uint16_t>(dataBytes.size());
    uint8_t dataLenHi = static_cast<uint8_t>((dataLen >> 8) & 0xFF);
    uint8_t dataLenLo = static_cast<uint8_t>(dataLen & 0xFF);

    std::ostringstream cmdStream;
    cmdStream << "busctl call org.open_power.OCC.Control"
              << " /org/open_power/control/chassis" << chassis << "/occ" << occ
              << " org.open_power.OCC.PassThrough Send ai " << totalElements
              << " " << static_cast<int>(command) << " "
              << static_cast<int>(dataLenHi) << " "
              << static_cast<int>(dataLenLo);
    for (auto b : dataBytes)
        cmdStream << " " << static_cast<int>(b);

    std::string output;
    int rc = captureCommand(cmdStream.str(), output);
    if (rc != 0)
    {
        std::cerr << "Failed to send OCC command (exit code: " << rc << ")\n";
        return false;
    }

    if (verbose)
    {
        std::cout << output;
        if (!output.empty() && output.back() != '\n')
        {
            std::cout << "\n";
        }
    }

    if (!parseBusctlArrayResponse(output, rspBytes))
    {
        std::cerr << "Failed to parse busctl response:\n" << output << "\n";
        return false;
    }

    if (rspBytes.size() < 5)
    {
        std::cerr
            << "ERROR: Response too short to contain OCC response header ("
            << rspBytes.size() << " bytes)\n";
        return false;
    }

    uint8_t seq = rspBytes[0];
    uint8_t rspCmd = rspBytes[1];
    uint8_t status = rspBytes[2];
    uint16_t dlen = static_cast<uint16_t>((rspBytes[3] << 8) | rspBytes[4]);

    if (printHeader)
    {
        std::cout << "OCC" << occ << " response (" << rspBytes.size()
                  << " bytes):\n";
        std::cout << "  Sequence: 0x" << std::hex << std::setw(2)
                  << std::setfill('0') << static_cast<int>(seq) << std::dec
                  << "\n";
        std::cout << "   Command: 0x" << std::hex << std::setw(2)
                  << std::setfill('0') << static_cast<int>(rspCmd) << std::dec
                  << "\n";
        std::cout << "    Status: 0x" << std::hex << std::setw(2)
                  << std::setfill('0') << static_cast<int>(status) << std::dec
                  << "  " << getRspStatusStr(status) << "\n";
        std::cout << "  Data Len: 0x" << std::hex << std::setw(4)
                  << std::setfill('0') << dlen << std::dec << "  (" << dlen
                  << ")\n";
    }

    if (status != 0x00 && status != 0x01)
    {
        std::cerr << "OCC returned error status: 0x" << std::hex
                  << static_cast<int>(status) << std::dec << " ("
                  << getRspStatusStr(status) << ")\n";
        return false;
    }

    return true;
}

int sendCmd(int chassis, int occ, uint8_t command,
            const std::vector<uint8_t>& dataBytes)
{
    if (!requireHostRunning() || !requireOccActive(chassis, occ))
    {
        return 1;
    }

    std::vector<uint8_t> rspBytes;
    if (!sendOccCommand(chassis, occ, command, dataBytes, rspBytes))
    {
        return 1;
    }

    const uint8_t* body = rspBytes.data() + 5;
    uint16_t bodyLen = static_cast<uint16_t>(rspBytes.size() - 5);

    if (command == CMD_POLL)
    {
        parsePollResponse(body, bodyLen);
    }
    else if (command == CMD_DEBUG_PT)
    {
        parseDebugPassthruResponse(dataBytes.data(),
                                   static_cast<uint16_t>(dataBytes.size()),
                                   body, bodyLen);
    }
    else if (command == CMD_MFG_TEST)
    {
        parseMfgTestResponse(dataBytes.data(),
                             static_cast<uint16_t>(dataBytes.size()), body,
                             bodyLen);
    }
    else
    {
        // Unknown command: hex dump the response body
        for (uint16_t i = 0; i < bodyLen; ++i)
        {
            if (i % 16 == 0)
            {
                std::cout << "  ";
            }
            std::cout << std::hex << std::setfill('0') << std::setw(2)
                      << static_cast<int>(body[i]) << std::dec;
            if (i % 16 == 15 || i + 1 == bodyLen)
            {
                std::cout << "\n";
            }
            else
            {
                std::cout << " ";
            }
        }
    }
    return 0;
}

// Returns true if SafeMode is set on the power_mode D-Bus object.
bool isSafeMode()
{
    constexpr auto powerModePath =
        "/xyz/openbmc_project/control/host0/power_mode";
    constexpr auto powerModeInterface =
        "xyz.openbmc_project.Control.Power.Mode";
    try
    {
        auto propVal = open_power::occ::utils::getProperty(
            powerModePath, powerModeInterface, "SafeMode");
        return std::get<bool>(propVal);
    }
    catch (const std::exception&)
    {
        return false;
    }
}

// Print lines in 'cur' that are not in 'prev', prefixed with a timestamp.
static void printNewLines(const std::string& cur, const std::string& prev)
{
    std::unordered_set<std::string> prevLines;
    std::istringstream prevSS(prev);
    std::string line;
    while (std::getline(prevSS, line))
    {
        prevLines.insert(line);
    }

    std::istringstream curSS(cur);
    while (std::getline(curSS, line))
    {
        if (!prevLines.contains(line))
        {
            time_t now = time(nullptr);
            struct tm tm_now{};
            localtime_r(&now, &tm_now);
            char tbuf[16];
            strftime(tbuf, sizeof(tbuf), "%H:%M:%S", &tm_now);
            std::cout << tbuf << " : " << Ansi::bold() << line << Ansi::reset()
                      << "\n";
        }
    }
}

// Returns true when no "^.* = 0$" lines remain in obmcutil power output.
static bool allPowerOn(const std::string& powerOut)
{
    constexpr std::string_view suffix = " = 0";
    std::istringstream ss(powerOut);
    std::string line;
    while (std::getline(ss, line))
    {
        // Line starts with a non-space char and ends with " = 0"
        if (line.size() > suffix.size() &&
            !std::isspace(static_cast<unsigned char>(line[0])) &&
            line.ends_with(suffix))
        {
            return false;
        }
    }
    return true;
}

// Wait (polling every 5 s) until the OCC reaches targetState (e.g. "ACTIVE").
// Checks the host state once at entry:
//   - If the host is NOT at runtime, OccActive being set implies the OCC is
//     already in the target state — return as soon as OccActive becomes true.
//   - If the host IS at runtime, the OCC state may still be transitioning, so
//     poll the OCC after OccActive is set until it reports the target state.
static int waitForOccState(const std::string& targetState, int timeoutSec = 600)
{
    constexpr int defaultChassis = open_power::occ::DEFAULT_CHASSIS;
    std::string occLabel =
        std::string(CHASSIS_NAME) + std::to_string(defaultChassis) + "/" +
        OCC_NAME + "0";
    std::string occPath = std::string(OCC_CONTROL_ROOT) + "/" + occLabel;
    constexpr auto occStatusInterface = "org.open_power.OCC.Status";

    // Capture host state once before waiting begins.
    bool hostRunning = checkHostRunning();

    std::cout << "Waiting for " << occLabel << " to reach " << Ansi::bold()
              << targetState << Ansi::reset()
              << " state (timeout: " << (timeoutSec / 60) << " min)\n";
    std::fflush(stdout);

    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::seconds(timeoutSec);

    // Phase 1: wait for OccActive to become true
    while (std::chrono::steady_clock::now() < deadline)
    {
        bool active = false;
        try
        {
            auto propVal = open_power::occ::utils::getProperty(
                occPath, occStatusInterface, "OccActive");
            active = std::get<bool>(propVal);
        }
        catch (const std::exception&)
        {}

        if (active)
        {
            time_t now = time(nullptr);
            struct tm tm_now{};
            localtime_r(&now, &tm_now);
            char tbuf[16];
            strftime(tbuf, sizeof(tbuf), "%H:%M:%S", &tm_now);
            std::cout << tbuf << " : OccActive sensor is true for " << occLabel
                      << "\n";
            std::fflush(stdout);
            break;
        }

        sleep(5);
    }

    if (std::chrono::steady_clock::now() >= deadline)
    {
        std::cerr << "Timeout waiting for OccActive (" << occLabel << ")\n";
        return 1;
    }

    // If the host was not at runtime and the target state is ACTIVE, OccActive
    // being set is sufficient — OCC is already ACTIVE
    if (!hostRunning && (targetState == "ACTIVE"))
    {
        time_t now = time(nullptr);
        struct tm tm_now{};
        localtime_r(&now, &tm_now);
        char tbuf[16];
        strftime(tbuf, sizeof(tbuf), "%H:%M:%S", &tm_now);
        std::cout << tbuf << " : OCC is ACTIVE\n";
        std::fflush(stdout);
        return 0;
    }

    // Phase 2: poll until the requested OCC state is reached
    std::string lastStateStr;
    while (std::chrono::steady_clock::now() < deadline)
    {
        std::vector<uint8_t> rspBytes;
        if (sendOccCommand(defaultChassis, 0, CMD_POLL, {0x20}, rspBytes,
                           /*printHeader=*/false) &&
            rspBytes.size() >= 10)
        {
            // Poll body starts at byte 5; state is body byte 4 → rspBytes[9]
            uint8_t state = rspBytes[9];
            std::string stateStr = getOccStateStr(state);

            if (stateStr != lastStateStr)
            {
                lastStateStr = stateStr;
                time_t now = time(nullptr);
                struct tm tm_now{};
                localtime_r(&now, &tm_now);
                char tbuf[16];
                strftime(tbuf, sizeof(tbuf), "%H:%M:%S", &tm_now);
                std::cout << tbuf << " : OCC state = " << stateStr << "\n";
                std::fflush(stdout);
            }

            if (stateStr == targetState)
                return 0;
        }
        sleep(5);
    }
    std::cerr << "Timeout waiting for OCC state " << targetState << "\n";
    return 1;
}

// powerOn(chassisOnly)
//   Powers on the host (chassisOnly=false) or chassis only (chassisOnly=true).
//   Tails the BMC console log while polling obmcutil power/status every 5 s.
//   Stops when all power values are non-zero and the host is running, or when
//   safe mode is detected, then waits for the OCC to reach ACTIVE state.
int powerOn(bool chassisOnly)
{
    bool waitActive = true;

    // Show current state before issuing power-on
    {
        std::string out;
        std::cout << "==> obmcutil power\n";
        std::fflush(stdout);
        captureCommand("obmcutil power 2>/dev/null", out);
        std::cout << out;
        out.clear();
        std::cout << "==> obmcutil status\n";
        std::fflush(stdout);
        captureCommand("obmcutil status 2>/dev/null", out);
        std::cout << out;
    }

    // Tail the BMC console log in a child process
    pid_t tailPid = -1;
    {
        time_t now = time(nullptr);
        struct tm tm_now{};
        localtime_r(&now, &tm_now);
        char tbuf[16];
        strftime(tbuf, sizeof(tbuf), "%H:%M:%S", &tm_now);
        std::cout << "==> tail -f /var/log/obmc-console.log &    (" << tbuf
                  << ")\n";
        std::fflush(stdout);
        tailPid = fork();
        if (tailPid == 0)
        {
            execlp("tail", "tail", "-f", "/var/log/obmc-console.log", "-n", "0",
                   nullptr);
            _exit(1);
        }
        if (tailPid > 0)
            std::cout << "    tail pid: " << tailPid << "\n";
        std::fflush(stdout);
    }

    auto killTail = [&]() {
        if (tailPid > 0)
        {
            kill(tailPid, SIGTERM);
            waitpid(tailPid, nullptr, 0);
            tailPid = -1;
        }
    };

    // Issue the power-on command
    {
        time_t now = time(nullptr);
        struct tm tm_now{};
        localtime_r(&now, &tm_now);
        char tbuf[16];
        strftime(tbuf, sizeof(tbuf), "%H:%M:%S", &tm_now);
        const char* ponCmd =
            chassisOnly ? "obmcutil chassison" : "obmcutil poweron";
        std::cout << Ansi::bold() << "==> " << ponCmd << Ansi::reset()
                  << "    (" << tbuf << ")\n";
        std::fflush(stdout);
        int rc = std::system(ponCmd);
        if (rc != 0)
        {
            std::cerr << Ansi::red() << "ERROR: " << ponCmd
                      << " failed with rc=" << rc << Ansi::reset() << "\n";
            killTail();
            return rc;
        }
    }

    // Poll every 5 s until power is fully on and host is running
    std::string lastPower, lastStatus;
    captureCommand("obmcutil power 2>/dev/null", lastPower);
    captureCommand("obmcutil status 2>/dev/null", lastStatus);

    while (true)
    {
        sleep(5);

        std::string curPower, curStatus;
        captureCommand("obmcutil power 2>/dev/null", curPower);
        captureCommand("obmcutil status 2>/dev/null", curStatus);

        if (curPower != lastPower)
            printNewLines(curPower, lastPower);
        if (curStatus != lastStatus)
            printNewLines(curStatus, lastStatus);
        std::fflush(stdout);

        bool powerDone = allPowerOn(curPower);
        bool hostUp = curStatus.find("HostState.Running") != std::string::npos;

        lastPower = curPower;
        lastStatus = curStatus;

        // Check for safe mode via D-Bus (avoids parsing journal output)
        if (isSafeMode())
        {
            time_t now = time(nullptr);
            struct tm tm_now{};
            localtime_r(&now, &tm_now);
            char tbuf[16];
            strftime(tbuf, sizeof(tbuf), "%H:%M:%S", &tm_now);
            std::cout << "  " << Ansi::red() << "System is in Safe Mode\t\t"
                      << tbuf << Ansi::reset() << "\n";
            std::fflush(stdout);
            break;
        }

        if (powerDone && hostUp)
            break;
    }

    if (waitActive)
    {
        std::cout << "\n";
        std::fflush(stdout);
        waitForOccState("ACTIVE");
    }

    if (tailPid > 0)
    {
        std::cout << "...killing " << tailPid << "\n";
    }
    killTail();
    return 0;
}

int stopService()
{
    verbose = true;
    std::cout << "Stopping org.open_power.OCC.Control.service...\n";
    std::fflush(stdout);
    auto rc = runSystemCmd("systemctl stop org.open_power.OCC.Control.service");
    if (rc != 0)
    {
        std::cerr << "Failed to stop occ-control service (exit code: " << rc
                  << ")\n";
        return 1;
    }
    std::cout << "Service stopped.\n";
    return 0;
}

int restartService()
{
    verbose = true;
    std::cout << "Restarting org.open_power.OCC.Control.service...\n";
    std::fflush(stdout);
    auto rc =
        runSystemCmd("systemctl restart org.open_power.OCC.Control.service");
    if (rc != 0)
    {
        std::cerr << "Failed to restart occ-control service (exit code: " << rc
                  << ")\n";
        return 1;
    }
    std::cout << "Service restarted.\n";
    return 0;
}

// Send OCC reset command (0x40, data decimal 20 82 83 84 0) to specified OCC.
int resetOcc(int chassis, int occ)
{
    if (!requireHostRunning() || !requireOccActive(chassis, occ))
    {
        return 1;
    }

    // Data bytes are decimal: 20, 82, 83, 84, 0
    const std::vector<uint8_t> resetData = {20, 82, 83, 84, 0};
    std::vector<uint8_t> rspBytes;
    if (!sendOccCommand(chassis, occ, CMD_DEBUG_PT, resetData, rspBytes))
    {
        return 1;
    }
    return 0;
}

// Fetch the ERR, IMP and INF OCC traces via debug pass-through and parse them
// on the BMC using the udparsers python module (errludP_occ.parseTrace).
int occTrace(int chassis, int occ)
{
    if (!requireHostRunning() || !requireOccActive(chassis, occ))
    {
        return 1;
    }

    const char* scriptFile = "/tmp/occ_trace_parse.py";

    auto cleanup = [&]() {
        for (const auto& t : traceList)
        {
            unlink(t.file);
        }
        unlink(scriptFile);
    };

    // Collect each trace; a failure on one does not prevent collecting the
    // others. Only fail if none could be collected.
    std::vector<const TraceType*> collectedTraces;
    for (const auto& t : traceList)
    {
        const std::vector<uint8_t> data = {
            0x03,
            0x01,
            0x00,
            static_cast<uint8_t>(t.id[0]),
            static_cast<uint8_t>(t.id[1]),
            static_cast<uint8_t>(t.id[2]),
            0x00};
        std::vector<uint8_t> rspBytes;
        std::cout << t.name << " trace:\n";
        bool ok = sendOccCommand(chassis, occ, CMD_DEBUG_PT, data, rspBytes);
        if (!ok)
        {
            std::cerr << "WARNING: Failed to collect " << t.name
                      << " trace; continuing\n";
            continue;
        }

        std::ofstream out(t.file, std::ios::binary | std::ios::trunc);
        if (!out.is_open())
        {
            std::cerr << "WARNING: Unable to create " << t.file << "; skipping "
                      << t.name << " trace\n";
            continue;
        }
        out.write(reinterpret_cast<const char*>(rspBytes.data()),
                  static_cast<std::streamsize>(rspBytes.size()));
        out.close();
        collectedTraces.push_back(&t);
    }

    if (collectedTraces.empty())
    {
        std::cerr << "ERROR: Failed to collect any OCC traces\n";
        cleanup();
        return 1;
    }

    // Each file holds the full response (5-byte header + trace data); the
    // script strips the header, parses the traces, drops the initial lines
    // and prints all remaining lines in sorted order.
    static const char* script = R"PY(
import sys
import os
from udparsers.b2a00.b2a00 import *
import udparsers.helpers.hostfw_trace as hostfw_trace
import udparsers.helpers.miscUtils as miscUtils

# Search the current directory first for the OCC string file
# (either "occStringFile" or "81e00687.lid"), before the system paths.
miscUtils.LID_SEARCH_DIRS.insert(0, os.getcwd())

# Older hostfw_trace versions crash (len(None)) when a trace hash is not in
# the string file. Return None for unknown hashes so the trace is reported
# as "NO STRING NO TRACE" instead.
_orig_get_hash_entry = hostfw_trace.get_hash_entry
def _safe_get_hash_entry(hashValue, lineNumber):
    if hostfw_trace.hTable.get(hashValue) is None:
        return None
    return _orig_get_hash_entry(hashValue, lineNumber)
hostfw_trace.get_hash_entry = _safe_get_hash_entry

# Arguments are (file, skipCount) pairs
if len(sys.argv) < 3 or len(sys.argv) % 2 != 1:
    print("Python incorrect number of arguments.")
    sys.exit(1)

final_trace = []
input_file = sys.argv[1::2]
input_skip = [int(x) for x in sys.argv[2::2]]

for file, skipCount in zip(input_file, input_skip):
    sz = os.path.getsize(file)
    if sz < 37:  # One trace is 32 bytes + 5 byte header
        print("no Traces in: " + file + " Size: " + str(sz))
        continue

    input_bytearray = bytearray(open(file, "rb").read())
    memory_view = memoryview(input_bytearray)
    memory_view = memory_view[5:]

    traceresults, to = errludP_occ.parseTrace(memory_view, 0)
    for skip, x in enumerate(traceresults):
        if skip > skipCount:
            final_trace.append(x)

final_trace.sort()
for x in final_trace:
    print(x)
)PY";

    {
        std::ofstream sf(scriptFile, std::ios::trunc);
        if (!sf.is_open())
        {
            std::cerr << "ERROR: Unable to create " << scriptFile << "\n";
            cleanup();
            return 1;
        }
        sf << script;
    }

    std::ostringstream cmd;
    cmd << "python3 " << scriptFile;
    for (const auto* t : collectedTraces)
    {
        cmd << " " << t->file << " " << t->skipLines;
    }
    auto rc = runSystemCmd(cmd.str());
    cleanup();
    if (rc != 0)
    {
        std::cerr << "Failed to parse OCC traces (exit code: " << rc << ")\n";
        return 1;
    }
    return 0;
}

const TraceType traceList[3] = {
    {"ERR", "/tmp/occ_trace_err_rsp.bin", {'E', 'R', 'R'}, 5},
    {"IMP", "/tmp/occ_trace_imp_rsp.bin", {'I', 'M', 'P'}, 7},
    {"INF", "/tmp/occ_trace_inf_rsp.bin", {'I', 'N', 'F'}, 10},
};

} // namespace occctl

using namespace occctl;

void printUsage(const char* appName)
{
    std::cout << "Usage: " << appName << " [options] <command> [args]\n\n";
    std::cout << "Options:\n";
    std::cout << "  -v, --verbose   Enable verbose output\n";
    std::cout
        << "  -f, --force     Bypass some condition checks (use with caution)\n";
    std::cout << "  -h, --help      Show this help message\n\n";
    std::cout << "\nCommands:\n";
    std::cout
        << "  status          Dump the occActive status for all available OCCs\n";
    std::cout << "  poll [chassis] [occ]\n";
    std::cout << "                  Send POLL command to specified OCC\n";
    std::cout << "  cmd <chassis> <occ> <command_hex_byte> [data_hex_string]\n";
    std::cout << "                  Send Command to specified OCC\n";
    std::cout << "  mode [mode [oemdata]]\n";
    std::cout
        << "                  Dump PowerMode property (optionally set mode first)\n";
    std::cout
        << "                  Mode can be a name or numeric value (decimal or hex):\n";
    std::cout << "                    1 Static, 2 NonDeterministic, 3 SFP*,\n";
    std::cout << "                    5 PowerSaving, 6 EfficiencyFavorPower,\n";
    std::cout
        << "                    7 EfficiencyFavorPerformance, 9 MaxFrequency,\n";
    std::cout
        << "                    10 BalancedPerformance, 11 FFO*, 12 MaximumPerformance\n";
    std::cout
        << "                  * requires oemdata (non-zero frequency point)\n";
    std::cout
        << "  sensors         Dump all D-Bus sensor values from occ-control\n";
    std::cout << "  callhome [chassis]\n";
    std::cout
        << "                  Force OCC Telemetry / Call Home PEL (2A01)\n";
    std::cout
        << "  introspect      Introspect OCC status, power mode, IPS, and power cap D-Bus objects\n";
    std::cout << "  reset [chassis] [occ]\n";
    std::cout
        << "                  Inject OCC error which triggers a PM Complex Reset\n";
    std::cout
        << "  exitsafe        Request HTMGT to attempt to exit Safe Mode via PLDM\n";
    std::cout << "  active          Wait for OCC to go active\n";
    std::cout
        << "  pon             Power on the host and wait for OCCs to go active\n";
    std::cout
        << "                  Displays the BMC console log during the boot\n";
    std::cout << "  con             Same as pon except uses chassison\n";
    std::cout << "  occtrace [chassis] [occ]\n";
    std::cout
        << "                  Retrieve and parse the OCC ERR/IMP/INF traces\n";
    std::cout
        << "  trace [lines]   Dump journal traces from openpower-occ-control\n";
    std::cout << "  restart         Restart the occ-control service\n";
    std::cout << "  stop            Stop the occ-control service\n";
}

#ifndef UNIT_TEST
int main(int argc, char* argv[])
{
    // Separate flags from positional arguments so flags may appear anywhere.
    std::string command;
    std::vector<std::string> posArgs; // positional args after the command

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "-v" || arg == "--verbose" || arg == "-verbose")
        {
            verbose = true;
        }
        else if (arg == "-f" || arg == "--force")
        {
            force = true;
        }
        else if (arg == "-h" || arg == "--help" || arg == "help")
        {
            printUsage(argv[0]);
            return 0;
        }
        else if (command.empty())
        {
            command = arg;
        }
        else
        {
            posArgs.push_back(arg);
        }
    }

    // No command given (possibly only flags) — default to status.
    if (command.empty())
    {
        command = "status";
    }

    if (command == "status")
    {
        return displayOccStatus();
    }
    else if (command == "trace")
    {
        return dumpTrace(posArgs.empty() ? "" : posArgs[0]);
    }
    else if (command == "mode")
    {
        if (!posArgs.empty())
        {
            uint16_t oemData = 0;
            if (posArgs.size() >= 2)
            {
                try
                {
                    oemData = static_cast<uint16_t>(
                        std::stoul(posArgs[1], nullptr, 0));
                }
                catch (const std::exception&)
                {
                    std::cerr << "ERROR: Invalid oemData value: " << posArgs[1]
                              << "\n";
                    return 1;
                }
            }
            int rc = setMode(posArgs[0], oemData);
            if (rc != 0)
                return rc;
        }
        return dumpMode();
    }
    else if (command == "introspect")
    {
        return doIntrospect();
    }
    else if (command == "exitsafe")
    {
        return exitSafe();
    }
    else if (command == "sensors")
    {
        return dumpSensors();
    }
    else if (command == "callhome")
    {
        int chassis = open_power::occ::DEFAULT_CHASSIS;
        if (!posArgs.empty())
        {
            try
            {
                chassis = std::stoi(posArgs[0]);
            }
            catch (const std::exception&)
            {
                std::cerr << "ERROR: Invalid chassis argument\n";
                return 1;
            }
        }
        std::cout << "Forcing OCC Call Home PEL (2A01)\n";
        return sendCmd(chassis, 0, CMD_DEBUG_PT, {0x24});
    }
    else if (command == "poll")
    {
        int chassis = open_power::occ::DEFAULT_CHASSIS;
        int occ = 0;
        try
        {
            if (posArgs.size() >= 1)
                chassis = std::stoi(posArgs[0]);
            if (posArgs.size() >= 2)
                occ = std::stoi(posArgs[1]);
        }
        catch (const std::exception&)
        {
            std::cerr << "ERROR: Invalid chassis/occ argument\n";
            return 1;
        }
        return sendCmd(chassis, occ, CMD_POLL, {0x20});
    }
    else if (command == "active")
    {
        return waitForOccState("ACTIVE");
    }
    else if (command == "restart")
    {
        return restartService();
    }
    else if (command == "reset")
    {
        int chassis = open_power::occ::DEFAULT_CHASSIS;
        int occ = 0;
        try
        {
            if (posArgs.size() >= 1)
                chassis = std::stoi(posArgs[0]);
            if (posArgs.size() >= 2)
                occ = std::stoi(posArgs[1]);
        }
        catch (const std::exception&)
        {
            std::cerr << "ERROR: Invalid chassis/occ argument\n";
            return 1;
        }
        return resetOcc(chassis, occ);
    }
    else if (command == "occtrace")
    {
        int chassis = open_power::occ::DEFAULT_CHASSIS;
        int occ = 0;
        try
        {
            if (posArgs.size() >= 1)
                chassis = std::stoi(posArgs[0]);
            if (posArgs.size() >= 2)
                occ = std::stoi(posArgs[1]);
        }
        catch (const std::exception&)
        {
            std::cerr << "ERROR: Invalid chassis/occ argument\n";
            return 1;
        }
        return occTrace(chassis, occ);
    }
    else if (command == "stop")
    {
        return stopService();
    }
    else if (command == "cmd")
    {
        if (posArgs.size() < 3)
        {
            std::cerr << "Usage: " << argv[0]
                      << " cmd <chassis> <occ> <command_hex> [data_hex]\n";
            return 1;
        }

        int chassis = 0;
        int occ = 0;
        uint8_t cmd = 0;
        std::vector<uint8_t> dataBytes;
        try
        {
            chassis = std::stoi(posArgs[0]);
            occ = std::stoi(posArgs[1]);
            unsigned long cmdVal = std::stoul(posArgs[2], nullptr, 0);
            if (cmdVal > 0xFF)
            {
                std::cerr << "ERROR: Command byte out of range (0x00-0xFF): "
                          << posArgs[2] << "\n";
                return 1;
            }
            cmd = static_cast<uint8_t>(cmdVal);
        }
        catch (const std::exception&)
        {
            std::cerr << "ERROR: Invalid chassis, occ, or command argument\n";
            return 1;
        }

        if (posArgs.size() >= 4)
        {
            std::string hexStr = posArgs[3];
            if (hexStr.rfind("0x", 0) == 0 || hexStr.rfind("0X", 0) == 0)
                hexStr = hexStr.substr(2);
            if (hexStr.length() % 2 != 0)
                hexStr = "0" + hexStr;
            for (size_t i = 0; i < hexStr.length(); i += 2)
            {
                try
                {
                    size_t pos = 0;
                    unsigned long val =
                        std::stoul(hexStr.substr(i, 2), &pos, 16);
                    if (pos != 2)
                    {
                        std::cerr << "ERROR: Invalid hex data: " << posArgs[3]
                                  << "\n";
                        return 1;
                    }
                    dataBytes.push_back(static_cast<uint8_t>(val));
                }
                catch (const std::exception&)
                {
                    std::cerr
                        << "ERROR: Invalid hex data: " << posArgs[3] << "\n";
                    return 1;
                }
            }
        }

        return sendCmd(chassis, occ, cmd, dataBytes);
    }
    else if (command == "pon")
    {
        return powerOn(false);
    }
    else if (command == "con")
    {
        return powerOn(true);
    }
    else
    {
        std::cerr << "Unknown command: " << command << "\n\n";
        printUsage(argv[0]);
        return 1;
    }
}
#endif
