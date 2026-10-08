#include "occctl.hpp"
#include "occtrace_testdata.hpp"

#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace fs = std::filesystem;
using namespace occctl;

// ===========================================================================
// Verify ERR Trace Data from occtrace_testdata.hpp
// ===========================================================================
TEST(OccTraceTest, VerifyErrTraceData)
{
    std::vector<uint8_t> rspBytes;
    bool ok = parseBusctlArrayResponse(occTraceTestData::err, rspBytes);
    ASSERT_TRUE(ok);
    EXPECT_EQ(rspBytes.size(), 185);

    // Verify 5-byte OCC response header
    uint8_t seq = rspBytes[0];
    uint8_t cmd = rspBytes[1];
    uint8_t status = rspBytes[2];
    uint16_t dlen = be16(&rspBytes[3]);

    EXPECT_EQ(seq, 237);  // 0xED
    EXPECT_EQ(cmd, 0x40); // CMD_DEBUG_PT
    EXPECT_EQ(status, 0x00);
    EXPECT_STREQ(getRspStatusStr(status), "SUCCESS");
    EXPECT_EQ(dlen, 180);
    EXPECT_EQ(rspBytes.size(), static_cast<size_t>(5 + dlen));

    // Verify Trace Buffer Header (starts at rspBytes[5])
    const uint8_t* traceHdr = &rspBytes[5];
    uint8_t ver = traceHdr[0];
    uint8_t hdrLen = traceHdr[1];
    EXPECT_EQ(ver, 1);
    EXPECT_EQ(hdrLen, 40);

    // Buffer Tag: "BERR"
    char tag[5] = {};
    memcpy(tag, &traceHdr[3], 4);
    EXPECT_STREQ(tag, "BERR");

    // Buffer Size at offset 20
    uint32_t bufSize = be32(&traceHdr[20]);
    EXPECT_EQ(bufSize, 180);

    // Number of trace entries at offset 32
    uint32_t numEntries = be32(&traceHdr[32]);
    EXPECT_EQ(numEntries, 4);

    // Ensure total bytes for entries match dlen - hdrLen
    EXPECT_EQ(dlen - hdrLen, 140);
}

// ===========================================================================
// Verify IMP Trace Data from occtrace_testdata.hpp
// ===========================================================================
TEST(OccTraceTest, VerifyImpTraceData)
{
    std::vector<uint8_t> rspBytes;
    bool ok = parseBusctlArrayResponse(occTraceTestData::imp, rspBytes);
    ASSERT_TRUE(ok);
    EXPECT_EQ(rspBytes.size(), 2957);

    // Verify 5-byte OCC response header
    uint8_t seq = rspBytes[0];
    uint8_t cmd = rspBytes[1];
    uint8_t status = rspBytes[2];
    uint16_t dlen = be16(&rspBytes[3]);

    EXPECT_EQ(seq, 240);  // 0xF0
    EXPECT_EQ(cmd, 0x40); // CMD_DEBUG_PT
    EXPECT_EQ(status, 0x00);
    EXPECT_STREQ(getRspStatusStr(status), "SUCCESS");
    EXPECT_EQ(dlen, 2952);
    EXPECT_EQ(rspBytes.size(), static_cast<size_t>(5 + dlen));

    // Verify Trace Buffer Header
    const uint8_t* traceHdr = &rspBytes[5];
    uint8_t ver = traceHdr[0];
    uint8_t hdrLen = traceHdr[1];
    EXPECT_EQ(ver, 1);
    EXPECT_EQ(hdrLen, 40);

    // Buffer Tag: "BIMP"
    char tag[5] = {};
    memcpy(tag, &traceHdr[3], 4);
    EXPECT_STREQ(tag, "BIMP");

    // Buffer Size at offset 20
    uint32_t bufSize = be32(&traceHdr[20]);
    EXPECT_EQ(bufSize, 2952);

    // Number of trace entries at offset 32
    uint32_t numEntries = be32(&traceHdr[32]);
    EXPECT_EQ(numEntries, 83);

    EXPECT_EQ(dlen - hdrLen, 2912);
}

// ===========================================================================
// Verify INF Trace Data from occtrace_testdata.hpp
// ===========================================================================
TEST(OccTraceTest, VerifyInfTraceData)
{
    std::vector<uint8_t> rspBytes;
    bool ok = parseBusctlArrayResponse(occTraceTestData::inf, rspBytes);
    ASSERT_TRUE(ok);
    EXPECT_EQ(rspBytes.size(), 4086);

    // Verify 5-byte OCC response header
    uint8_t seq = rspBytes[0];
    uint8_t cmd = rspBytes[1];
    uint8_t status = rspBytes[2];
    uint16_t dlen = be16(&rspBytes[3]);

    EXPECT_EQ(seq, 244);  // 0xF4
    EXPECT_EQ(cmd, 0x40); // CMD_DEBUG_PT
    EXPECT_EQ(status, 0x00);
    EXPECT_STREQ(getRspStatusStr(status), "SUCCESS");
    EXPECT_EQ(dlen, 4081);
    EXPECT_EQ(rspBytes.size(), static_cast<size_t>(5 + dlen));

    // Verify Trace Buffer Header
    const uint8_t* traceHdr = &rspBytes[5];
    uint8_t ver = traceHdr[0];
    uint8_t hdrLen = traceHdr[1];
    EXPECT_EQ(ver, 1);
    EXPECT_EQ(hdrLen, 40);

    // Buffer Tag: "BINF"
    char tag[5] = {};
    memcpy(tag, &traceHdr[3], 4);
    EXPECT_STREQ(tag, "BINF");

    // Buffer Size at offset 20
    uint32_t bufSize = be32(&traceHdr[20]);
    EXPECT_EQ(bufSize, 4081);

    EXPECT_EQ(dlen - hdrLen, 4041);
}

// ===========================================================================
// Verify POLL Response Data from occtrace_testdata.hpp
// ===========================================================================
TEST(OccTraceTest, VerifyPollResponseData)
{
    std::vector<uint8_t> rspBytes;
    bool ok = parseBusctlArrayResponse(occTraceTestData::poll, rspBytes);
    ASSERT_TRUE(ok);
    EXPECT_EQ(rspBytes.size(), 782);

    // Verify 5-byte OCC response header
    uint8_t seq = rspBytes[0];
    uint8_t cmd = rspBytes[1];
    uint8_t status = rspBytes[2];
    uint16_t dlen = be16(&rspBytes[3]);

    EXPECT_EQ(seq, 32);      // 0x20
    EXPECT_EQ(cmd, 0x00);    // CMD_POLL
    EXPECT_EQ(status, 0x00); // SUCCESS
    EXPECT_STREQ(getRspStatusStr(status), "SUCCESS");
    EXPECT_EQ(dlen, 777);
    EXPECT_EQ(rspBytes.size(), static_cast<size_t>(5 + dlen));

    // Decode poll response via parsePollResponse and verify decoded fields
    std::ostringstream out;
    parsePollResponse(rspBytes.data() + 5, dlen, out);
    std::string text = out.str();

    EXPECT_NE(text.find("Master OCCPmcrOwner ObsReady ActReady"),
              std::string::npos);
    EXPECT_NE(text.find("ACTIVE"), std::string::npos);
    EXPECT_NE(text.find("MaxPerformance"), std::string::npos);
    EXPECT_NE(text.find("ENABLED and ACTIVE"), std::string::npos);
    EXPECT_NE(text.find("Sensor: TEMP"), std::string::npos);
    EXPECT_NE(text.find("Sensor: FREQ"), std::string::npos);
    EXPECT_NE(text.find("Sensor: POWR"), std::string::npos);
    EXPECT_NE(text.find("Sensor: CAPS"), std::string::npos);
    EXPECT_NE(text.find("Sensor: EXTN"), std::string::npos);
    EXPECT_NE(text.find("op_p11_260212a"), std::string::npos);
}

// ===========================================================================
// Verify EXTT and EXTN blocks are decoded
// ===========================================================================
TEST(OccTraceTest, VerifyPollExttExtnData)
{
    std::vector<uint8_t> rspBytes;
    ASSERT_TRUE(parseBusctlArrayResponse(occTraceTestData::pollExtt, rspBytes));
    ASSERT_EQ(rspBytes.size(), 392);
    uint16_t dlen = be16(&rspBytes[3]);
    ASSERT_EQ(dlen, 387);

    std::ostringstream out;
    parsePollResponse(rspBytes.data() + 5, dlen, out);
    std::string text = out.str();

    // EXTT: sensor ID, FRU type, temperature only; FRU 0x0a is proc-mma
    EXPECT_NE(text.find("Sensor: EXTT - format:0, 16 sensors (6 bytes/sensor)"),
              std::string::npos);
    EXPECT_NE(text.find("cc000000 00 2b ( 43C core)"), std::string::npos);
    EXPECT_NE(text.find("cc000000 0a 4d ( 77C proc-mma)"), std::string::npos);

    // EXTN: named entries decoded
    EXPECT_NE(text.find("Sensor: EXTN - format:0, 8 sensors (12 bytes/sensor)"),
              std::string::npos);
    EXPECT_NE(
        text.find("\"FMIN\" 00 00 8c07d0000000    pState: 140 / 2000 MHz"),
        std::string::npos);
    EXPECT_NE(text.find("OCC clip pState: 188, count:255, reason:PROC_THRM"),
              std::string::npos);
    EXPECT_NE(text.find("\"WOFC\" 00 00 ff0000010200    WOF Disabled: "
                        "THERMAL_WOF UTURBO_IS_ZERO"),
              std::string::npos);
    EXPECT_NE(text.find("\"CEF0\" 00 00 ff0000000000"), std::string::npos);
}

// ===========================================================================
// Busctl Array Parser Error Handling
// ===========================================================================
TEST(OccTraceTest, ParseBusctlArrayResponse_InvalidInputs)
{
    std::vector<uint8_t> bytes;

    // Empty input
    EXPECT_FALSE(parseBusctlArrayResponse("", bytes));

    // Missing 'ai' prefix
    EXPECT_FALSE(parseBusctlArrayResponse("s 3 1 2 3", bytes));
    EXPECT_FALSE(parseBusctlArrayResponse("10 1 2 3", bytes));

    // Negative count
    EXPECT_FALSE(parseBusctlArrayResponse("ai -1 1 2 3", bytes));

    // Count exceeding maximum limit (65536)
    EXPECT_FALSE(parseBusctlArrayResponse("ai 70000 1 2 3", bytes));

    // Truncated data
    EXPECT_FALSE(parseBusctlArrayResponse("ai 5 1 2 3", bytes));

    // Non-numeric elements
    EXPECT_FALSE(parseBusctlArrayResponse("ai 3 1 invalid 3", bytes));

    // Valid empty array
    EXPECT_TRUE(parseBusctlArrayResponse("ai 0", bytes));
    EXPECT_TRUE(bytes.empty());
}

// ===========================================================================
// Trace File Writing and Stripping Simulation
// ===========================================================================
TEST(OccTraceTest, TraceFileWritingAndStripping)
{
    struct TestCase
    {
        const char* name;
        const char* rawData;
        const char* expectedTag;
        size_t expectedFileSize;
    };

    const TestCase testCases[] = {
        {"ERR", occTraceTestData::err, "BERR", 185},
        {"IMP", occTraceTestData::imp, "BIMP", 2957},
        {"INF", occTraceTestData::inf, "BINF", 4086},
    };

    char tmpTemplate[] = "/tmp/occtrace_test_XXXXXX";
    char* tmpDir = mkdtemp(tmpTemplate);
    ASSERT_NE(tmpDir, nullptr);

    for (const auto& tc : testCases)
    {
        std::vector<uint8_t> rspBytes;
        ASSERT_TRUE(parseBusctlArrayResponse(tc.rawData, rspBytes));
        EXPECT_EQ(rspBytes.size(), tc.expectedFileSize);

        fs::path filePath = fs::path(tmpDir) / (std::string(tc.name) + ".bin");
        std::ofstream out(filePath, std::ios::binary | std::ios::trunc);
        ASSERT_TRUE(out.is_open());
        out.write(reinterpret_cast<const char*>(rspBytes.data()),
                  static_cast<std::streamsize>(rspBytes.size()));
        out.close();

        // Read back and check size
        EXPECT_EQ(fs::file_size(filePath), tc.expectedFileSize);

        // Verify the 5-byte header stripping
        std::ifstream in(filePath, std::ios::binary);
        ASSERT_TRUE(in.is_open());
        std::vector<uint8_t> fileBytes((std::istreambuf_iterator<char>(in)),
                                       std::istreambuf_iterator<char>());
        in.close();

        ASSERT_EQ(fileBytes.size(), tc.expectedFileSize);
        ASSERT_GE(fileBytes.size(), 5 + 40);

        // Memory view offset 5 has the trace buffer header
        const uint8_t* traceBody = fileBytes.data() + 5;
        EXPECT_EQ(traceBody[0], 1);  // version
        EXPECT_EQ(traceBody[1], 40); // header len

        char tag[5] = {};
        memcpy(tag, &traceBody[3], 4);
        EXPECT_STREQ(tag, tc.expectedTag);
    }

    fs::remove_all(tmpDir);
}

// ===========================================================================
// Trace Helpers and Type Definitions
// ===========================================================================
TEST(OccTraceTest, TraceTypeListValidation)
{
    EXPECT_STREQ(traceList[0].name, "ERR");
    EXPECT_EQ(traceList[0].id[0], 'E');
    EXPECT_EQ(traceList[0].id[1], 'R');
    EXPECT_EQ(traceList[0].id[2], 'R');
    EXPECT_EQ(traceList[0].skipLines, 5);

    EXPECT_STREQ(traceList[1].name, "IMP");
    EXPECT_EQ(traceList[1].id[0], 'I');
    EXPECT_EQ(traceList[1].id[1], 'M');
    EXPECT_EQ(traceList[1].id[2], 'P');
    EXPECT_EQ(traceList[1].skipLines, 7);

    EXPECT_STREQ(traceList[2].name, "INF");
    EXPECT_EQ(traceList[2].id[0], 'I');
    EXPECT_EQ(traceList[2].id[1], 'N');
    EXPECT_EQ(traceList[2].id[2], 'F');
    EXPECT_EQ(traceList[2].skipLines, 10);
}

// ===========================================================================
// Debug Pass-Through Sensor Parsing
// ===========================================================================
TEST(OccTraceTest, ParseDebugPassthruSensorData)
{
    const char* sensorRaw =
        "ai 391 105 64 0 1 130 0 16 77 69 77 85 84 73 76 77 48 48 0 0 0 0 0 0 1 "
        "208 0 2 0 2 0 3 77 69 77 85 84 73 76 77 48 49 0 0 0 0 0 0 1 209 0 2 0 2 "
        "0 2 77 69 77 85 84 73 76 77 48 50 0 0 0 0 0 0 1 210 0 1 0 1 0 2 77 69 77 "
        "85 84 73 76 77 48 51 0 0 0 0 0 0 1 211 0 1 0 1 0 2 77 69 77 85 84 73 76 "
        "77 48 52 0 0 0 0 0 0 1 212 0 2 0 2 0 3 77 69 77 85 84 73 76 77 48 53 0 0 "
        "0 0 0 0 1 213 0 1 0 1 0 2 77 69 77 85 84 73 76 77 48 54 0 0 0 0 0 0 1 "
        "214 0 1 0 1 0 2 77 69 77 85 84 73 76 77 48 55 0 0 0 0 0 0 1 215 0 0 0 0 "
        "0 2 77 69 77 85 84 73 76 77 48 56 0 0 0 0 0 0 1 216 0 0 0 0 0 0 77 69 77 "
        "85 84 73 76 77 48 57 0 0 0 0 0 0 1 217 0 0 0 0 0 0 77 69 77 85 84 73 76 "
        "77 49 48 0 0 0 0 0 0 1 218 0 0 0 0 0 0 77 69 77 85 84 73 76 77 49 49 0 0 "
        "0 0 0 0 1 219 0 0 0 0 0 0 77 69 77 85 84 73 76 77 49 50 0 0 0 0 0 0 1 "
        "220 0 0 0 0 0 0 77 69 77 85 84 73 76 77 49 51 0 0 0 0 0 0 1 221 0 0 0 0 "
        "0 0 77 69 77 85 84 73 76 77 49 52 0 0 0 0 0 0 1 222 0 0 0 0 0 0 77 69 77 "
        "85 84 73 76 77 49 53 0 0 0 0 0 0 1 223 0 0 0 0 0 0";

    std::vector<uint8_t> rspBytes;
    ASSERT_TRUE(parseBusctlArrayResponse(sensorRaw, rspBytes));
    EXPECT_EQ(rspBytes.size(), 391);

    // Header validation
    EXPECT_EQ(rspBytes[0], 105);
    EXPECT_EQ(rspBytes[1], 0x40); // CMD_DEBUG_PT
    EXPECT_EQ(rspBytes[2], 0x00); // SUCCESS
    EXPECT_EQ(be16(&rspBytes[3]), 386);

    // Parse body through debug passthru parser
    const uint8_t cmdData[] = {0x07}; // Get Multiple Sensor Data
    std::ostringstream out;
    parseDebugPassthruResponse(cmdData, sizeof(cmdData), rspBytes.data() + 5,
                               static_cast<uint16_t>(rspBytes.size() - 5), out);
    std::string text = out.str();
    EXPECT_NE(text.find("Number of sensors retrieved: 16"), std::string::npos);
    EXPECT_NE(text.find("MEMUTILM00"), std::string::npos);
    EXPECT_NE(text.find("0x01d0"), std::string::npos);
}

// ===========================================================================
// Response Status and Mode Strings
// ===========================================================================
TEST(OccTraceTest, ResponseStatusStrings)
{
    EXPECT_STREQ(getRspStatusStr(0x00), "SUCCESS");
    EXPECT_STREQ(getRspStatusStr(0x01), "CONDITIONAL_SUCCESS");
    EXPECT_STREQ(getRspStatusStr(0x11), "INVALID_COMMAND");
    EXPECT_STREQ(getRspStatusStr(0x12), "INVALID_CMD_LENGTH");
    EXPECT_STREQ(getRspStatusStr(0x13), "INVALID_DATA");
    EXPECT_STREQ(getRspStatusStr(0x14), "CHECKSUM_FAIL");
    EXPECT_STREQ(getRspStatusStr(0x15), "INTERNAL_FAIL");
    EXPECT_STREQ(getRspStatusStr(0x16), "INVALID_STATE");
    EXPECT_STREQ(getRspStatusStr(0xE0), "EXCEPTION-PANIC");
    EXPECT_STREQ(getRspStatusStr(0xFF), "CMD_IN_PROGRESS");
    EXPECT_STREQ(getRspStatusStr(0x99), "UNKNOWN");
}

TEST(OccTraceTest, StateAndModeStrings)
{
    EXPECT_STREQ(getOccStateStr(0x01), "STANDBY");
    EXPECT_STREQ(getOccStateStr(0x02), "OBSERVATION");
    EXPECT_STREQ(getOccStateStr(0x03), "ACTIVE");
    EXPECT_STREQ(getOccStateStr(0x04), "SAFE");
    EXPECT_STREQ(getOccStateStr(0xFF), "UNKNOWN");

    EXPECT_STREQ(getOccModeStr(0x01), "OEM: Static");
    EXPECT_STREQ(getOccModeStr(0x04), "SAFE");
    EXPECT_STREQ(getOccModeStr(0x05), "PowerSaving");
    EXPECT_STREQ(getOccModeStr(0x0C), "MaxPerformance");
    EXPECT_STREQ(getOccModeStr(0xFF), "UNKNOWN");
}
