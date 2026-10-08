#pragma once

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace occctl
{

// Parse a busctl "ai <count> <v0> <v1> ..." response into a byte vector.
// Returns true on success.
bool parseBusctlArrayResponse(const std::string& busctlOutput,
                              std::vector<uint8_t>& bytes);

uint16_t be16(const uint8_t* p);
uint32_t be32(const uint8_t* p);

const char* getPollStatusStr(uint8_t flags);
const char* getPollExtStatusStr(uint8_t flags);
const char* getOccStateStr(uint8_t state);
const char* getOccModeStr(uint8_t mode);
const char* getRspStatusStr(uint8_t status);
const char* getFruStr(uint8_t fru);

void parsePollResponse(const uint8_t* rsp, uint16_t rspLen,
                       std::ostream& out = std::cout);

void parseDebugPassthruResponse(const uint8_t* cmdData, uint16_t cmdLen,
                                const uint8_t* rsp, uint16_t rspLen,
                                std::ostream& out = std::cout);

void parseMfgTestResponse(const uint8_t* cmdData, uint16_t cmdLen,
                          const uint8_t* rsp, uint16_t rspLen,
                          std::ostream& out = std::cout);

struct TraceType
{
    const char* name;
    const char* file;
    char id[3];
    int skipLines; // initial trace lines to drop from parsed output
};

extern const TraceType traceList[3];

int occTrace(int chassis, int occ);

} // namespace occctl
