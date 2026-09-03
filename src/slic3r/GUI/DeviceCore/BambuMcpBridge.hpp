#pragma once

#include <string>
#include <utility>
#include <vector>

namespace Slic3r {

// Dispatch a device-control tool through the local bambu-mcp process. The
// launcher reads the printer access code from Keychain; it is never passed in
// argv or persisted by BambuStudio. Returns 1 after a successful MCP command,
// or -1 when the helper or the command fails.
long dispatch_bambu_mcp(const std::string &tool, const std::vector<std::pair<std::string, std::string>> &arguments);

bool is_x2d_printer(const std::string &printer_type);

} // namespace Slic3r
