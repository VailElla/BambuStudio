#pragma once

#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace Slic3r {

// Dispatch a device-control tool through the local bambu-mcp process. The
// launcher reads the printer access code from Keychain; it is never passed in
// argv or persisted by BambuStudio. Returns 1 after a successful MCP command,
// or -1 when the helper or the command fails.
long dispatch_bambu_mcp(const std::string &tool, const std::vector<std::pair<std::string, std::string>> &arguments);

// Stream native Bambu printing stages while the MCP command is running. The
// callback is invoked on the calling job thread. Cancellation terminates the
// complete local MCP process group, including the native helper.
constexpr long BAMBU_MCP_DISPATCH_CANCELED = -2;
long dispatch_bambu_mcp_with_progress(
    const std::string &tool,
    const std::vector<std::pair<std::string, std::string>> &arguments,
    const std::function<void(int, int, std::string)> &progress,
    const std::function<bool()> &was_canceled);

bool is_x2d_printer(const std::string &printer_type);

} // namespace Slic3r
