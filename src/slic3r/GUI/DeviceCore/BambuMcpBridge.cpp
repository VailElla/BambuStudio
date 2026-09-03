#include "BambuMcpBridge.hpp"

#include <wx/string.h>
#include <wx/utils.h>

#include <cstdlib>

namespace Slic3r {

bool is_x2d_printer(const std::string &printer_type)
{
    return printer_type == "N6" || printer_type == "N6-V2" || printer_type == "X2D" || printer_type == "x2d";
}

long dispatch_bambu_mcp(const std::string &tool, const std::vector<std::pair<std::string, std::string>> &arguments)
{
#ifdef __APPLE__
    wxString node = wxString::FromUTF8(std::getenv("BAMBU_MCP_NODE") ? std::getenv("BAMBU_MCP_NODE") :
        "/opt/homebrew/bin/node");
    wxString bridge = wxString::FromUTF8(std::getenv("BAMBU_MCP_BRIDGE") ? std::getenv("BAMBU_MCP_BRIDGE") :
        "/Volumes/Apple/Projects/3D/bambu-mcp/scripts/call-bambu-mcp.mjs");
    auto shellQuote = [](const wxString &value) {
        wxString escaped = value;
        escaped.Replace("'", "'\\''");
        return "'" + escaped + "'";
    };
    wxString command = shellQuote(node) + " " + shellQuote(bridge) + " " + shellQuote(wxString::FromUTF8(tool));
    for (const auto &argument : arguments) {
        command += " " + shellQuote(wxString::FromUTF8(argument.first + "=" + argument.second));
    }
    // The device command must not be reported as successful merely because the
    // helper process started. Wait for MCP/AppleScript to finish and translate
    // its exit status to the legacy positive-success convention used by the
    // callers in DevFan and DeviceManager.
    const int exit_code = wxExecute(command, wxEXEC_SYNC | wxEXEC_HIDE_CONSOLE);
    return exit_code == 0 ? 1 : -1;
#else
    (void)tool;
    (void)arguments;
    return -1;
#endif
}

} // namespace Slic3r
