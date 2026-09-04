#include "BambuMcpBridge.hpp"

#include <wx/string.h>
#include <wx/utils.h>

#include <cstdlib>

#ifdef __APPLE__
#include <cerrno>
#include <chrono>
#include <csignal>
#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

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

long dispatch_bambu_mcp_with_progress(
    const std::string &tool,
    const std::vector<std::pair<std::string, std::string>> &arguments,
    const std::function<void(int, int, std::string)> &progress,
    const std::function<bool()> &was_canceled)
{
#ifdef __APPLE__
    const char *node_env = std::getenv("BAMBU_MCP_NODE");
    const char *bridge_env = std::getenv("BAMBU_MCP_BRIDGE");
    std::vector<std::string> command {
        node_env ? node_env : "/opt/homebrew/bin/node",
        bridge_env ? bridge_env : "/Volumes/Apple/Projects/3D/bambu-mcp/scripts/call-bambu-mcp.mjs",
        tool,
    };
    command.reserve(command.size() + arguments.size());
    for (const auto &argument : arguments)
        command.emplace_back(argument.first + "=" + argument.second);

    std::vector<char *> argv;
    argv.reserve(command.size() + 1);
    for (std::string &value : command)
        argv.push_back(value.data());
    argv.push_back(nullptr);

    int output_pipe[2] {-1, -1};
    if (::pipe(output_pipe) != 0)
        return -1;

    const pid_t pid = ::fork();
    if (pid == 0) {
        ::setpgid(0, 0);
        ::dup2(output_pipe[1], STDOUT_FILENO);
        ::dup2(output_pipe[1], STDERR_FILENO);
        ::close(output_pipe[0]);
        ::close(output_pipe[1]);
        ::execv(argv[0], argv.data());
        ::_exit(127);
    }
    if (pid < 0) {
        ::close(output_pipe[0]);
        ::close(output_pipe[1]);
        return -1;
    }

    // Close the race between fork() and the child's setpgid(). EACCES means
    // the child has already exec'd after successfully creating its group.
    const bool has_process_group = ::setpgid(pid, pid) == 0 || errno == EACCES;
    ::close(output_pipe[1]);
    const int current_flags = ::fcntl(output_pipe[0], F_GETFL, 0);
    if (current_flags >= 0)
        ::fcntl(output_pipe[0], F_SETFL, current_flags | O_NONBLOCK);

    std::string pending;
    const std::string marker = "QINGXIAO_MCP_PROGRESS\t";
    const auto process_line = [&progress, &marker](const std::string &line) {
        if (!progress || line.compare(0, marker.size(), marker) != 0)
            return;
        const size_t stage_end = line.find('\t', marker.size());
        const size_t code_end = stage_end == std::string::npos ? std::string::npos : line.find('\t', stage_end + 1);
        if (stage_end == std::string::npos || code_end == std::string::npos)
            return;
        try {
            const int stage = std::stoi(line.substr(marker.size(), stage_end - marker.size()));
            const int code = std::stoi(line.substr(stage_end + 1, code_end - stage_end - 1));
            progress(stage, code, line.substr(code_end + 1));
        } catch (...) {
            // Ignore malformed helper output; process exit status remains the
            // authoritative command result.
        }
    };
    const auto consume_lines = [&pending, &process_line]() {
        size_t newline = std::string::npos;
        while ((newline = pending.find('\n')) != std::string::npos) {
            std::string line = pending.substr(0, newline);
            pending.erase(0, newline + 1);
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            process_line(line);
        }
    };

    bool child_exited = false;
    bool pipe_closed = false;
    bool canceled = false;
    bool sent_sigkill = false;
    int child_status = 0;
    auto cancel_deadline = std::chrono::steady_clock::time_point::max();

    while (!child_exited || !pipe_closed) {
        if (!canceled && was_canceled && was_canceled()) {
            canceled = true;
            cancel_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            ::kill(has_process_group ? -pid : pid, SIGTERM);
        } else if (canceled && !sent_sigkill && std::chrono::steady_clock::now() >= cancel_deadline) {
            sent_sigkill = true;
            ::kill(has_process_group ? -pid : pid, SIGKILL);
        }

        pollfd descriptor {output_pipe[0], static_cast<short>(POLLIN | POLLHUP), 0};
        const int poll_result = ::poll(&descriptor, 1, 50);
        if (poll_result > 0 && (descriptor.revents & (POLLIN | POLLHUP | POLLERR))) {
            char buffer[4096];
            while (true) {
                const ssize_t count = ::read(output_pipe[0], buffer, sizeof(buffer));
                if (count > 0) {
                    pending.append(buffer, static_cast<size_t>(count));
                    consume_lines();
                } else if (count == 0) {
                    pipe_closed = true;
                    break;
                } else if (errno == EINTR) {
                    continue;
                } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    break;
                } else {
                    pipe_closed = true;
                    break;
                }
            }
        }

        if (!child_exited) {
            const pid_t wait_result = ::waitpid(pid, &child_status, WNOHANG);
            if (wait_result == pid)
                child_exited = true;
            else if (wait_result < 0 && errno != EINTR)
                child_exited = true;
        }
    }

    ::close(output_pipe[0]);
    if (!pending.empty())
        process_line(pending);
    if (!child_exited)
        ::waitpid(pid, &child_status, 0);

    if (canceled)
        return BAMBU_MCP_DISPATCH_CANCELED;
    return WIFEXITED(child_status) && WEXITSTATUS(child_status) == 0 ? 1 : -1;
#else
    (void)tool;
    (void)arguments;
    (void)progress;
    (void)was_canceled;
    return -1;
#endif
}

} // namespace Slic3r
