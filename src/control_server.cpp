/**
 * @file control_server.cpp
 * @brief Control server implementation
 */

#include "control_server.hpp"
#include "binary_logger.hpp"

#include <iostream>
#include <cstring>

#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

namespace dls {

ControlServer::ControlServer(Stats& stats, Config& config,
                             SerialReader& reader, BinaryLogger& logger,
                             PacketDispatcher& dispatcher)
    : stats_(stats)
    , config_(config)
    , reader_(reader)
    , logger_(logger)
    , dispatcher_(dispatcher)
{
}

ControlServer::~ControlServer() {
    stop();
}

// ============================================================================
// Server Control
// ============================================================================

bool ControlServer::start(int port) {
    if (running_.load(std::memory_order_relaxed)) {
        return true;
    }

    port_ = port;

    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        std::cerr << "Error: Cannot create control socket: " << strerror(errno) << "\n";
        return false;
    }

    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(port));

    if (bind(server_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "Error: Cannot bind control socket to port " << port
                  << ": " << strerror(errno) << "\n";
        ::close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    if (listen(server_fd_, 8) < 0) {
        std::cerr << "Error: listen failed: " << strerror(errno) << "\n";
        ::close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    running_.store(true, std::memory_order_relaxed);
    accept_thread_ = std::thread(&ControlServer::accept_loop, this);

    std::cout << "Control server listening on TCP port: " << port << "\n";
    return true;
}

void ControlServer::stop() {
    running_.store(false, std::memory_order_relaxed);

    if (server_fd_ >= 0) {
        // Shutdown to wake up accept()
        shutdown(server_fd_, SHUT_RDWR);
        ::close(server_fd_);
        server_fd_ = -1;
    }

    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }

    std::lock_guard<std::mutex> lock(threads_mutex_);
    for (auto& t : client_threads_) {
        if (t.joinable()) {
            t.join();
        }
    }
    client_threads_.clear();
}

bool ControlServer::is_running() const {
    return running_.load(std::memory_order_relaxed);
}

// ============================================================================
// Accept Loop
// ============================================================================

void ControlServer::accept_loop() {
    while (running_.load(std::memory_order_relaxed)) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);

        int client_fd = accept(server_fd_, reinterpret_cast<struct sockaddr*>(&client_addr),
                               &addr_len);
        if (client_fd < 0) {
            if (errno == EINTR || !running_.load(std::memory_order_relaxed)) {
                break;
            }
            continue;
        }

        stats_.increment_control_clients();

        std::lock_guard<std::mutex> lock(threads_mutex_);
        client_threads_.emplace_back(&ControlServer::client_handler, this, client_fd);
    }
}

// ============================================================================
// Client Handler
// ============================================================================

void ControlServer::client_handler(int fd) {
    // Set TCP_NODELAY
    int opt = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

    // Set read timeout so we can check running_ periodically
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    while (running_.load(std::memory_order_relaxed)) {
        nlohmann::json cmd;
        if (!recv_command(fd, cmd)) {
            break;
        }

        nlohmann::json response = handle_command(cmd);
        send_json(fd, response);
    }

    stats_.decrement_control_clients();
    ::close(fd);
}

nlohmann::json ControlServer::handle_command(const nlohmann::json& cmd) {
    std::string cmd_type = cmd.value("cmd", "");

    if (cmd_type == "stats") {
        return cmd_stats();
    } else if (cmd_type == "status") {
        return cmd_status();
    } else if (cmd_type == "start") {
        return cmd_start();
    } else if (cmd_type == "stop") {
        return cmd_stop();
    } else if (cmd_type == "flush") {
        return cmd_flush();
    } else if (cmd_type == "config") {
        if (cmd.contains("get")) {
            return cmd_config_get(cmd["get"].get<std::string>());
        } else if (cmd.contains("set") && cmd.contains("value")) {
            return cmd_config_set(cmd["set"].get<std::string>(), cmd["value"]);
        }
    } else if (cmd_type == "log") {
        if (cmd.contains("enable") && cmd["enable"].get<bool>()) {
            std::string file = cmd.value("file", "");
            return cmd_log_enable(file);
        } else {
            return cmd_log_disable();
        }
    } else if (cmd_type == "clients") {
        return cmd_clients();
    } else if (cmd_type == "gaps") {
        int count = cmd.value("count", 100);
        return cmd_gaps(count);
    }

    nlohmann::json resp;
    resp["status"] = "error";
    resp["error"] = "Unknown command: " + cmd_type;
    return resp;
}

// ============================================================================
// Command Handlers
// ============================================================================

nlohmann::json ControlServer::cmd_stats() {
    nlohmann::json resp = stats_.to_json(config_.serial_port, config_.baud_rate);
    resp["status"] = "ok";
    return resp;
}

nlohmann::json ControlServer::cmd_status() {
    nlohmann::json resp;
    resp["status"] = "ok";
    resp["daemon"] = DLS_DAEMON_NAME;
    resp["version"] = DLS_DAEMON_VERSION;
    resp["serial_connected"] = reader_.is_open();
    resp["serial_port"] = config_.serial_port;
    resp["baud_rate"] = config_.baud_rate;
    resp["uptime_sec"] = stats_.uptime_seconds();
    resp["packets_total"] = stats_.packets_total();
    resp["data_clients"] = stats_.data_clients();
    resp["logging_enabled"] = stats_.logging_enabled();
    return resp;
}

nlohmann::json ControlServer::cmd_start() {
    nlohmann::json resp;

    if (reader_.is_running()) {
        resp["status"] = "ok";
        resp["message"] = "Already running";
        return resp;
    }

    if (!reader_.is_open()) {
        if (!reader_.open(config_.serial_port, config_.baud_rate)) {
            resp["status"] = "error";
            resp["error"] = "Failed to open serial port";
            return resp;
        }
    }

    reader_.start();
    resp["status"] = "ok";
    return resp;
}

nlohmann::json ControlServer::cmd_stop() {
    nlohmann::json resp;

    reader_.stop();
    resp["status"] = "ok";
    return resp;
}

nlohmann::json ControlServer::cmd_flush() {
    nlohmann::json resp;

    // Clear all client queues
    auto client_ids = dispatcher_.get_client_ids();
    for (auto id : client_ids) {
        dispatcher_.clear_queue(id);
    }

    resp["status"] = "ok";
    resp["clients_flushed"] = client_ids.size();
    return resp;
}

nlohmann::json ControlServer::cmd_config_get(const std::string& key) {
    nlohmann::json resp;
    resp["status"] = "ok";

    if (key == "serial_port") {
        resp["value"] = config_.serial_port;
    } else if (key == "baud_rate") {
        resp["value"] = config_.baud_rate;
    } else if (key == "buffer_size") {
        resp["value"] = config_.buffer_size_kb;
    } else if (key == "data_socket") {
        resp["value"] = config_.data_socket_path;
    } else if (key == "data_tcp_port") {
        resp["value"] = config_.data_tcp_port;
    } else if (key == "control_port") {
        resp["value"] = config_.control_port;
    } else if (key == "verbose") {
        resp["value"] = config_.verbose;
    } else {
        resp["status"] = "error";
        resp["error"] = "Unknown config key: " + key;
    }

    return resp;
}

nlohmann::json ControlServer::cmd_config_set(const std::string& key,
                                              const nlohmann::json& value) {
    nlohmann::json resp;

    if (key == "verbose") {
        config_.verbose = value.get<bool>();
        resp["status"] = "ok";
    } else {
        resp["status"] = "error";
        resp["error"] = "Cannot change config key at runtime: " + key;
    }

    return resp;
}

nlohmann::json ControlServer::cmd_log_enable(const std::string& file) {
    nlohmann::json resp;

    if (file.empty()) {
        resp["status"] = "error";
        resp["error"] = "Log file path required";
        return resp;
    }

    if (logger_.open(file, config_.serial_port, config_.baud_rate)) {
        stats_.set_logging_enabled(true, file);
        resp["status"] = "ok";
        resp["file"] = file;
    } else {
        resp["status"] = "error";
        resp["error"] = "Failed to open log file";
    }

    return resp;
}

nlohmann::json ControlServer::cmd_log_disable() {
    nlohmann::json resp;

    uint64_t bytes = logger_.bytes_written();
    logger_.close();
    stats_.set_logging_enabled(false);

    resp["status"] = "ok";
    resp["bytes_logged"] = bytes;
    return resp;
}

nlohmann::json ControlServer::cmd_clients() {
    nlohmann::json resp;
    resp["status"] = "ok";
    resp["clients"] = nlohmann::json::array();

    auto client_ids = dispatcher_.get_client_ids();
    for (auto id : client_ids) {
        resp["clients"].push_back(dispatcher_.get_client_info(id));
    }

    return resp;
}

nlohmann::json ControlServer::cmd_gaps(int count) {
    nlohmann::json resp;
    resp["status"] = "ok";

    auto gaps = stats_.recent_gaps(static_cast<size_t>(count));
    resp["gaps"] = nlohmann::json::array();

    for (const auto& gap : gaps) {
        nlohmann::json g;
        g["ts"] = gap.timestamp_us;
        g["expected"] = gap.expected;
        g["actual"] = gap.actual;
        g["gap"] = gap.gap_size;
        resp["gaps"].push_back(g);
    }

    resp["total_gaps"] = stats_.mcnt_gaps_total();
    resp["dropped_estimate"] = stats_.dropped_estimate();

    return resp;
}

// ============================================================================
// I/O Helpers
// ============================================================================

void ControlServer::send_json(int fd, const nlohmann::json& j) {
    std::string data = j.dump() + "\n";
    ssize_t total = 0;
    ssize_t len = static_cast<ssize_t>(data.size());

    while (total < len) {
        ssize_t n = write(fd, data.c_str() + total, len - total);
        if (n <= 0) break;
        total += n;
    }
}

bool ControlServer::recv_command(int fd, nlohmann::json& cmd) {
    std::string line;
    char c;

    while (true) {
        ssize_t n = read(fd, &c, 1);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Timeout - check if we should continue
                if (!running_.load(std::memory_order_relaxed)) {
                    return false;
                }
                continue;  // Try again
            }
            return false;  // Real error
        }
        if (n == 0) return false;  // Connection closed
        if (c == '\n') break;
        line += c;
        if (line.size() > 4096) return false;
    }

    if (line.empty()) return true;

    try {
        cmd = nlohmann::json::parse(line);
        return true;
    } catch (...) {
        return false;
    }
}

} // namespace dls
