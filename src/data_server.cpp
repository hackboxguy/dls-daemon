/**
 * @file data_server.cpp
 * @brief Data server implementation
 */

#include "data_server.hpp"

#include <iostream>
#include <cstring>

#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <fcntl.h>

namespace dls {

DataServer::DataServer(PacketDispatcher& dispatcher, const Config& config, Stats& stats)
    : dispatcher_(dispatcher)
    , config_(config)
    , stats_(stats)
    , socket_path_(config.data_socket_path)
    , tcp_port_(config.data_tcp_port)
{
}

DataServer::~DataServer() {
    stop();
}

// ============================================================================
// Server Control
// ============================================================================

bool DataServer::start() {
    if (running_.load(std::memory_order_relaxed)) {
        return true;
    }

    // Create Unix socket (required)
    if (!create_unix_socket()) {
        return false;
    }

    // Create TCP socket (optional)
    if (tcp_port_ > 0) {
        if (!create_tcp_socket()) {
            ::close(unix_fd_);
            unix_fd_ = -1;
            return false;
        }
    }

    running_.store(true, std::memory_order_relaxed);

    // Start accept threads
    unix_accept_thread_ = std::thread(&DataServer::accept_loop_unix, this);

    if (tcp_fd_ >= 0) {
        tcp_accept_thread_ = std::thread(&DataServer::accept_loop_tcp, this);
    }

    return true;
}

void DataServer::stop() {
    running_.store(false, std::memory_order_relaxed);

    // Shutdown and close listening sockets to wake up accept threads
    if (unix_fd_ >= 0) {
        shutdown(unix_fd_, SHUT_RDWR);
        ::close(unix_fd_);
        unix_fd_ = -1;
    }
    if (tcp_fd_ >= 0) {
        shutdown(tcp_fd_, SHUT_RDWR);
        ::close(tcp_fd_);
        tcp_fd_ = -1;
    }

    // Join accept threads
    if (unix_accept_thread_.joinable()) {
        unix_accept_thread_.join();
    }
    if (tcp_accept_thread_.joinable()) {
        tcp_accept_thread_.join();
    }

    // Join client threads
    {
        std::lock_guard<std::mutex> lock(threads_mutex_);
        for (auto& t : client_threads_) {
            if (t.joinable()) {
                t.join();
            }
        }
        client_threads_.clear();
    }

    // Remove socket file
    if (!socket_path_.empty()) {
        unlink(socket_path_.c_str());
    }
}

bool DataServer::is_running() const {
    return running_.load(std::memory_order_relaxed);
}

size_t DataServer::client_count() const {
    return dispatcher_.client_count();
}

// ============================================================================
// Socket Creation
// ============================================================================

bool DataServer::create_unix_socket() {
    // Remove existing socket file
    unlink(socket_path_.c_str());

    unix_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (unix_fd_ < 0) {
        std::cerr << "Error: Cannot create Unix socket: " << strerror(errno) << "\n";
        return false;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

    if (bind(unix_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "Error: Cannot bind Unix socket to " << socket_path_
                  << ": " << strerror(errno) << "\n";
        ::close(unix_fd_);
        unix_fd_ = -1;
        return false;
    }

    if (listen(unix_fd_, 16) < 0) {
        std::cerr << "Error: listen failed: " << strerror(errno) << "\n";
        ::close(unix_fd_);
        unix_fd_ = -1;
        return false;
    }

    std::cout << "Data server listening on Unix socket: " << socket_path_ << "\n";
    return true;
}

bool DataServer::create_tcp_socket() {
    tcp_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_fd_ < 0) {
        std::cerr << "Error: Cannot create TCP socket: " << strerror(errno) << "\n";
        return false;
    }

    // Allow reuse
    int opt = 1;
    setsockopt(tcp_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(tcp_port_));

    if (bind(tcp_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "Error: Cannot bind TCP socket to port " << tcp_port_
                  << ": " << strerror(errno) << "\n";
        ::close(tcp_fd_);
        tcp_fd_ = -1;
        return false;
    }

    if (listen(tcp_fd_, 16) < 0) {
        std::cerr << "Error: listen failed: " << strerror(errno) << "\n";
        ::close(tcp_fd_);
        tcp_fd_ = -1;
        return false;
    }

    std::cout << "Data server listening on TCP port: " << tcp_port_ << "\n";
    return true;
}

// ============================================================================
// Accept Loops
// ============================================================================

void DataServer::accept_loop_unix() {
    while (running_.load(std::memory_order_relaxed)) {
        int client_fd = accept(unix_fd_, nullptr, nullptr);
        if (client_fd < 0) {
            if (errno == EINTR || !running_.load(std::memory_order_relaxed)) {
                break;
            }
            std::cerr << "Warning: Unix accept failed: " << strerror(errno) << "\n";
            continue;
        }

        // Spawn client handler thread
        std::lock_guard<std::mutex> lock(threads_mutex_);
        client_threads_.emplace_back(&DataServer::client_handler, this, client_fd, false);
    }
}

void DataServer::accept_loop_tcp() {
    while (running_.load(std::memory_order_relaxed)) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);

        int client_fd = accept(tcp_fd_, reinterpret_cast<struct sockaddr*>(&client_addr),
                               &addr_len);
        if (client_fd < 0) {
            if (errno == EINTR || !running_.load(std::memory_order_relaxed)) {
                break;
            }
            std::cerr << "Warning: TCP accept failed: " << strerror(errno) << "\n";
            continue;
        }

        if (config_.verbose) {
            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
            std::cout << "TCP client connected from " << client_ip << "\n";
        }

        // Spawn client handler thread
        std::lock_guard<std::mutex> lock(threads_mutex_);
        client_threads_.emplace_back(&DataServer::client_handler, this, client_fd, true);
    }
}

// ============================================================================
// Client Handler
// ============================================================================

void DataServer::client_handler(int fd, bool is_tcp) {
    // Register with dispatcher
    ClientId client_id = dispatcher_.add_client();

    // Set TCP_NODELAY for lower latency
    if (is_tcp) {
        int opt = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));
    }

    // Set read timeout so we can check running_ periodically
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Send welcome message
    nlohmann::json welcome;
    welcome["type"] = "welcome";
    welcome["version"] = DLS_DAEMON_VERSION;
    welcome["client_id"] = client_id;
    send_json(fd, welcome);

    bool subscribed = false;

    while (running_.load(std::memory_order_relaxed)) {
        if (!subscribed) {
            // Wait for subscribe command
            nlohmann::json cmd;
            if (!recv_command(fd, cmd)) {
                break;  // Client disconnected
            }

            std::string cmd_type = cmd.value("cmd", "");

            if (cmd_type == "subscribe") {
                handle_subscribe(fd, client_id, cmd);
                subscribed = true;

                // Start streaming
                stream_packets(fd, client_id);
                break;  // Stream ended (client disconnected or unsubscribed)
            } else if (cmd_type == "clear") {
                dispatcher_.clear_queue(client_id);
                nlohmann::json resp;
                resp["status"] = "ok";
                send_json(fd, resp);
            } else {
                nlohmann::json resp;
                resp["status"] = "error";
                resp["error"] = "Unknown command: " + cmd_type;
                send_json(fd, resp);
            }
        }
    }

    // Cleanup
    dispatcher_.remove_client(client_id);
    ::close(fd);
}

void DataServer::handle_subscribe(int fd, ClientId client_id, const nlohmann::json& cmd) {
    ClientFilter filter;
    filter.all_packets = false;

    // Parse categories filter
    if (cmd.contains("categories") && cmd["categories"].is_array()) {
        for (const auto& cat : cmd["categories"]) {
            if (cat.is_string()) {
                filter.categories.insert(cat.get<std::string>());
            }
        }
    }

    // Parse msg_ids filter
    if (cmd.contains("msg_ids") && cmd["msg_ids"].is_array()) {
        for (const auto& id : cmd["msg_ids"]) {
            if (id.is_number()) {
                filter.msg_ids.insert(id.get<uint32_t>());
            }
        }
    }

    // If no filters specified, subscribe to all
    if (filter.categories.empty() && filter.msg_ids.empty()) {
        filter.all_packets = true;
    }

    dispatcher_.set_filter(client_id, filter);

    // Send confirmation
    nlohmann::json resp;
    resp["status"] = "ok";
    resp["type"] = "subscribed";
    send_json(fd, resp);
}

void DataServer::stream_packets(int fd, ClientId client_id) {
    while (running_.load(std::memory_order_relaxed)) {
        DltPacket packet;
        if (dispatcher_.get_packet(client_id, packet, 100)) {
            send_packet_json(fd, packet);
        }
    }
}

void DataServer::send_packet_json(int fd, const DltPacket& packet) {
    nlohmann::json j;
    j["type"] = "packet";
    j["ts"] = packet.capture_ts_us / 1000000.0;
    j["seq"] = packet.sequence;
    j["mcnt"] = packet.mcnt;
    j["msg_id"] = packet.msg_id;
    j["msg_id_hex"] = packet.msg_id_hex();

    if (!packet.category.empty()) {
        j["category"] = packet.category;
    }
    if (!packet.name.empty()) {
        j["name"] = packet.name;
    }
    if (!packet.apid.empty()) {
        j["apid"] = packet.apid;
    }
    if (!packet.ctid.empty()) {
        j["ctid"] = packet.ctid;
    }
    if (!packet.ecu_id.empty()) {
        j["ecu_id"] = packet.ecu_id;
    }

    j["payload_hex"] = packet.payload_hex();

    std::string ascii = packet.payload_ascii();
    if (!ascii.empty()) {
        j["payload_ascii"] = ascii;
    }

    send_json(fd, j);
}

void DataServer::send_json(int fd, const nlohmann::json& j) {
    std::string data = j.dump() + "\n";
    ssize_t total = 0;
    ssize_t len = static_cast<ssize_t>(data.size());

    while (total < len) {
        ssize_t n = write(fd, data.c_str() + total, len - total);
        if (n <= 0) {
            break;
        }
        total += n;
    }
}

bool DataServer::recv_command(int fd, nlohmann::json& cmd) {
    // Read JSON line (newline-terminated)
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
        if (n == 0) {
            return false;  // Connection closed
        }
        if (c == '\n') {
            break;
        }
        line += c;
        if (line.size() > 4096) {
            return false;  // Too long
        }
    }

    if (line.empty()) {
        return true;  // Empty line, try again
    }

    try {
        cmd = nlohmann::json::parse(line);
        return true;
    } catch (const nlohmann::json::parse_error&) {
        return false;
    }
}

} // namespace dls
