/**
 * @file data_server.hpp
 * @brief Data server for packet streaming to clients
 *
 * Provides Unix domain socket (always) and optional TCP socket for
 * streaming DLT packets to connected clients.
 */

#ifndef DLS_DATA_SERVER_HPP
#define DLS_DATA_SERVER_HPP

#include "packet_dispatcher.hpp"
#include "config.hpp"
#include "stats.hpp"

#include <string>
#include <thread>
#include <vector>
#include <atomic>
#include <mutex>

namespace dls {

/**
 * @brief Data server for Unix socket and TCP packet streaming
 */
class DataServer {
public:
    DataServer(PacketDispatcher& dispatcher, const Config& config, Stats& stats);
    ~DataServer();

    // Non-copyable
    DataServer(const DataServer&) = delete;
    DataServer& operator=(const DataServer&) = delete;

    // ========================================================================
    // Server Control
    // ========================================================================

    /**
     * @brief Start the server
     * @return true on success
     */
    bool start();

    /**
     * @brief Stop the server and disconnect all clients
     */
    void stop();

    /**
     * @brief Check if server is running
     */
    bool is_running() const;

    // ========================================================================
    // Information
    // ========================================================================

    /**
     * @brief Get number of connected clients
     */
    size_t client_count() const;

private:
    void accept_loop_unix();
    void accept_loop_tcp();
    void client_handler(int fd, bool is_tcp);

    void handle_subscribe(int fd, ClientId client_id, const nlohmann::json& cmd);
    void stream_packets(int fd, ClientId client_id);
    void send_packet_json(int fd, const DltPacket& packet);
    void send_json(int fd, const nlohmann::json& j);
    bool recv_command(int fd, nlohmann::json& cmd);

    bool create_unix_socket();
    bool create_tcp_socket();

    PacketDispatcher& dispatcher_;
    const Config& config_;
    Stats& stats_;

    // Unix socket
    int unix_fd_ = -1;
    std::string socket_path_;
    std::thread unix_accept_thread_;

    // TCP socket (optional)
    int tcp_fd_ = -1;
    int tcp_port_ = 0;
    std::thread tcp_accept_thread_;

    // Client threads
    std::mutex threads_mutex_;
    std::vector<std::thread> client_threads_;

    // Control
    std::atomic<bool> running_{false};
};

} // namespace dls

#endif // DLS_DATA_SERVER_HPP
