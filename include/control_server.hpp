/**
 * @file control_server.hpp
 * @brief Control server for dls-ctl commands
 */

#ifndef DLS_CONTROL_SERVER_HPP
#define DLS_CONTROL_SERVER_HPP

#include "stats.hpp"
#include "config.hpp"
#include "serial_reader.hpp"
#include "packet_dispatcher.hpp"

#include <thread>
#include <vector>
#include <atomic>
#include <mutex>

namespace dls {

// Forward declaration
class BinaryLogger;

/**
 * @brief TCP control server for dls-ctl
 */
class ControlServer {
public:
    ControlServer(Stats& stats, Config& config,
                  SerialReader& reader, BinaryLogger& logger,
                  PacketDispatcher& dispatcher);
    ~ControlServer();

    // Non-copyable
    ControlServer(const ControlServer&) = delete;
    ControlServer& operator=(const ControlServer&) = delete;

    // ========================================================================
    // Server Control
    // ========================================================================

    /**
     * @brief Start the control server
     * @param port TCP port to listen on
     * @return true on success
     */
    bool start(int port);

    /**
     * @brief Stop the server
     */
    void stop();

    /**
     * @brief Check if server is running
     */
    bool is_running() const;

private:
    void accept_loop();
    void client_handler(int fd);

    nlohmann::json handle_command(const nlohmann::json& cmd);

    nlohmann::json cmd_stats();
    nlohmann::json cmd_status();
    nlohmann::json cmd_start();
    nlohmann::json cmd_stop();
    nlohmann::json cmd_flush();
    nlohmann::json cmd_config_get(const std::string& key);
    nlohmann::json cmd_config_set(const std::string& key, const nlohmann::json& value);
    nlohmann::json cmd_log_enable(const std::string& file);
    nlohmann::json cmd_log_disable();
    nlohmann::json cmd_clients();
    nlohmann::json cmd_gaps(int count);

    void send_json(int fd, const nlohmann::json& j);
    bool recv_command(int fd, nlohmann::json& cmd);

    Stats& stats_;
    Config& config_;
    SerialReader& reader_;
    BinaryLogger& logger_;
    PacketDispatcher& dispatcher_;

    int server_fd_ = -1;
    int port_ = 0;

    std::thread accept_thread_;
    std::mutex threads_mutex_;
    std::vector<std::thread> client_threads_;

    std::atomic<bool> running_{false};
};

} // namespace dls

#endif // DLS_CONTROL_SERVER_HPP
