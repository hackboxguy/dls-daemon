/**
 * @file serial_reader.hpp
 * @brief High-performance serial port reader thread
 *
 * Dedicated thread for reading serial data with:
 * - Real-time SCHED_FIFO priority
 * - epoll-based non-blocking I/O
 * - Large kernel buffers
 * - Auto-reconnect on disconnect
 */

#ifndef DLS_SERIAL_READER_HPP
#define DLS_SERIAL_READER_HPP

#include "ring_buffer.hpp"
#include "stats.hpp"

#include <string>
#include <thread>
#include <atomic>
#include <functional>

namespace dls {

/**
 * @brief Serial port reader with dedicated high-priority thread
 */
class SerialReader {
public:
    /**
     * @brief Construct serial reader
     * @param buffer Ring buffer to write data to
     * @param stats Statistics collector
     */
    SerialReader(RingBuffer& buffer, Stats& stats);

    ~SerialReader();

    // Non-copyable
    SerialReader(const SerialReader&) = delete;
    SerialReader& operator=(const SerialReader&) = delete;

    // ========================================================================
    // Serial Port Operations
    // ========================================================================

    /**
     * @brief Open serial port
     * @param port Serial port path (e.g., /dev/ttyUSB2)
     * @param baud_rate Baud rate (e.g., 115200)
     * @return true on success
     */
    bool open(const std::string& port, int baud_rate);

    /**
     * @brief Close serial port
     */
    void close();

    /**
     * @brief Check if serial port is open
     */
    bool is_open() const;

    // ========================================================================
    // Replay Mode
    // ========================================================================

    /**
     * @brief Open binary log file for replay
     * @param path Path to binary log file
     * @param speed_factor Replay speed (1.0 = real-time, 0 = fast as possible)
     * @return true on success
     */
    bool open_replay(const std::string& path, double speed_factor = 1.0);

    /**
     * @brief Check if in replay mode
     */
    bool is_replay_mode() const;

    // ========================================================================
    // Thread Control
    // ========================================================================

    /**
     * @brief Start reader thread
     */
    void start();

    /**
     * @brief Stop reader thread
     */
    void stop();

    /**
     * @brief Check if reader is running
     */
    bool is_running() const;

    // ========================================================================
    // State Queries
    // ========================================================================

    std::string port() const { return port_; }
    int baud_rate() const { return baud_rate_; }

    /**
     * @brief Set callback for connection state changes
     */
    void set_connection_callback(std::function<void(bool connected)> callback);

private:
    void reader_loop();
    void replay_loop();

    bool configure_serial();
    bool set_realtime_priority();
    void handle_disconnect();
    bool try_reconnect();

    int speed_to_baud(int baud) const;

    // Ring buffer to write to
    RingBuffer& buffer_;
    Stats& stats_;

    // Serial port state
    std::string port_;
    int baud_rate_ = 0;
    int serial_fd_ = -1;
    int epoll_fd_ = -1;

    // Replay state
    std::string replay_path_;
    double replay_speed_ = 1.0;
    int replay_fd_ = -1;
    bool replay_mode_ = false;

    // Thread control
    std::thread reader_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};

    // Reconnect state
    int reconnect_attempts_ = 0;
    static constexpr int MAX_RECONNECT_DELAY_MS = 5000;

    // Callback
    std::function<void(bool)> connection_callback_;

    // Local read buffer
    static constexpr size_t READ_BUFFER_SIZE = 4096;
};

} // namespace dls

#endif // DLS_SERIAL_READER_HPP
