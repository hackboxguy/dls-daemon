/**
 * @file binary_logger.hpp
 * @brief Binary packet logger for capture and replay
 */

#ifndef DLS_BINARY_LOGGER_HPP
#define DLS_BINARY_LOGGER_HPP

#include "common.hpp"

#include <string>
#include <fstream>
#include <mutex>
#include <atomic>

namespace dls {

/**
 * @brief Binary log file format:
 *
 * File Header (64 bytes):
 *   - Magic: "DLSB" (4 bytes)
 *   - Version: uint32_t (4 bytes)
 *   - Start timestamp: uint64_t microseconds (8 bytes)
 *   - Serial port: char[32] (32 bytes)
 *   - Baud rate: uint32_t (4 bytes)
 *   - Reserved: 12 bytes
 *
 * Packet Records:
 *   - Timestamp: uint64_t microseconds (8 bytes)
 *   - Length: uint32_t (4 bytes)
 *   - Data: raw packet bytes (Length bytes)
 */

constexpr uint8_t BINARY_LOG_MAGIC[4] = {'D', 'L', 'S', 'B'};
constexpr uint32_t BINARY_LOG_VERSION = 1;
constexpr size_t BINARY_LOG_HEADER_SIZE = 64;

struct BinaryLogHeader {
    uint8_t magic[4];
    uint32_t version;
    uint64_t start_timestamp;
    char serial_port[32];
    uint32_t baud_rate;
    uint8_t reserved[12];
};

static_assert(sizeof(BinaryLogHeader) == BINARY_LOG_HEADER_SIZE,
              "BinaryLogHeader size mismatch");

/**
 * @brief Binary packet logger
 */
class BinaryLogger {
public:
    BinaryLogger();
    ~BinaryLogger();

    // Non-copyable
    BinaryLogger(const BinaryLogger&) = delete;
    BinaryLogger& operator=(const BinaryLogger&) = delete;

    // ========================================================================
    // Write Mode
    // ========================================================================

    /**
     * @brief Open log file for writing
     * @param filename Output file path
     * @param port Serial port name (for header)
     * @param baud Baud rate (for header)
     * @return true on success
     */
    bool open(const std::string& filename, const std::string& port, int baud);

    /**
     * @brief Close log file
     */
    void close();

    /**
     * @brief Check if log is open
     */
    bool is_open() const;

    /**
     * @brief Log a raw packet
     * @param data Packet data
     * @param len Packet length
     * @param timestamp_us Capture timestamp in microseconds
     */
    void log_packet(const uint8_t* data, size_t len, uint64_t timestamp_us);

    /**
     * @brief Log a parsed packet
     */
    void log_packet(const DltPacket& packet);

    /**
     * @brief Get bytes written
     */
    uint64_t bytes_written() const;

    /**
     * @brief Get current log filename
     */
    std::string filename() const;

    // ========================================================================
    // Read Mode (for replay)
    // ========================================================================

    /**
     * @brief Open log file for reading (replay)
     * @param filename Input file path
     * @param port Output: serial port from header
     * @param baud Output: baud rate from header
     * @return true on success
     */
    static bool open_for_replay(const std::string& filename,
                                 std::string& port, int& baud);

private:
    bool write_header(const std::string& port, int baud);

    std::ofstream file_;
    std::string filename_;
    std::mutex mutex_;
    std::atomic<uint64_t> bytes_written_{0};
};

} // namespace dls

#endif // DLS_BINARY_LOGGER_HPP
