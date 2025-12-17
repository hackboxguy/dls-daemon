/**
 * @file common.hpp
 * @brief Common types, constants, and utilities for DLS daemon
 */

#ifndef DLS_COMMON_HPP
#define DLS_COMMON_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <chrono>

namespace dls {

// ============================================================================
// Version Information
// ============================================================================

constexpr const char* DLS_DAEMON_VERSION = "1.0.0";
constexpr const char* DLS_DAEMON_NAME = "dls-daemon";

// ============================================================================
// DLT Protocol Constants (from GENIVI/COVESA DLT specification)
// ============================================================================

// Serial header magic
constexpr uint8_t DLS_MAGIC[4] = {'D', 'L', 'S', 0x01};
constexpr size_t DLS_MAGIC_SIZE = 4;

// Header sizes
constexpr size_t DLT_SERIAL_HEADER_SIZE = 4;   // "DLS\x01"
constexpr size_t DLT_STD_HEADER_SIZE = 4;      // htyp + mcnt + len(2)
constexpr size_t DLT_ID_SIZE = 4;              // ECU/APP/CTX ID size
constexpr size_t DLT_EXT_HEADER_SIZE = 10;     // msin + noar + apid(4) + ctid(4)

// Minimum packet size: serial header + standard header
constexpr size_t DLT_MIN_PACKET_SIZE = DLT_SERIAL_HEADER_SIZE + DLT_STD_HEADER_SIZE;

// Maximum reasonable packet size (64KB)
constexpr size_t DLT_MAX_PACKET_SIZE = 65535;

// Header Type (htyp) bit flags - byte 4 after serial header
constexpr uint8_t DLT_HTYP_UEH  = 0x01;  // Use Extended Header (APID/CTID present)
constexpr uint8_t DLT_HTYP_MSBF = 0x02;  // MSB First (big-endian payload)
constexpr uint8_t DLT_HTYP_WEID = 0x04;  // With ECU ID (4 bytes)
constexpr uint8_t DLT_HTYP_WSID = 0x08;  // With Session ID (4 bytes)
constexpr uint8_t DLT_HTYP_WTMS = 0x10;  // With Timestamp (4 bytes)
constexpr uint8_t DLT_HTYP_VERS = 0xE0;  // Version number mask (bits 5-7)

// Message type (mstp) - from extended header msin field (bits 1-3)
constexpr uint8_t DLT_TYPE_LOG       = 0x00;  // Log message
constexpr uint8_t DLT_TYPE_APP_TRACE = 0x01;  // Application trace
constexpr uint8_t DLT_TYPE_NW_TRACE  = 0x02;  // Network trace
constexpr uint8_t DLT_TYPE_CONTROL   = 0x03;  // Control message

// Log level (mtin for DLT_TYPE_LOG) - bits 4-7 of msin
constexpr uint8_t DLT_LOG_OFF     = 0x00;
constexpr uint8_t DLT_LOG_FATAL   = 0x01;
constexpr uint8_t DLT_LOG_ERROR   = 0x02;
constexpr uint8_t DLT_LOG_WARN    = 0x03;
constexpr uint8_t DLT_LOG_INFO    = 0x04;
constexpr uint8_t DLT_LOG_DEBUG   = 0x05;
constexpr uint8_t DLT_LOG_VERBOSE = 0x06;

// ============================================================================
// Packet Structure
// ============================================================================

/**
 * @brief Parsed DLT packet
 */
struct DltPacket {
    // Capture information
    uint64_t capture_ts_us = 0;     // Capture timestamp (microseconds since epoch)
    uint64_t sequence = 0;          // Daemon-assigned sequence number

    // Standard header fields
    uint8_t htyp = 0;               // Header type flags
    uint8_t mcnt = 0;               // Message counter (0-255, wraps)
    uint16_t msg_len = 0;           // Message length (from standard header)

    // Flags parsed from htyp
    bool has_ecu_id = false;
    bool has_session_id = false;
    bool has_timestamp = false;
    bool has_ext_header = false;
    bool is_big_endian = false;

    // Optional standard header fields
    std::string ecu_id;             // 4-char ECU ID (if WEID)
    uint32_t session_id = 0;        // Session ID (if WSID)
    uint32_t timestamp = 0;         // Timestamp (if WTMS)

    // Extended header fields (if UEH flag set)
    std::string apid;               // 4-char Application ID
    std::string ctid;               // 4-char Context ID
    uint8_t msg_type = 0;           // Message type (log, trace, control)
    uint8_t msg_subtype = 0;        // Message subtype / log level
    uint8_t noar = 0;               // Number of arguments
    bool is_verbose = false;        // Verbose mode flag

    // Payload
    uint32_t msg_id = 0;            // Message ID (first 4 bytes of payload for non-verbose)
    std::vector<uint8_t> payload;   // Payload data (after msg_id for non-verbose)
    std::vector<uint8_t> raw_data;  // Complete raw packet data

    // FIBEX enrichment (filled by dispatcher if config loaded)
    std::string category;           // Category from FIBEX (e.g., "VERSION", "BOOT")
    std::string name;               // Message name from FIBEX

    // Total packet size (including serial header)
    size_t total_size = 0;

    /**
     * @brief Get message ID in hex format
     */
    std::string msg_id_hex() const {
        char buf[16];
        snprintf(buf, sizeof(buf), "0x%08x", msg_id);
        return std::string(buf);
    }

    /**
     * @brief Get payload as hex string
     */
    std::string payload_hex() const {
        std::string result;
        result.reserve(payload.size() * 2);
        static const char hex[] = "0123456789abcdef";
        for (uint8_t b : payload) {
            result += hex[b >> 4];
            result += hex[b & 0x0f];
        }
        return result;
    }

    /**
     * @brief Get printable ASCII from payload
     */
    std::string payload_ascii() const {
        std::string result;
        result.reserve(payload.size());
        for (uint8_t b : payload) {
            if (b >= 0x20 && b <= 0x7E) {
                result += static_cast<char>(b);
            }
        }
        return result;
    }

    /**
     * @brief Get log level string
     */
    const char* log_level_str() const {
        switch (msg_subtype) {
            case DLT_LOG_OFF:     return "OFF";
            case DLT_LOG_FATAL:   return "FATAL";
            case DLT_LOG_ERROR:   return "ERROR";
            case DLT_LOG_WARN:    return "WARN";
            case DLT_LOG_INFO:    return "INFO";
            case DLT_LOG_DEBUG:   return "DEBUG";
            case DLT_LOG_VERBOSE: return "VERBOSE";
            default:              return "UNKNOWN";
        }
    }
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Get current timestamp in microseconds since epoch
 */
inline uint64_t get_timestamp_us() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

/**
 * @brief Get current timestamp in seconds (floating point)
 */
inline double get_timestamp_sec() {
    return static_cast<double>(get_timestamp_us()) / 1000000.0;
}

/**
 * @brief Convert 4-byte buffer to string (null-terminated or space-padded)
 */
inline std::string bytes_to_id(const uint8_t* data) {
    std::string result;
    result.reserve(4);
    for (int i = 0; i < 4; ++i) {
        if (data[i] == 0) break;
        if (data[i] >= 0x20 && data[i] <= 0x7E) {
            result += static_cast<char>(data[i]);
        }
    }
    return result;
}

/**
 * @brief Read 16-bit big-endian value
 */
inline uint16_t read_be16(const uint8_t* data) {
    return static_cast<uint16_t>((data[0] << 8) | data[1]);
}

/**
 * @brief Read 16-bit little-endian value
 */
inline uint16_t read_le16(const uint8_t* data) {
    return static_cast<uint16_t>((data[1] << 8) | data[0]);
}

/**
 * @brief Read 32-bit big-endian value
 */
inline uint32_t read_be32(const uint8_t* data) {
    return static_cast<uint32_t>(
        (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
}

/**
 * @brief Read 32-bit little-endian value
 */
inline uint32_t read_le32(const uint8_t* data) {
    return static_cast<uint32_t>(
        (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);
}

/**
 * @brief Write 32-bit big-endian value
 */
inline void write_be32(uint8_t* data, uint32_t value) {
    data[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
    data[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
    data[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>(value & 0xFF);
}

/**
 * @brief Write 64-bit big-endian value
 */
inline void write_be64(uint8_t* data, uint64_t value) {
    data[0] = static_cast<uint8_t>((value >> 56) & 0xFF);
    data[1] = static_cast<uint8_t>((value >> 48) & 0xFF);
    data[2] = static_cast<uint8_t>((value >> 40) & 0xFF);
    data[3] = static_cast<uint8_t>((value >> 32) & 0xFF);
    data[4] = static_cast<uint8_t>((value >> 24) & 0xFF);
    data[5] = static_cast<uint8_t>((value >> 16) & 0xFF);
    data[6] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data[7] = static_cast<uint8_t>(value & 0xFF);
}

/**
 * @brief Read 64-bit big-endian value
 */
inline uint64_t read_be64(const uint8_t* data) {
    return (static_cast<uint64_t>(data[0]) << 56) |
           (static_cast<uint64_t>(data[1]) << 48) |
           (static_cast<uint64_t>(data[2]) << 40) |
           (static_cast<uint64_t>(data[3]) << 32) |
           (static_cast<uint64_t>(data[4]) << 24) |
           (static_cast<uint64_t>(data[5]) << 16) |
           (static_cast<uint64_t>(data[6]) << 8) |
           static_cast<uint64_t>(data[7]);
}

// ============================================================================
// Default Configuration Values
// ============================================================================

constexpr const char* DEFAULT_SERIAL_PORT = "/dev/ttyUSB2";
constexpr int DEFAULT_BAUD_RATE = 115200;
constexpr const char* DEFAULT_DATA_SOCKET = "/tmp/dls-data.sock";
constexpr int DEFAULT_CONTROL_PORT = 3491;
constexpr size_t DEFAULT_BUFFER_SIZE_KB = 1024;
constexpr size_t DEFAULT_CLIENT_QUEUE_SIZE = 1000;

} // namespace dls

#endif // DLS_COMMON_HPP
