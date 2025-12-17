/**
 * @file binary_logger.cpp
 * @brief Binary logger implementation
 */

#include "binary_logger.hpp"

#include <cstring>
#include <iostream>
#include <fstream>

namespace dls {

BinaryLogger::BinaryLogger() = default;

BinaryLogger::~BinaryLogger() {
    close();
}

// ============================================================================
// Write Mode
// ============================================================================

bool BinaryLogger::open(const std::string& filename, const std::string& port, int baud) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (file_.is_open()) {
        file_.close();
    }

    filename_ = filename;
    bytes_written_.store(0, std::memory_order_relaxed);

    file_.open(filename, std::ios::binary | std::ios::out | std::ios::trunc);
    if (!file_.is_open()) {
        std::cerr << "Error: Cannot open log file: " << filename << "\n";
        return false;
    }

    if (!write_header(port, baud)) {
        file_.close();
        return false;
    }

    return true;
}

bool BinaryLogger::write_header(const std::string& port, int baud) {
    BinaryLogHeader header;
    memset(&header, 0, sizeof(header));

    memcpy(header.magic, BINARY_LOG_MAGIC, 4);
    header.version = BINARY_LOG_VERSION;
    header.start_timestamp = get_timestamp_us();
    strncpy(header.serial_port, port.c_str(), sizeof(header.serial_port) - 1);
    header.baud_rate = static_cast<uint32_t>(baud);

    file_.write(reinterpret_cast<const char*>(&header), sizeof(header));
    if (!file_.good()) {
        std::cerr << "Error: Failed to write log header\n";
        return false;
    }

    bytes_written_.fetch_add(sizeof(header), std::memory_order_relaxed);
    return true;
}

void BinaryLogger::close() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (file_.is_open()) {
        file_.flush();
        file_.close();
    }
}

bool BinaryLogger::is_open() const {
    return file_.is_open();
}

void BinaryLogger::log_packet(const uint8_t* data, size_t len, uint64_t timestamp_us) {
    if (!file_.is_open() || len == 0) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    // Write record: timestamp (8) + length (4) + data
    uint64_t ts = timestamp_us;
    uint32_t length = static_cast<uint32_t>(len);

    file_.write(reinterpret_cast<const char*>(&ts), sizeof(ts));
    file_.write(reinterpret_cast<const char*>(&length), sizeof(length));
    file_.write(reinterpret_cast<const char*>(data), len);

    bytes_written_.fetch_add(sizeof(ts) + sizeof(length) + len,
                             std::memory_order_relaxed);
}

void BinaryLogger::log_packet(const DltPacket& packet) {
    if (!packet.raw_data.empty()) {
        log_packet(packet.raw_data.data(), packet.raw_data.size(),
                   packet.capture_ts_us);
    }
}

uint64_t BinaryLogger::bytes_written() const {
    return bytes_written_.load(std::memory_order_relaxed);
}

std::string BinaryLogger::filename() const {
    return filename_;
}

// ============================================================================
// Read Mode
// ============================================================================

bool BinaryLogger::open_for_replay(const std::string& filename,
                                    std::string& port, int& baud) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open replay file: " << filename << "\n";
        return false;
    }

    BinaryLogHeader header;
    file.read(reinterpret_cast<char*>(&header), sizeof(header));
    if (!file.good()) {
        std::cerr << "Error: Cannot read replay file header\n";
        return false;
    }

    // Verify magic
    if (memcmp(header.magic, BINARY_LOG_MAGIC, 4) != 0) {
        std::cerr << "Error: Invalid replay file magic\n";
        return false;
    }

    // Check version
    if (header.version != BINARY_LOG_VERSION) {
        std::cerr << "Warning: Replay file version " << header.version
                  << " (expected " << BINARY_LOG_VERSION << ")\n";
    }

    port = header.serial_port;
    baud = static_cast<int>(header.baud_rate);

    return true;
}

} // namespace dls
