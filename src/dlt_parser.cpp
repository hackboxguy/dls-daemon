/**
 * @file dlt_parser.cpp
 * @brief DLT packet parser implementation
 */

#include "dlt_parser.hpp"

#include <iostream>
#include <cstring>

namespace dls {

DltParser::DltParser(RingBuffer& buffer, Stats& stats)
    : buffer_(buffer)
    , stats_(stats)
{
}

DltParser::~DltParser() {
    stop();
}

// ============================================================================
// Configuration
// ============================================================================

void DltParser::set_packet_callback(PacketCallback callback) {
    packet_callback_ = std::move(callback);
}

// ============================================================================
// Thread Control
// ============================================================================

void DltParser::start() {
    if (running_.load(std::memory_order_relaxed)) {
        return;
    }

    running_.store(true, std::memory_order_relaxed);
    parser_thread_ = std::thread(&DltParser::parser_loop, this);
}

void DltParser::stop() {
    running_.store(false, std::memory_order_relaxed);

    if (parser_thread_.joinable()) {
        parser_thread_.join();
    }
}

bool DltParser::is_running() const {
    return running_.load(std::memory_order_relaxed);
}

// ============================================================================
// Parser Thread
// ============================================================================

void DltParser::parser_loop() {
    while (running_.load(std::memory_order_relaxed)) {
        // Try to parse packets
        bool parsed = false;
        while (parse_next()) {
            parsed = true;
        }

        // If no packets parsed, wait a bit
        if (!parsed) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
}

// ============================================================================
// Parsing
// ============================================================================

bool DltParser::parse_next() {
    // Need at least minimum packet size
    if (buffer_.read_available() < DLT_MIN_PACKET_SIZE) {
        return false;
    }

    // Find sync (DLS magic)
    if (!find_sync()) {
        return false;
    }

    // Try to parse packet
    DltPacket packet;
    if (!try_parse_packet(packet)) {
        return false;
    }

    // Check for mcnt gaps (per source - each ECU/APP/CTX has its own counter)
    // Build composite key from available identifiers
    std::string mcnt_key;
    if (!packet.ecu_id.empty()) {
        mcnt_key = packet.ecu_id;
    }
    if (!packet.apid.empty()) {
        mcnt_key += ":" + packet.apid;
    }
    if (!packet.ctid.empty()) {
        mcnt_key += ":" + packet.ctid;
    }
    if (mcnt_key.empty()) {
        mcnt_key = "_default_";
    }
    check_mcnt_gap(mcnt_key, packet.mcnt);

    // Assign sequence number
    packet.sequence = sequence_.fetch_add(1, std::memory_order_relaxed);

    // Set capture timestamp
    packet.capture_ts_us = get_timestamp_us();

    // Stats
    stats_.increment_packets();

    // Invoke callback
    if (packet_callback_) {
        packet_callback_(std::move(packet));
    }

    return true;
}

bool DltParser::find_sync() {
    // Search for "DLS\x01" magic
    ssize_t offset = buffer_.find_dls_magic();

    if (offset < 0) {
        // No magic found - skip all but last 3 bytes (magic might be split)
        size_t available = buffer_.read_available();
        if (available > 3) {
            buffer_.skip(available - 3);
        }
        return false;
    }

    if (offset > 0) {
        // Skip garbage bytes before magic
        buffer_.skip(static_cast<size_t>(offset));
    }

    return true;
}

bool DltParser::try_parse_packet(DltPacket& packet) {
    size_t available = buffer_.read_available();

    // Need minimum header to determine full size
    if (available < DLT_MIN_PACKET_SIZE) {
        return false;
    }

    // Peek at header
    if (buffer_.peek(peek_buf_, DLT_MIN_PACKET_SIZE) < DLT_MIN_PACKET_SIZE) {
        return false;
    }

    // Verify magic
    if (peek_buf_[0] != 'D' || peek_buf_[1] != 'L' ||
        peek_buf_[2] != 'S' || peek_buf_[3] != 0x01) {
        // Not at valid sync - skip one byte and try again
        buffer_.skip(1);
        stats_.increment_parse_errors();
        return false;
    }

    // Parse standard header
    uint8_t htyp = peek_buf_[4];
    uint8_t mcnt = peek_buf_[5];
    uint16_t msg_len = read_be16(&peek_buf_[6]);

    // Extract flags
    packet.htyp = htyp;
    packet.mcnt = mcnt;
    packet.msg_len = msg_len;

    packet.has_ext_header = (htyp & DLT_HTYP_UEH) != 0;
    packet.is_big_endian = (htyp & DLT_HTYP_MSBF) != 0;
    packet.has_ecu_id = (htyp & DLT_HTYP_WEID) != 0;
    packet.has_session_id = (htyp & DLT_HTYP_WSID) != 0;
    packet.has_timestamp = (htyp & DLT_HTYP_WTMS) != 0;

    // Calculate total packet size
    // Serial header (4) + msg_len (includes standard header)
    size_t total_size = DLT_SERIAL_HEADER_SIZE + msg_len;

    // Sanity check
    if (total_size < DLT_MIN_PACKET_SIZE || total_size > DLT_MAX_PACKET_SIZE) {
        // Invalid length - skip magic and try again
        buffer_.skip(1);
        stats_.increment_parse_errors();
        return false;
    }

    // Check if we have complete packet
    if (available < total_size) {
        // Incomplete packet - wait for more data
        return false;
    }

    // Read complete packet
    packet.raw_data.resize(total_size);
    if (buffer_.read(packet.raw_data.data(), total_size) < total_size) {
        stats_.increment_parse_errors();
        return false;
    }

    packet.total_size = total_size;

    // Parse remaining fields
    const uint8_t* data = packet.raw_data.data();
    size_t pos = DLT_SERIAL_HEADER_SIZE + DLT_STD_HEADER_SIZE;  // After serial + std header

    // Optional ECU ID
    if (packet.has_ecu_id) {
        if (pos + 4 > total_size) {
            stats_.increment_parse_errors();
            return false;
        }
        packet.ecu_id = bytes_to_id(&data[pos]);
        pos += 4;
    }

    // Optional Session ID
    if (packet.has_session_id) {
        if (pos + 4 > total_size) {
            stats_.increment_parse_errors();
            return false;
        }
        if (packet.is_big_endian) {
            packet.session_id = read_be32(&data[pos]);
        } else {
            packet.session_id = read_le32(&data[pos]);
        }
        pos += 4;
    }

    // Optional Timestamp
    if (packet.has_timestamp) {
        if (pos + 4 > total_size) {
            stats_.increment_parse_errors();
            return false;
        }
        if (packet.is_big_endian) {
            packet.timestamp = read_be32(&data[pos]);
        } else {
            packet.timestamp = read_le32(&data[pos]);
        }
        pos += 4;
    }

    // Extended header
    if (packet.has_ext_header) {
        if (pos + DLT_EXT_HEADER_SIZE > total_size) {
            stats_.increment_parse_errors();
            return false;
        }

        uint8_t msin = data[pos];
        packet.noar = data[pos + 1];
        packet.apid = bytes_to_id(&data[pos + 2]);
        packet.ctid = bytes_to_id(&data[pos + 6]);
        pos += DLT_EXT_HEADER_SIZE;

        // Parse msin
        packet.is_verbose = (msin & 0x01) != 0;
        packet.msg_type = (msin >> 1) & 0x07;
        packet.msg_subtype = (msin >> 4) & 0x0F;
    }

    // Payload starts at pos
    size_t payload_size = total_size - pos;

    // For non-verbose messages, first 4 bytes of payload is message ID
    if (!packet.is_verbose && payload_size >= 4) {
        if (packet.is_big_endian) {
            packet.msg_id = read_be32(&data[pos]);
        } else {
            packet.msg_id = read_le32(&data[pos]);
        }
        pos += 4;
        payload_size -= 4;
    }

    // Copy remaining payload
    if (payload_size > 0) {
        packet.payload.assign(&data[pos], &data[pos + payload_size]);
    }

    return true;
}

void DltParser::check_mcnt_gap(const std::string& ecu_id, uint8_t current_mcnt) {
    auto& state = mcnt_per_ecu_[ecu_id];

    if (state.valid) {
        uint8_t expected = (state.last_mcnt + 1) & 0xFF;
        if (current_mcnt != expected) {
            // Gap detected
            int gap = (current_mcnt - expected) & 0xFF;
            if (gap > 128) {
                // Likely wrapped the other way (or reset)
                gap = 256 - gap;
            }
            stats_.record_mcnt_gap(expected, current_mcnt, gap);
        }
    }
    state.last_mcnt = current_mcnt;
    state.valid = true;
}

} // namespace dls
