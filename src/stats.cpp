/**
 * @file stats.cpp
 * @brief Statistics collection implementation
 */

#include "stats.hpp"

namespace dls {

Stats::Stats()
    : start_time_(std::chrono::steady_clock::now())
    , last_throughput_time_(start_time_)
{
}

// ============================================================================
// Serial Reader Stats
// ============================================================================

void Stats::add_bytes_read(size_t bytes) {
    bytes_read_.fetch_add(bytes, std::memory_order_relaxed);
}

void Stats::increment_reconnects() {
    reconnect_count_.fetch_add(1, std::memory_order_relaxed);
}

void Stats::set_serial_connected(bool connected) {
    serial_connected_.store(connected, std::memory_order_relaxed);
}

uint64_t Stats::bytes_read() const {
    return bytes_read_.load(std::memory_order_relaxed);
}

uint32_t Stats::reconnect_count() const {
    return reconnect_count_.load(std::memory_order_relaxed);
}

bool Stats::serial_connected() const {
    return serial_connected_.load(std::memory_order_relaxed);
}

// ============================================================================
// Parser Stats
// ============================================================================

void Stats::increment_packets() {
    packets_total_.fetch_add(1, std::memory_order_relaxed);
}

void Stats::increment_parse_errors() {
    parse_errors_.fetch_add(1, std::memory_order_relaxed);
}

void Stats::set_gap_threshold(int threshold) {
    gap_threshold_.store(threshold, std::memory_order_relaxed);
}

void Stats::record_mcnt_gap(uint8_t expected, uint8_t actual, int gap_size) {
    int threshold = gap_threshold_.load(std::memory_order_relaxed);

    // Skip if gap tracking disabled or gap below threshold
    if (threshold <= 0 || gap_size < threshold) {
        return;
    }

    mcnt_gaps_total_.fetch_add(1, std::memory_order_relaxed);
    dropped_estimate_.fetch_add(static_cast<uint64_t>(gap_size),
                                std::memory_order_relaxed);

    // Record gap in history
    McntGapRecord record;
    record.timestamp_us = get_timestamp_us();
    record.expected = expected;
    record.actual = actual;
    record.gap_size = gap_size;

    std::lock_guard<std::mutex> lock(gap_mutex_);
    gap_history_.push_back(record);
    if (gap_history_.size() > MAX_GAP_HISTORY) {
        gap_history_.pop_front();
    }
}

uint64_t Stats::packets_total() const {
    return packets_total_.load(std::memory_order_relaxed);
}

uint64_t Stats::parse_errors() const {
    return parse_errors_.load(std::memory_order_relaxed);
}

uint64_t Stats::mcnt_gaps_total() const {
    return mcnt_gaps_total_.load(std::memory_order_relaxed);
}

uint64_t Stats::dropped_estimate() const {
    return dropped_estimate_.load(std::memory_order_relaxed);
}

// ============================================================================
// Ring Buffer Stats
// ============================================================================

void Stats::update_buffer_stats(const RingBuffer& rb) {
    buffer_capacity_.store(rb.capacity(), std::memory_order_relaxed);
    buffer_used_.store(rb.read_available(), std::memory_order_relaxed);
    buffer_high_water_.store(rb.high_water_mark(), std::memory_order_relaxed);
    buffer_overflows_.store(rb.overflow_count(), std::memory_order_relaxed);
}

size_t Stats::buffer_size_kb() const {
    return buffer_capacity_.load(std::memory_order_relaxed) / 1024;
}

size_t Stats::buffer_used_kb() const {
    return buffer_used_.load(std::memory_order_relaxed) / 1024;
}

size_t Stats::buffer_high_water_kb() const {
    return buffer_high_water_.load(std::memory_order_relaxed) / 1024;
}

uint64_t Stats::buffer_overflows() const {
    return buffer_overflows_.load(std::memory_order_relaxed);
}

// ============================================================================
// Client Stats
// ============================================================================

void Stats::increment_data_clients() {
    data_clients_.fetch_add(1, std::memory_order_relaxed);
}

void Stats::decrement_data_clients() {
    data_clients_.fetch_sub(1, std::memory_order_relaxed);
}

void Stats::increment_control_clients() {
    control_clients_.fetch_add(1, std::memory_order_relaxed);
}

void Stats::decrement_control_clients() {
    control_clients_.fetch_sub(1, std::memory_order_relaxed);
}

uint32_t Stats::data_clients() const {
    return data_clients_.load(std::memory_order_relaxed);
}

uint32_t Stats::control_clients() const {
    return control_clients_.load(std::memory_order_relaxed);
}

// ============================================================================
// Logging Stats
// ============================================================================

void Stats::set_logging_enabled(bool enabled, const std::string& file) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    logging_enabled_.store(enabled, std::memory_order_relaxed);
    log_file_ = file;
    if (!enabled) {
        bytes_logged_.store(0, std::memory_order_relaxed);
    }
}

void Stats::add_logged_bytes(size_t bytes) {
    bytes_logged_.fetch_add(bytes, std::memory_order_relaxed);
}

bool Stats::logging_enabled() const {
    return logging_enabled_.load(std::memory_order_relaxed);
}

std::string Stats::log_file() const {
    std::lock_guard<std::mutex> lock(log_mutex_);
    return log_file_;
}

uint64_t Stats::bytes_logged() const {
    return bytes_logged_.load(std::memory_order_relaxed);
}

// ============================================================================
// Throughput Calculation
// ============================================================================

void Stats::update_throughput() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_throughput_time_).count();

    if (elapsed < 100) {
        return;  // Don't update too frequently
    }

    double elapsed_sec = elapsed / 1000.0;

    uint64_t current_packets = packets_total_.load(std::memory_order_relaxed);
    uint64_t current_bytes = bytes_read_.load(std::memory_order_relaxed);

    uint64_t packet_diff = current_packets - last_packets_;
    uint64_t bytes_diff = current_bytes - last_bytes_;

    packets_per_sec_.store(packet_diff / elapsed_sec, std::memory_order_relaxed);
    bytes_per_sec_.store(bytes_diff / elapsed_sec, std::memory_order_relaxed);

    last_packets_ = current_packets;
    last_bytes_ = current_bytes;
    last_throughput_time_ = now;
}

double Stats::packets_per_second() const {
    return packets_per_sec_.load(std::memory_order_relaxed);
}

double Stats::bytes_per_second() const {
    return bytes_per_sec_.load(std::memory_order_relaxed);
}

// ============================================================================
// Gap History
// ============================================================================

std::vector<McntGapRecord> Stats::recent_gaps(size_t max_count) const {
    std::lock_guard<std::mutex> lock(gap_mutex_);

    std::vector<McntGapRecord> result;
    size_t count = std::min(max_count, gap_history_.size());
    result.reserve(count);

    // Return most recent gaps
    auto it = gap_history_.end();
    for (size_t i = 0; i < count; ++i) {
        --it;
        result.push_back(*it);
    }

    return result;
}

// ============================================================================
// Uptime
// ============================================================================

uint64_t Stats::uptime_seconds() const {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::seconds>(
        now - start_time_).count();
}

// ============================================================================
// JSON Export
// ============================================================================

nlohmann::json Stats::to_json(const std::string& serial_port,
                               int baud_rate) const {
    nlohmann::json j;

    j["uptime_sec"] = uptime_seconds();

    // Serial stats
    j["serial"]["port"] = serial_port;
    j["serial"]["baud"] = baud_rate;
    j["serial"]["state"] = serial_connected() ? "connected" : "disconnected";
    j["serial"]["bytes_read"] = bytes_read();
    j["serial"]["reconnects"] = reconnect_count();

    // Packet stats
    j["packets"]["total"] = packets_total();
    j["packets"]["per_second"] = packets_per_second();
    j["packets"]["parse_errors"] = parse_errors();
    j["packets"]["mcnt_gaps"] = mcnt_gaps_total();
    j["packets"]["dropped_estimate"] = dropped_estimate();

    // Buffer stats
    j["ring_buffer"]["size_kb"] = buffer_size_kb();
    j["ring_buffer"]["used_kb"] = buffer_used_kb();
    j["ring_buffer"]["high_water_kb"] = buffer_high_water_kb();
    j["ring_buffer"]["overflows"] = buffer_overflows();

    // Client stats
    j["clients"]["data_connected"] = data_clients();
    j["clients"]["control_connected"] = control_clients();

    // Logging stats
    j["logging"]["enabled"] = logging_enabled();
    j["logging"]["file"] = log_file();
    j["logging"]["bytes_written"] = bytes_logged();

    // Recent gaps (last 10)
    auto gaps = recent_gaps(10);
    j["recent_gaps"] = nlohmann::json::array();
    for (const auto& gap : gaps) {
        nlohmann::json gap_j;
        gap_j["ts"] = gap.timestamp_us;
        gap_j["expected"] = gap.expected;
        gap_j["actual"] = gap.actual;
        gap_j["gap"] = gap.gap_size;
        j["recent_gaps"].push_back(gap_j);
    }

    return j;
}

// ============================================================================
// Reset
// ============================================================================

void Stats::reset() {
    bytes_read_.store(0, std::memory_order_relaxed);
    reconnect_count_.store(0, std::memory_order_relaxed);
    packets_total_.store(0, std::memory_order_relaxed);
    parse_errors_.store(0, std::memory_order_relaxed);
    mcnt_gaps_total_.store(0, std::memory_order_relaxed);
    dropped_estimate_.store(0, std::memory_order_relaxed);
    bytes_logged_.store(0, std::memory_order_relaxed);

    reset_throughput();

    {
        std::lock_guard<std::mutex> lock(gap_mutex_);
        gap_history_.clear();
    }

    start_time_ = std::chrono::steady_clock::now();
}

void Stats::reset_throughput() {
    packets_per_sec_.store(0.0, std::memory_order_relaxed);
    bytes_per_sec_.store(0.0, std::memory_order_relaxed);
    last_packets_ = packets_total_.load(std::memory_order_relaxed);
    last_bytes_ = bytes_read_.load(std::memory_order_relaxed);
    last_throughput_time_ = std::chrono::steady_clock::now();
}

} // namespace dls
