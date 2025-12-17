/**
 * @file stats.hpp
 * @brief Statistics collection for DLS daemon
 */

#ifndef DLS_STATS_HPP
#define DLS_STATS_HPP

#include "common.hpp"
#include "ring_buffer.hpp"

#include <atomic>
#include <mutex>
#include <deque>
#include <chrono>
#include <nlohmann/json.hpp>

namespace dls {

/**
 * @brief Record of a message counter gap (dropped packets)
 */
struct McntGapRecord {
    uint64_t timestamp_us;      // When gap was detected
    uint8_t expected;           // Expected mcnt value
    uint8_t actual;             // Actual mcnt value received
    int gap_size;               // Number of packets assumed dropped
};

/**
 * @brief Statistics collector for daemon monitoring
 *
 * Thread-safe statistics collection using atomic operations where possible
 * and mutex protection for complex data structures.
 */
class Stats {
public:
    Stats();

    // ========================================================================
    // Serial Reader Stats
    // ========================================================================

    void add_bytes_read(size_t bytes);
    void increment_reconnects();
    void set_serial_connected(bool connected);

    uint64_t bytes_read() const;
    uint32_t reconnect_count() const;
    bool serial_connected() const;

    // ========================================================================
    // Parser Stats
    // ========================================================================

    void increment_packets();
    void increment_parse_errors();
    void set_gap_threshold(int threshold);
    void record_mcnt_gap(uint8_t expected, uint8_t actual, int gap_size);

    uint64_t packets_total() const;
    uint64_t parse_errors() const;
    uint64_t mcnt_gaps_total() const;
    uint64_t dropped_estimate() const;

    // ========================================================================
    // Ring Buffer Stats (pulled from RingBuffer object)
    // ========================================================================

    void update_buffer_stats(const RingBuffer& rb);

    size_t buffer_size_kb() const;
    size_t buffer_used_kb() const;
    size_t buffer_high_water_kb() const;
    uint64_t buffer_overflows() const;

    // ========================================================================
    // Client Stats
    // ========================================================================

    void increment_data_clients();
    void decrement_data_clients();
    void increment_control_clients();
    void decrement_control_clients();

    uint32_t data_clients() const;
    uint32_t control_clients() const;

    // ========================================================================
    // Logging Stats
    // ========================================================================

    void set_logging_enabled(bool enabled, const std::string& file = "");
    void add_logged_bytes(size_t bytes);

    bool logging_enabled() const;
    std::string log_file() const;
    uint64_t bytes_logged() const;

    // ========================================================================
    // Throughput Calculation
    // ========================================================================

    /**
     * @brief Update throughput calculations (call periodically, e.g., every second)
     */
    void update_throughput();

    double packets_per_second() const;
    double bytes_per_second() const;

    // ========================================================================
    // Gap History
    // ========================================================================

    /**
     * @brief Get recent mcnt gaps
     * @param max_count Maximum number of gaps to return
     */
    std::vector<McntGapRecord> recent_gaps(size_t max_count = 100) const;

    // ========================================================================
    // Uptime
    // ========================================================================

    uint64_t uptime_seconds() const;

    // ========================================================================
    // JSON Export
    // ========================================================================

    /**
     * @brief Export all stats as JSON
     * @param serial_port Serial port name for inclusion in output
     * @param baud_rate Baud rate for inclusion in output
     */
    nlohmann::json to_json(const std::string& serial_port = "",
                           int baud_rate = 0) const;

    // ========================================================================
    // Reset
    // ========================================================================

    void reset();
    void reset_throughput();

private:
    // Start time for uptime calculation
    std::chrono::steady_clock::time_point start_time_;

    // Serial stats
    std::atomic<uint64_t> bytes_read_{0};
    std::atomic<uint32_t> reconnect_count_{0};
    std::atomic<bool> serial_connected_{false};

    // Parser stats
    std::atomic<uint64_t> packets_total_{0};
    std::atomic<uint64_t> parse_errors_{0};
    std::atomic<uint64_t> mcnt_gaps_total_{0};
    std::atomic<uint64_t> dropped_estimate_{0};
    std::atomic<int> gap_threshold_{1};  // Min gap to report (0 = disabled)

    // Buffer stats (updated by update_buffer_stats)
    std::atomic<size_t> buffer_capacity_{0};
    std::atomic<size_t> buffer_used_{0};
    std::atomic<size_t> buffer_high_water_{0};
    std::atomic<uint64_t> buffer_overflows_{0};

    // Client stats
    std::atomic<uint32_t> data_clients_{0};
    std::atomic<uint32_t> control_clients_{0};

    // Logging stats
    std::atomic<bool> logging_enabled_{false};
    std::atomic<uint64_t> bytes_logged_{0};
    mutable std::mutex log_mutex_;
    std::string log_file_;

    // Throughput calculation
    std::atomic<double> packets_per_sec_{0.0};
    std::atomic<double> bytes_per_sec_{0.0};
    uint64_t last_packets_{0};
    uint64_t last_bytes_{0};
    std::chrono::steady_clock::time_point last_throughput_time_;

    // Gap history
    mutable std::mutex gap_mutex_;
    std::deque<McntGapRecord> gap_history_;
    static constexpr size_t MAX_GAP_HISTORY = 1000;
};

} // namespace dls

#endif // DLS_STATS_HPP
