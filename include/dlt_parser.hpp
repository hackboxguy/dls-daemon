/**
 * @file dlt_parser.hpp
 * @brief DLT/DLS packet parser
 *
 * Parses DLT packets from ring buffer following GENIVI/COVESA specification.
 */

#ifndef DLS_DLT_PARSER_HPP
#define DLS_DLT_PARSER_HPP

#include "ring_buffer.hpp"
#include "stats.hpp"
#include "common.hpp"

#include <thread>
#include <atomic>
#include <functional>
#include <unordered_map>
#include <string>

namespace dls {

/**
 * @brief DLT packet parser with dedicated thread
 */
class DltParser {
public:
    using PacketCallback = std::function<void(DltPacket&&)>;

    /**
     * @brief Construct parser
     * @param buffer Ring buffer to read from
     * @param stats Statistics collector
     */
    DltParser(RingBuffer& buffer, Stats& stats);

    ~DltParser();

    // Non-copyable
    DltParser(const DltParser&) = delete;
    DltParser& operator=(const DltParser&) = delete;

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * @brief Set callback for parsed packets
     * @param callback Function called for each parsed packet
     */
    void set_packet_callback(PacketCallback callback);

    // ========================================================================
    // Thread Control
    // ========================================================================

    /**
     * @brief Start parser thread
     */
    void start();

    /**
     * @brief Stop parser thread
     */
    void stop();

    /**
     * @brief Check if parser is running
     */
    bool is_running() const;

    // ========================================================================
    // Manual Parsing (for testing)
    // ========================================================================

    /**
     * @brief Try to parse next packet from buffer
     * @return true if packet was parsed (callback invoked)
     */
    bool parse_next();

private:
    void parser_loop();

    bool find_sync();
    bool try_parse_packet(DltPacket& packet);
    void check_mcnt_gap(const std::string& ecu_id, uint8_t current_mcnt);

    RingBuffer& buffer_;
    Stats& stats_;
    PacketCallback packet_callback_;

    std::thread parser_thread_;
    std::atomic<bool> running_{false};

    // Sequence number for packets
    std::atomic<uint64_t> sequence_{0};

    // Message counter tracking per ECU (each ECU has its own mcnt)
    struct McntState {
        uint8_t last_mcnt = 0;
        bool valid = false;
    };
    std::unordered_map<std::string, McntState> mcnt_per_ecu_;

    // Parse buffer for peeking
    static constexpr size_t PEEK_BUFFER_SIZE = 256;
    uint8_t peek_buf_[PEEK_BUFFER_SIZE];
};

} // namespace dls

#endif // DLS_DLT_PARSER_HPP
