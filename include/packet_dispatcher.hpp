/**
 * @file packet_dispatcher.hpp
 * @brief Packet dispatcher for routing packets to subscribed clients
 */

#ifndef DLS_PACKET_DISPATCHER_HPP
#define DLS_PACKET_DISPATCHER_HPP

#include "common.hpp"
#include "stats.hpp"

#include <memory>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <set>
#include <unordered_map>
#include <atomic>

namespace dls {

/**
 * @brief Client subscription filter
 */
struct ClientFilter {
    bool all_packets = true;                    // If true, receive all packets
    std::set<std::string> categories;           // Filter by category
    std::set<uint32_t> msg_ids;                 // Filter by message ID
};

/**
 * @brief Unique client identifier
 */
using ClientId = uint64_t;

/**
 * @brief Packet dispatcher for routing to multiple clients
 */
class PacketDispatcher {
public:
    PacketDispatcher(Stats& stats);

    // ========================================================================
    // Packet Dispatch
    // ========================================================================

    /**
     * @brief Dispatch a packet to all subscribed clients
     * @param packet Packet to dispatch (moved)
     */
    void dispatch(DltPacket&& packet);

    // ========================================================================
    // Client Management
    // ========================================================================

    /**
     * @brief Add a new client
     * @return Unique client ID
     */
    ClientId add_client();

    /**
     * @brief Remove a client
     * @param id Client ID to remove
     */
    void remove_client(ClientId id);

    /**
     * @brief Set filter for a client
     * @param id Client ID
     * @param filter Filter configuration
     */
    void set_filter(ClientId id, const ClientFilter& filter);

    /**
     * @brief Get current filter for a client
     * @param id Client ID
     * @return Filter (default if not found)
     */
    ClientFilter get_filter(ClientId id) const;

    // ========================================================================
    // Packet Retrieval
    // ========================================================================

    /**
     * @brief Get next packet for a client (blocking with timeout)
     * @param id Client ID
     * @param packet Output packet
     * @param timeout_ms Timeout in milliseconds (0 = non-blocking)
     * @return true if packet retrieved, false on timeout
     */
    bool get_packet(ClientId id, DltPacket& packet, int timeout_ms = 100);

    /**
     * @brief Check if client has packets queued
     * @param id Client ID
     */
    bool has_packets(ClientId id) const;

    /**
     * @brief Get queue depth for a client
     * @param id Client ID
     */
    size_t queue_depth(ClientId id) const;

    /**
     * @brief Clear queued packets for a client
     * @param id Client ID
     */
    void clear_queue(ClientId id);

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * @brief Set maximum queue size per client
     * @param id Client ID
     * @param max_packets Maximum packets in queue
     */
    void set_queue_size(ClientId id, size_t max_packets);

    // ========================================================================
    // Information
    // ========================================================================

    /**
     * @brief Get number of connected clients
     */
    size_t client_count() const;

    /**
     * @brief Get list of client IDs
     */
    std::vector<ClientId> get_client_ids() const;

    /**
     * @brief Get client info as JSON
     * @param id Client ID
     */
    nlohmann::json get_client_info(ClientId id) const;

private:
    struct ClientQueue {
        ClientFilter filter;
        std::deque<DltPacket> queue;
        size_t max_size = DEFAULT_CLIENT_QUEUE_SIZE;
        mutable std::mutex mutex;
        std::condition_variable cv;
        uint64_t dropped = 0;
        uint64_t received = 0;
    };

    bool matches_filter(const DltPacket& packet, const ClientFilter& filter) const;

    Stats& stats_;

    mutable std::mutex clients_mutex_;
    std::unordered_map<ClientId, std::unique_ptr<ClientQueue>> clients_;
    std::atomic<ClientId> next_id_{1};
};

} // namespace dls

#endif // DLS_PACKET_DISPATCHER_HPP
