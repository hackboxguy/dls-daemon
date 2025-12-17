/**
 * @file packet_dispatcher.cpp
 * @brief Packet dispatcher implementation
 */

#include "packet_dispatcher.hpp"

namespace dls {

PacketDispatcher::PacketDispatcher(Stats& stats)
    : stats_(stats)
{
}

// ============================================================================
// Packet Dispatch
// ============================================================================

void PacketDispatcher::dispatch(DltPacket&& packet) {
    std::lock_guard<std::mutex> lock(clients_mutex_);

    for (auto& kv : clients_) {
        auto& client = kv.second;
        if (matches_filter(packet, client->filter)) {
            std::lock_guard<std::mutex> queue_lock(client->mutex);

            if (client->queue.size() >= client->max_size) {
                // Queue full - drop oldest packet
                client->queue.pop_front();
                client->dropped++;
            }

            // Make a copy for this client
            client->queue.push_back(packet);
            client->received++;
            client->cv.notify_one();
        }
    }
}

bool PacketDispatcher::matches_filter(const DltPacket& packet,
                                       const ClientFilter& filter) const {
    if (filter.all_packets) {
        return true;
    }

    // Check category filter
    if (!filter.categories.empty()) {
        if (filter.categories.count(packet.category) > 0) {
            return true;
        }
    }

    // Check message ID filter
    if (!filter.msg_ids.empty()) {
        if (filter.msg_ids.count(packet.msg_id) > 0) {
            return true;
        }
        // Also check lower 16 bits (FIBEX ID)
        uint32_t fibex_id = packet.msg_id & 0xFFFF;
        if (filter.msg_ids.count(fibex_id) > 0) {
            return true;
        }
    }

    return false;
}

// ============================================================================
// Client Management
// ============================================================================

ClientId PacketDispatcher::add_client() {
    ClientId id = next_id_.fetch_add(1, std::memory_order_relaxed);

    auto client = std::make_unique<ClientQueue>();

    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients_[id] = std::move(client);

    stats_.increment_data_clients();

    return id;
}

void PacketDispatcher::remove_client(ClientId id) {
    std::lock_guard<std::mutex> lock(clients_mutex_);

    auto it = clients_.find(id);
    if (it != clients_.end()) {
        // Wake up any waiting threads
        {
            std::lock_guard<std::mutex> queue_lock(it->second->mutex);
            it->second->cv.notify_all();
        }
        clients_.erase(it);
        stats_.decrement_data_clients();
    }
}

void PacketDispatcher::set_filter(ClientId id, const ClientFilter& filter) {
    std::lock_guard<std::mutex> lock(clients_mutex_);

    auto it = clients_.find(id);
    if (it != clients_.end()) {
        std::lock_guard<std::mutex> queue_lock(it->second->mutex);
        it->second->filter = filter;
    }
}

ClientFilter PacketDispatcher::get_filter(ClientId id) const {
    std::lock_guard<std::mutex> lock(clients_mutex_);

    auto it = clients_.find(id);
    if (it != clients_.end()) {
        std::lock_guard<std::mutex> queue_lock(it->second->mutex);
        return it->second->filter;
    }
    return ClientFilter{};
}

// ============================================================================
// Packet Retrieval
// ============================================================================

bool PacketDispatcher::get_packet(ClientId id, DltPacket& packet, int timeout_ms) {
    ClientQueue* client = nullptr;

    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        auto it = clients_.find(id);
        if (it == clients_.end()) {
            return false;
        }
        client = it->second.get();
    }

    std::unique_lock<std::mutex> queue_lock(client->mutex);

    if (timeout_ms > 0) {
        // Wait with timeout
        if (!client->cv.wait_for(queue_lock,
                std::chrono::milliseconds(timeout_ms),
                [&] { return !client->queue.empty(); })) {
            return false;  // Timeout
        }
    } else if (timeout_ms == 0) {
        // Non-blocking
        if (client->queue.empty()) {
            return false;
        }
    } else {
        // Block forever
        client->cv.wait(queue_lock, [&] { return !client->queue.empty(); });
    }

    packet = std::move(client->queue.front());
    client->queue.pop_front();

    return true;
}

bool PacketDispatcher::has_packets(ClientId id) const {
    std::lock_guard<std::mutex> lock(clients_mutex_);

    auto it = clients_.find(id);
    if (it != clients_.end()) {
        std::lock_guard<std::mutex> queue_lock(it->second->mutex);
        return !it->second->queue.empty();
    }
    return false;
}

size_t PacketDispatcher::queue_depth(ClientId id) const {
    std::lock_guard<std::mutex> lock(clients_mutex_);

    auto it = clients_.find(id);
    if (it != clients_.end()) {
        std::lock_guard<std::mutex> queue_lock(it->second->mutex);
        return it->second->queue.size();
    }
    return 0;
}

void PacketDispatcher::clear_queue(ClientId id) {
    std::lock_guard<std::mutex> lock(clients_mutex_);

    auto it = clients_.find(id);
    if (it != clients_.end()) {
        std::lock_guard<std::mutex> queue_lock(it->second->mutex);
        it->second->queue.clear();
    }
}

// ============================================================================
// Configuration
// ============================================================================

void PacketDispatcher::set_queue_size(ClientId id, size_t max_packets) {
    std::lock_guard<std::mutex> lock(clients_mutex_);

    auto it = clients_.find(id);
    if (it != clients_.end()) {
        std::lock_guard<std::mutex> queue_lock(it->second->mutex);
        it->second->max_size = max_packets;
    }
}

// ============================================================================
// Information
// ============================================================================

size_t PacketDispatcher::client_count() const {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    return clients_.size();
}

std::vector<ClientId> PacketDispatcher::get_client_ids() const {
    std::lock_guard<std::mutex> lock(clients_mutex_);

    std::vector<ClientId> ids;
    ids.reserve(clients_.size());
    for (const auto& kv : clients_) {
        ids.push_back(kv.first);
    }
    return ids;
}

nlohmann::json PacketDispatcher::get_client_info(ClientId id) const {
    std::lock_guard<std::mutex> lock(clients_mutex_);

    auto it = clients_.find(id);
    if (it == clients_.end()) {
        return nlohmann::json{};
    }

    std::lock_guard<std::mutex> queue_lock(it->second->mutex);

    nlohmann::json j;
    j["id"] = id;
    j["queue_depth"] = it->second->queue.size();
    j["max_queue_size"] = it->second->max_size;
    j["received"] = it->second->received;
    j["dropped"] = it->second->dropped;

    // Filter info
    j["filter"]["all_packets"] = it->second->filter.all_packets;

    j["filter"]["categories"] = nlohmann::json::array();
    for (const auto& cat : it->second->filter.categories) {
        j["filter"]["categories"].push_back(cat);
    }

    j["filter"]["msg_ids"] = nlohmann::json::array();
    for (uint32_t msg_id : it->second->filter.msg_ids) {
        j["filter"]["msg_ids"].push_back(msg_id);
    }

    return j;
}

} // namespace dls
