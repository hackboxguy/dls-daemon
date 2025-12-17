/**
 * @file ring_buffer.hpp
 * @brief Lock-free Single-Producer Single-Consumer (SPSC) ring buffer
 *
 * This ring buffer is designed for high-performance serial data capture:
 * - Lock-free using atomic operations (no mutex contention)
 * - Cache-line aligned to prevent false sharing
 * - Single producer (serial reader) and single consumer (parser)
 * - Supports both byte-level and block-level operations
 */

#ifndef DLS_RING_BUFFER_HPP
#define DLS_RING_BUFFER_HPP

#include <atomic>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <memory>
#include <algorithm>

namespace dls {

// Cache line size for alignment (typical x86/ARM)
constexpr size_t CACHE_LINE_SIZE = 64;

/**
 * @brief Lock-free SPSC ring buffer for byte streams
 *
 * Thread safety:
 * - One thread may call write()/write_available() (producer)
 * - One thread may call read()/read_available()/peek() (consumer)
 * - No locks needed between these operations
 */
class RingBuffer {
public:
    /**
     * @brief Construct ring buffer with given capacity
     * @param capacity Buffer size in bytes (will be rounded up to power of 2)
     */
    explicit RingBuffer(size_t capacity)
        : capacity_(next_power_of_2(capacity))
        , mask_(capacity_ - 1)
        , buffer_(new uint8_t[capacity_])
        , write_pos_(0)
        , read_pos_(0)
        , total_written_(0)
        , total_read_(0)
        , overflow_count_(0)
        , high_water_mark_(0)
    {
        std::memset(buffer_.get(), 0, capacity_);
    }

    ~RingBuffer() = default;

    // Non-copyable, non-movable (atomic members)
    RingBuffer(const RingBuffer&) = delete;
    RingBuffer& operator=(const RingBuffer&) = delete;
    RingBuffer(RingBuffer&&) = delete;
    RingBuffer& operator=(RingBuffer&&) = delete;

    /**
     * @brief Write data to the buffer (producer only)
     * @param data Source data pointer
     * @param len Number of bytes to write
     * @return Number of bytes actually written (may be less if buffer full)
     */
    size_t write(const uint8_t* data, size_t len) {
        const size_t write_idx = write_pos_.load(std::memory_order_relaxed);
        const size_t read_idx = read_pos_.load(std::memory_order_acquire);

        const size_t available = capacity_ - (write_idx - read_idx);
        const size_t to_write = std::min(len, available);

        if (to_write == 0) {
            overflow_count_.fetch_add(1, std::memory_order_relaxed);
            return 0;
        }

        if (to_write < len) {
            overflow_count_.fetch_add(1, std::memory_order_relaxed);
        }

        // Write data (may wrap around)
        const size_t pos = write_idx & mask_;
        const size_t first_chunk = std::min(to_write, capacity_ - pos);

        std::memcpy(buffer_.get() + pos, data, first_chunk);
        if (to_write > first_chunk) {
            std::memcpy(buffer_.get(), data + first_chunk, to_write - first_chunk);
        }

        // Update write position (release ensures data is visible before position update)
        write_pos_.store(write_idx + to_write, std::memory_order_release);
        total_written_.fetch_add(to_write, std::memory_order_relaxed);

        // Update high water mark
        size_t current_used = (write_idx + to_write) - read_idx;
        size_t current_high = high_water_mark_.load(std::memory_order_relaxed);
        while (current_used > current_high) {
            if (high_water_mark_.compare_exchange_weak(current_high, current_used,
                    std::memory_order_relaxed, std::memory_order_relaxed)) {
                break;
            }
        }

        return to_write;
    }

    /**
     * @brief Read data from the buffer (consumer only)
     * @param data Destination buffer
     * @param len Maximum bytes to read
     * @return Number of bytes actually read
     */
    size_t read(uint8_t* data, size_t len) {
        const size_t read_idx = read_pos_.load(std::memory_order_relaxed);
        const size_t write_idx = write_pos_.load(std::memory_order_acquire);

        const size_t available = write_idx - read_idx;
        const size_t to_read = std::min(len, available);

        if (to_read == 0) {
            return 0;
        }

        // Read data (may wrap around)
        const size_t pos = read_idx & mask_;
        const size_t first_chunk = std::min(to_read, capacity_ - pos);

        std::memcpy(data, buffer_.get() + pos, first_chunk);
        if (to_read > first_chunk) {
            std::memcpy(data + first_chunk, buffer_.get(), to_read - first_chunk);
        }

        // Update read position
        read_pos_.store(read_idx + to_read, std::memory_order_release);
        total_read_.fetch_add(to_read, std::memory_order_relaxed);

        return to_read;
    }

    /**
     * @brief Peek at data without consuming it (consumer only)
     * @param data Destination buffer
     * @param len Maximum bytes to peek
     * @param offset Offset from current read position
     * @return Number of bytes actually peeked
     */
    size_t peek(uint8_t* data, size_t len, size_t offset = 0) const {
        const size_t read_idx = read_pos_.load(std::memory_order_relaxed);
        const size_t write_idx = write_pos_.load(std::memory_order_acquire);

        const size_t available = write_idx - read_idx;
        if (offset >= available) {
            return 0;
        }

        const size_t to_peek = std::min(len, available - offset);
        const size_t pos = (read_idx + offset) & mask_;
        const size_t first_chunk = std::min(to_peek, capacity_ - pos);

        std::memcpy(data, buffer_.get() + pos, first_chunk);
        if (to_peek > first_chunk) {
            std::memcpy(data + first_chunk, buffer_.get(), to_peek - first_chunk);
        }

        return to_peek;
    }

    /**
     * @brief Peek a single byte at offset (consumer only)
     * @param offset Offset from current read position
     * @param byte Output byte
     * @return true if byte was read, false if offset beyond available data
     */
    bool peek_byte(size_t offset, uint8_t& byte) const {
        const size_t read_idx = read_pos_.load(std::memory_order_relaxed);
        const size_t write_idx = write_pos_.load(std::memory_order_acquire);

        const size_t available = write_idx - read_idx;
        if (offset >= available) {
            return false;
        }

        const size_t pos = (read_idx + offset) & mask_;
        byte = buffer_[pos];
        return true;
    }

    /**
     * @brief Skip bytes without copying (consumer only)
     * @param len Number of bytes to skip
     * @return Number of bytes actually skipped
     */
    size_t skip(size_t len) {
        const size_t read_idx = read_pos_.load(std::memory_order_relaxed);
        const size_t write_idx = write_pos_.load(std::memory_order_acquire);

        const size_t available = write_idx - read_idx;
        const size_t to_skip = std::min(len, available);

        read_pos_.store(read_idx + to_skip, std::memory_order_release);
        total_read_.fetch_add(to_skip, std::memory_order_relaxed);

        return to_skip;
    }

    /**
     * @brief Get number of bytes available to read
     */
    size_t read_available() const {
        const size_t write_idx = write_pos_.load(std::memory_order_acquire);
        const size_t read_idx = read_pos_.load(std::memory_order_relaxed);
        return write_idx - read_idx;
    }

    /**
     * @brief Get number of bytes available to write
     */
    size_t write_available() const {
        const size_t write_idx = write_pos_.load(std::memory_order_relaxed);
        const size_t read_idx = read_pos_.load(std::memory_order_acquire);
        return capacity_ - (write_idx - read_idx);
    }

    /**
     * @brief Check if buffer is empty
     */
    bool empty() const {
        return read_available() == 0;
    }

    /**
     * @brief Check if buffer is full
     */
    bool full() const {
        return write_available() == 0;
    }

    /**
     * @brief Get buffer capacity
     */
    size_t capacity() const {
        return capacity_;
    }

    /**
     * @brief Get total bytes written since creation
     */
    uint64_t total_written() const {
        return total_written_.load(std::memory_order_relaxed);
    }

    /**
     * @brief Get total bytes read since creation
     */
    uint64_t total_read() const {
        return total_read_.load(std::memory_order_relaxed);
    }

    /**
     * @brief Get overflow count (write attempts when full)
     */
    uint64_t overflow_count() const {
        return overflow_count_.load(std::memory_order_relaxed);
    }

    /**
     * @brief Get high water mark (maximum bytes used)
     */
    size_t high_water_mark() const {
        return high_water_mark_.load(std::memory_order_relaxed);
    }

    /**
     * @brief Reset the buffer (not thread-safe, call only when idle)
     */
    void reset() {
        write_pos_.store(0, std::memory_order_relaxed);
        read_pos_.store(0, std::memory_order_relaxed);
    }

    /**
     * @brief Reset statistics only
     */
    void reset_stats() {
        total_written_.store(0, std::memory_order_relaxed);
        total_read_.store(0, std::memory_order_relaxed);
        overflow_count_.store(0, std::memory_order_relaxed);
        high_water_mark_.store(0, std::memory_order_relaxed);
    }

    /**
     * @brief Search for a byte sequence in the buffer (consumer thread)
     * @param pattern Pattern to search for
     * @param pattern_len Length of pattern
     * @param start_offset Offset from read position to start search
     * @return Offset from read position where pattern starts, or -1 if not found
     */
    ssize_t find(const uint8_t* pattern, size_t pattern_len, size_t start_offset = 0) const {
        if (pattern_len == 0) {
            return static_cast<ssize_t>(start_offset);
        }

        const size_t available = read_available();
        if (start_offset + pattern_len > available) {
            return -1;
        }

        const size_t read_idx = read_pos_.load(std::memory_order_relaxed);

        for (size_t i = start_offset; i <= available - pattern_len; ++i) {
            bool match = true;
            for (size_t j = 0; j < pattern_len && match; ++j) {
                size_t pos = (read_idx + i + j) & mask_;
                if (buffer_[pos] != pattern[j]) {
                    match = false;
                }
            }
            if (match) {
                return static_cast<ssize_t>(i);
            }
        }

        return -1;
    }

    /**
     * @brief Search for "DLS\x01" magic sequence
     * @param start_offset Offset from read position to start search
     * @return Offset from read position where magic starts, or -1 if not found
     */
    ssize_t find_dls_magic(size_t start_offset = 0) const {
        static const uint8_t DLS_MAGIC[] = {'D', 'L', 'S', 0x01};
        return find(DLS_MAGIC, 4, start_offset);
    }

private:
    static size_t next_power_of_2(size_t n) {
        if (n == 0) return 1;
        n--;
        n |= n >> 1;
        n |= n >> 2;
        n |= n >> 4;
        n |= n >> 8;
        n |= n >> 16;
#if SIZE_MAX > 0xFFFFFFFF
        n |= n >> 32;
#endif
        return n + 1;
    }

    const size_t capacity_;
    const size_t mask_;  // For fast modulo (capacity - 1)
    std::unique_ptr<uint8_t[]> buffer_;

    // Separate cache lines for producer and consumer to avoid false sharing
    alignas(CACHE_LINE_SIZE) std::atomic<size_t> write_pos_;
    alignas(CACHE_LINE_SIZE) std::atomic<size_t> read_pos_;

    // Statistics (updated atomically)
    std::atomic<uint64_t> total_written_;
    std::atomic<uint64_t> total_read_;
    std::atomic<uint64_t> overflow_count_;
    std::atomic<size_t> high_water_mark_;
};

} // namespace dls

#endif // DLS_RING_BUFFER_HPP
