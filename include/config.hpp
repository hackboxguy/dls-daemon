/**
 * @file config.hpp
 * @brief Configuration management and FIBEX message loading
 */

#ifndef DLS_CONFIG_HPP
#define DLS_CONFIG_HPP

#include "common.hpp"

#include <string>
#include <unordered_map>
#include <vector>
#include <nlohmann/json.hpp>

namespace dls {

/**
 * @brief Message information from FIBEX configuration
 */
struct MessageInfo {
    std::string category;       // Category (e.g., "VERSION", "BOOT", "POWER")
    std::string name;           // Message name (e.g., "APPA_SW_VERSION")
    std::string description;    // Human-readable description
};

/**
 * @brief Category information
 */
struct CategoryInfo {
    std::string color;          // Display color hint
    std::string priority;       // Priority level (high, medium, low)
};

/**
 * @brief Daemon configuration and FIBEX message database
 */
class Config {
public:
    Config();

    // ========================================================================
    // FIBEX Message Configuration
    // ========================================================================

    /**
     * @brief Load FIBEX message definitions from JSON file
     * @param json_path Path to JSON configuration file
     * @return true on success, false on error
     */
    bool load_fibex(const std::string& json_path);

    /**
     * @brief Load FIBEX from JSON string (for embedded defaults)
     * @param json_str JSON string
     * @return true on success
     */
    bool load_fibex_string(const std::string& json_str);

    /**
     * @brief Lookup message info by message ID
     * @param msg_id Message ID to lookup
     * @return Pointer to MessageInfo or nullptr if not found
     */
    const MessageInfo* lookup_message(uint32_t msg_id) const;

    /**
     * @brief Get all message IDs in a category
     * @param category Category name
     * @return Vector of message IDs
     */
    std::vector<uint32_t> get_category_ids(const std::string& category) const;

    /**
     * @brief Get list of all known categories
     */
    std::vector<std::string> get_categories() const;

    /**
     * @brief Get category info
     */
    const CategoryInfo* get_category_info(const std::string& category) const;

    /**
     * @brief Check if a message ID is known
     */
    bool has_message(uint32_t msg_id) const;

    /**
     * @brief Get total number of known messages
     */
    size_t message_count() const;

    // ========================================================================
    // Command Line Parsing
    // ========================================================================

    /**
     * @brief Parse command line arguments
     * @param argc Argument count
     * @param argv Argument values
     * @return true on success, false on error (prints usage)
     */
    bool parse_args(int argc, char* argv[]);

    /**
     * @brief Print usage information
     * @param program_name Program name (argv[0])
     */
    static void print_usage(const char* program_name);

    /**
     * @brief Print version information
     */
    static void print_version();

    // ========================================================================
    // Daemon Configuration (set by parse_args or directly)
    // ========================================================================

    // Serial port settings
    std::string serial_port = DEFAULT_SERIAL_PORT;
    int baud_rate = DEFAULT_BAUD_RATE;

    // Network settings
    std::string data_socket_path = DEFAULT_DATA_SOCKET;
    int data_tcp_port = 0;          // 0 = disabled
    int control_port = DEFAULT_CONTROL_PORT;

    // Buffer settings
    size_t buffer_size_kb = DEFAULT_BUFFER_SIZE_KB;

    // Logging
    std::string log_file;           // Binary packet log (empty = disabled)
    std::string activity_log;       // Daemon activity log (empty = stderr)
    bool verbose = false;

    // Daemon mode
    bool daemonize = false;
    std::string pid_file;

    // FIBEX config
    std::string fibex_file;

    // Replay mode
    std::string replay_file;        // Replay from binary log
    double replay_speed = 1.0;      // 0 = fast as possible

    // Gap tracking
    int gap_threshold = 1;          // Minimum gap size to report (0 = disable tracking)

    // Device wait (for boot timing issues)
    int wait_for_device = 0;        // Seconds to wait for device to appear (0 = no wait)

    // Flags
    bool show_help = false;
    bool show_version = false;

private:
    bool load_fibex_json(const nlohmann::json& j);
    void load_builtin_messages();

    // Message database
    std::unordered_map<uint32_t, MessageInfo> messages_;
    std::unordered_map<std::string, std::vector<uint32_t>> category_index_;
    std::unordered_map<std::string, CategoryInfo> categories_;
};

} // namespace dls

#endif // DLS_CONFIG_HPP
