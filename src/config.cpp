/**
 * @file config.cpp
 * @brief Configuration management implementation
 */

#include "config.hpp"

#include <fstream>
#include <iostream>
#include <getopt.h>
#include <cstdlib>

namespace dls {

Config::Config() {
    // Load built-in message definitions
    load_builtin_messages();
}

// ============================================================================
// FIBEX Message Configuration
// ============================================================================

bool Config::load_fibex(const std::string& json_path) {
    std::ifstream file(json_path);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open FIBEX config file: " << json_path << "\n";
        return false;
    }

    try {
        nlohmann::json j;
        file >> j;
        return load_fibex_json(j);
    } catch (const nlohmann::json::parse_error& e) {
        std::cerr << "Error: Invalid JSON in FIBEX config: " << e.what() << "\n";
        return false;
    }
}

bool Config::load_fibex_string(const std::string& json_str) {
    try {
        nlohmann::json j = nlohmann::json::parse(json_str);
        return load_fibex_json(j);
    } catch (const nlohmann::json::parse_error& e) {
        std::cerr << "Error: Invalid JSON string: " << e.what() << "\n";
        return false;
    }
}

bool Config::load_fibex_json(const nlohmann::json& j) {
    // Load messages
    if (j.contains("messages") && j["messages"].is_object()) {
        for (auto& item : j["messages"].items()) {
            const auto& key = item.key();
            const auto& value = item.value();
            try {
                uint32_t msg_id = std::stoul(key);
                MessageInfo info;

                if (value.contains("category")) {
                    info.category = value["category"].get<std::string>();
                }
                if (value.contains("name")) {
                    info.name = value["name"].get<std::string>();
                }
                if (value.contains("description")) {
                    info.description = value["description"].get<std::string>();
                }

                messages_[msg_id] = info;
                category_index_[info.category].push_back(msg_id);
            } catch (const std::exception& e) {
                std::cerr << "Warning: Failed to parse message ID '" << key
                          << "': " << e.what() << "\n";
            }
        }
    }

    // Load category info
    if (j.contains("categories") && j["categories"].is_object()) {
        for (auto& item : j["categories"].items()) {
            const auto& key = item.key();
            const auto& value = item.value();
            CategoryInfo info;
            if (value.contains("color")) {
                info.color = value["color"].get<std::string>();
            }
            if (value.contains("priority")) {
                info.priority = value["priority"].get<std::string>();
            }
            categories_[key] = info;
        }
    }

    return true;
}

void Config::load_builtin_messages() {
    // Built-in message definitions from dls_parser.py
    // These are loaded by default and can be overridden by JSON config

    // VERSION messages
    messages_[1201] = {"VERSION", "APPA_SW_VERSION", "AppA SW version"};
    messages_[1203] = {"VERSION", "APPB_SW_VERSION", "AppB SW version"};
    messages_[9330] = {"VERSION", "VARIANT_CODE", "Variant code"};
    messages_[20490] = {"VERSION", "VERSION_INFO_SW", "SW component version"};
    messages_[20493] = {"VERSION", "EXTERNAL_IPC_SW_VERSION", "External IPC SW version"};
    messages_[3427] = {"VERSION", "HSM_SW_VERSION", "HSM SW version"};
    messages_[41163] = {"VERSION", "VERSION_REPORT", "Version report"};

    // BOOT messages
    messages_[24008] = {"BOOT", "BOOT_IOC_SEND_BOOTINFO", "IOC boot info"};
    messages_[29015] = {"BOOT", "BOOTLOADER_INFO", "Bootloader identification"};
    messages_[3001] = {"BOOT", "SECURE_BOOT_SUCCESS", "Secure boot successful"};
    messages_[3002] = {"BOOT", "SECURE_BOOT_FAIL_BL", "Bootloader secure boot failed"};
    messages_[3003] = {"BOOT", "SECURE_BOOT_FAIL_APP", "Application secure boot failed"};
    messages_[20086] = {"BOOT", "CLUSTER_BOOT_SLOT_A", "Cluster boot slot A"};
    messages_[20087] = {"BOOT", "CLUSTER_BOOT_SLOT_B", "Cluster boot slot B"};
    messages_[21086] = {"BOOT", "SAFE_INIT", "Safe state initialized"};
    messages_[29016] = {"BOOT", "BOOT_STATE_BOOTING", "System booting"};
    messages_[9338] = {"BOOT", "EV_VOLTAGE_IN_RANGE", "Voltage within range"};
    messages_[9339] = {"BOOT", "EV_VEHICLE_POWER_ON", "Vehicle power on"};
    messages_[9384] = {"BOOT", "EV_STARTUP_COMPLETE", "Startup complete"};

    // POWER messages
    messages_[9008] = {"POWER", "POWERMODE_TO_SOC", "Power mode sent to SOC"};
    messages_[9031] = {"POWER", "WAKEUP_FACTOR", "Wakeup factor"};
    messages_[9040] = {"POWER", "WAKEUP_REASON", "Wakeup reason"};
    messages_[9143] = {"POWER", "IGNITION_STATUS", "Ignition status"};
    messages_[9144] = {"POWER", "IGNITION_TIMEOUT", "Ignition timeout"};
    messages_[9140] = {"POWER", "CRITICAL_TEMP_SHUTDOWN", "Critical temp shutdown"};

    // S2R messages
    messages_[20030] = {"S2R", "S2R_MAINTENANCE_TIMER", "S2R maintenance timer"};
    messages_[20031] = {"S2R", "S2R_MAINTENANCE_REBOOT", "S2R maintenance reboot"};

    // THERMAL messages
    messages_[25004] = {"THERMAL", "THERMAL_SOC_READ_FAIL", "SOC temp read failed"};
    messages_[25045] = {"THERMAL", "THERMAL_CRITICAL_DETECT", "Critical temp detected"};

    // VOLTAGE messages
    messages_[9006] = {"VOLTAGE", "VOLT_ABNORMAL", "Abnormal voltage"};
    messages_[9007] = {"VOLTAGE", "VOLT_RECOVERED", "Voltage recovered"};
    messages_[9331] = {"VOLTAGE", "BATT_VOLTAGE", "Battery voltage"};

    // SFI messages
    messages_[21001] = {"SFI", "SFI_SOC_HB_DISABLED", "SFI & SOC heartbeat disabled"};
    messages_[21004] = {"SFI", "SFI_HB_NEVER_RECV", "SFI heartbeat never received"};
    messages_[21005] = {"SFI", "SFI_HB_OK", "SFI heartbeat OK"};
    messages_[21006] = {"SFI", "SFI_HB_MISSING", "SFI heartbeat missing"};

    // SAFETY messages
    messages_[22237] = {"SAFETY", "MCAL_CORETEST_FAIL", "MCAL CoreTest failed"};
    messages_[22284] = {"SAFETY", "RAM_TEST_FAIL", "RAM test failed"};
    messages_[22320] = {"SAFETY", "WDT_SELF_FAIL", "Watchdog self-test failed"};

    // ERROR messages
    messages_[1191] = {"ERROR", "NVM_WRITE_ERROR", "NVM write error"};
    messages_[1694] = {"ERROR", "I2C_WRITE_ERROR", "I2C write error"};
    messages_[1780] = {"ERROR", "I2C_READ_ERROR", "I2C read error"};

    // STATS messages
    messages_[9166] = {"STATS", "BOOT_STATISTICS", "Boot statistics"};

    // Build category index
    category_index_.clear();
    for (const auto& kv : messages_) {
        category_index_[kv.second.category].push_back(kv.first);
    }

    // Default category info
    categories_["VERSION"] = {"green", "high"};
    categories_["BOOT"] = {"cyan", "high"};
    categories_["POWER"] = {"yellow", "medium"};
    categories_["S2R"] = {"yellow", "medium"};
    categories_["THERMAL"] = {"red", "high"};
    categories_["VOLTAGE"] = {"yellow", "medium"};
    categories_["SFI"] = {"magenta", "medium"};
    categories_["SAFETY"] = {"red", "high"};
    categories_["ERROR"] = {"red", "high"};
    categories_["STATS"] = {"blue", "low"};
}

const MessageInfo* Config::lookup_message(uint32_t msg_id) const {
    // Try direct lookup first
    auto it = messages_.find(msg_id);
    if (it != messages_.end()) {
        return &it->second;
    }

    // Try lower 16 bits (FIBEX message ID)
    uint32_t fibex_id = msg_id & 0xFFFF;
    it = messages_.find(fibex_id);
    if (it != messages_.end()) {
        return &it->second;
    }

    return nullptr;
}

std::vector<uint32_t> Config::get_category_ids(const std::string& category) const {
    auto it = category_index_.find(category);
    if (it != category_index_.end()) {
        return it->second;
    }
    return {};
}

std::vector<std::string> Config::get_categories() const {
    std::vector<std::string> result;
    result.reserve(category_index_.size());
    for (const auto& kv : category_index_) {
        result.push_back(kv.first);
    }
    return result;
}

const CategoryInfo* Config::get_category_info(const std::string& category) const {
    auto it = categories_.find(category);
    return (it != categories_.end()) ? &it->second : nullptr;
}

bool Config::has_message(uint32_t msg_id) const {
    return lookup_message(msg_id) != nullptr;
}

size_t Config::message_count() const {
    return messages_.size();
}

// ============================================================================
// Command Line Parsing
// ============================================================================

bool Config::parse_args(int argc, char* argv[]) {
    static struct option long_options[] = {
        // Serial port
        {"port",           required_argument, nullptr, 'p'},
        {"baud",           required_argument, nullptr, 'b'},

        // Network
        {"data-socket",    required_argument, nullptr, 's'},
        {"data-tcp-port",  required_argument, nullptr, 'T'},
        {"control-port",   required_argument, nullptr, 'C'},

        // Configuration
        {"config",         required_argument, nullptr, 'c'},
        {"buffer-size",    required_argument, nullptr, 'B'},

        // Logging
        {"log-file",       required_argument, nullptr, 'l'},
        {"activity-log",   required_argument, nullptr, 'a'},
        {"verbose",        no_argument,       nullptr, 'v'},

        // Daemon
        {"daemon",         no_argument,       nullptr, 'd'},
        {"pid-file",       required_argument, nullptr, 'P'},

        // Replay
        {"replay",         required_argument, nullptr, 'r'},
        {"replay-speed",   required_argument, nullptr, 'S'},

        // Gap tracking
        {"gap-threshold",  required_argument, nullptr, 'G'},

        // Help
        {"help",           no_argument,       nullptr, 'h'},
        {"version",        no_argument,       nullptr, 'V'},

        {nullptr,          0,                 nullptr, 0}
    };

    int opt;
    int option_index = 0;

    while ((opt = getopt_long(argc, argv, "p:b:s:T:C:c:B:l:a:vdP:r:S:G:hV",
                              long_options, &option_index)) != -1) {
        switch (opt) {
            case 'p':
                serial_port = optarg;
                break;
            case 'b':
                baud_rate = std::atoi(optarg);
                if (baud_rate <= 0) {
                    std::cerr << "Error: Invalid baud rate: " << optarg << "\n";
                    return false;
                }
                break;
            case 's':
                data_socket_path = optarg;
                break;
            case 'T':
                data_tcp_port = std::atoi(optarg);
                if (data_tcp_port <= 0 || data_tcp_port > 65535) {
                    std::cerr << "Error: Invalid TCP port: " << optarg << "\n";
                    return false;
                }
                break;
            case 'C':
                control_port = std::atoi(optarg);
                if (control_port <= 0 || control_port > 65535) {
                    std::cerr << "Error: Invalid control port: " << optarg << "\n";
                    return false;
                }
                break;
            case 'c':
                fibex_file = optarg;
                break;
            case 'B':
                buffer_size_kb = std::stoul(optarg);
                if (buffer_size_kb < 16) {
                    std::cerr << "Error: Buffer size must be at least 16 KB\n";
                    return false;
                }
                break;
            case 'l':
                log_file = optarg;
                break;
            case 'a':
                activity_log = optarg;
                break;
            case 'v':
                verbose = true;
                break;
            case 'd':
                daemonize = true;
                break;
            case 'P':
                pid_file = optarg;
                break;
            case 'r':
                replay_file = optarg;
                break;
            case 'S':
                replay_speed = std::stod(optarg);
                if (replay_speed < 0) {
                    std::cerr << "Error: Invalid replay speed: " << optarg << "\n";
                    return false;
                }
                break;
            case 'G':
                gap_threshold = std::atoi(optarg);
                if (gap_threshold < 0) {
                    std::cerr << "Error: Invalid gap threshold: " << optarg << "\n";
                    return false;
                }
                break;
            case 'h':
                show_help = true;
                return true;
            case 'V':
                show_version = true;
                return true;
            default:
                return false;
        }
    }

    // Load FIBEX config if specified
    if (!fibex_file.empty()) {
        if (!load_fibex(fibex_file)) {
            return false;
        }
    }

    return true;
}

void Config::print_usage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [OPTIONS]\n\n"
        "High-performance DLS/DLT packet capture daemon\n\n"
        "Serial Port:\n"
        "  -p, --port PATH         Serial port (default: " << DEFAULT_SERIAL_PORT << ")\n"
        "  -b, --baud RATE         Baud rate (default: " << DEFAULT_BAUD_RATE << ")\n"
        "\n"
        "Network:\n"
        "  -s, --data-socket PATH  Unix socket for data (default: " << DEFAULT_DATA_SOCKET << ")\n"
        "  -T, --data-tcp-port PORT  Enable TCP for data streaming (disabled by default)\n"
        "  -C, --control-port PORT TCP port for control (default: " << DEFAULT_CONTROL_PORT << ")\n"
        "\n"
        "Configuration:\n"
        "  -c, --config FILE       FIBEX message config JSON file\n"
        "  -B, --buffer-size KB    Ring buffer size in KB (default: " << DEFAULT_BUFFER_SIZE_KB << ")\n"
        "\n"
        "Logging:\n"
        "  -l, --log-file FILE     Enable binary packet logging to FILE\n"
        "  -a, --activity-log FILE Daemon activity log (default: stderr)\n"
        "  -v, --verbose           Verbose logging\n"
        "\n"
        "Daemon:\n"
        "  -d, --daemon            Run as background daemon\n"
        "  -P, --pid-file FILE     PID file path\n"
        "\n"
        "Replay:\n"
        "  -r, --replay FILE       Replay from binary log file\n"
        "  -S, --replay-speed N    Replay speed factor (default: 1.0, 0 = fast)\n"
        "\n"
        "Gap Tracking:\n"
        "  -G, --gap-threshold N   Min mcnt gap to report (default: 1, 0 = disable)\n"
        "\n"
        "Misc:\n"
        "  -h, --help              Show this help\n"
        "  -V, --version           Show version\n"
        "\n"
        "Examples:\n"
        "  " << program_name << " -p /dev/dls-trace\n"
        "  " << program_name << " -p /dev/dls-trace -c fibex.json -l capture.bin\n"
        "  " << program_name << " -p /dev/dls-trace -T 3490 -d\n"
        "  " << program_name << " --replay capture.bin --replay-speed 0\n"
        "\n";
}

void Config::print_version() {
    std::cout << DLS_DAEMON_NAME << " version " << DLS_DAEMON_VERSION << "\n"
        "High-performance DLS/DLT packet capture daemon\n";
}

} // namespace dls
