/**
 * @file dls_ctl.cpp
 * @brief DLS daemon control client
 *
 * Usage:
 *   dls-ctl [OPTIONS] COMMAND [ARGS]
 *
 * Commands:
 *   status          Show daemon status
 *   stats           Show detailed statistics
 *   start           Start capture
 *   stop            Stop capture
 *   flush           Flush buffers
 *   log enable FILE Enable binary logging
 *   log disable     Disable binary logging
 *   config get KEY  Get configuration value
 *   config set K V  Set configuration value
 *   clients         List connected data clients
 *   gaps [COUNT]    Show recent mcnt gaps
 */

#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <cstdint>
#include <getopt.h>

#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include <nlohmann/json.hpp>

namespace {

constexpr const char* DEFAULT_HOST = "localhost";
constexpr int DEFAULT_PORT = 3491;

struct Options {
    std::string host = DEFAULT_HOST;
    int port = DEFAULT_PORT;
    bool json_output = false;
    std::string command;
    std::vector<std::string> args;
};

void print_usage(const char* program) {
    std::cout << "Usage: " << program << " [OPTIONS] COMMAND [ARGS]\n\n"
        "Control client for dls-daemon\n\n"
        "Options:\n"
        "  -H, --host HOST     Daemon host (default: localhost)\n"
        "  -P, --port PORT     Control port (default: 3491)\n"
        "  -j, --json          Output raw JSON\n"
        "  -h, --help          Show this help\n"
        "\n"
        "Commands:\n"
        "  status              Show daemon status\n"
        "  stats               Show detailed statistics\n"
        "  start               Start capture\n"
        "  stop                Stop capture\n"
        "  flush               Flush all client buffers\n"
        "  log enable FILE     Enable binary logging to FILE\n"
        "  log disable         Disable binary logging\n"
        "  config get KEY      Get configuration value\n"
        "  config set KEY VAL  Set configuration value\n"
        "  clients             List connected data clients\n"
        "  gaps [COUNT]        Show recent mcnt gaps (default: 10)\n"
        "\n"
        "Examples:\n"
        "  " << program << " status\n"
        "  " << program << " stats\n"
        "  " << program << " log enable /tmp/capture.bin\n"
        "  " << program << " -H 192.168.1.100 stats\n"
        "  " << program << " -j stats\n"
        "\n";
}

bool parse_args(int argc, char* argv[], Options& opts) {
    static struct option long_options[] = {
        {"host",  required_argument, nullptr, 'H'},
        {"port",  required_argument, nullptr, 'P'},
        {"json",  no_argument,       nullptr, 'j'},
        {"help",  no_argument,       nullptr, 'h'},
        {nullptr, 0,                 nullptr, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "H:P:jh", long_options, nullptr)) != -1) {
        switch (opt) {
            case 'H':
                opts.host = optarg;
                break;
            case 'P':
                opts.port = std::stoi(optarg);
                break;
            case 'j':
                opts.json_output = true;
                break;
            case 'h':
                print_usage(argv[0]);
                exit(0);
            default:
                return false;
        }
    }

    // Remaining arguments are command and args
    if (optind >= argc) {
        std::cerr << "Error: No command specified\n";
        return false;
    }

    opts.command = argv[optind++];
    while (optind < argc) {
        opts.args.push_back(argv[optind++]);
    }

    return true;
}

int connect_to_daemon(const std::string& host, int port) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        std::cerr << "Error: Cannot create socket\n";
        return -1;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));

    if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) <= 0) {
        // Try as hostname - resolve localhost
        if (host == "localhost") {
            addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        } else {
            std::cerr << "Error: Invalid address: " << host << "\n";
            close(sock);
            return -1;
        }
    }

    if (connect(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "Error: Cannot connect to " << host << ":" << port
                  << " - " << strerror(errno) << "\n";
        close(sock);
        return -1;
    }

    // Set TCP_NODELAY
    int opt = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

    return sock;
}

bool send_json(int sock, const nlohmann::json& j) {
    std::string data = j.dump() + "\n";
    ssize_t total = 0;
    ssize_t len = static_cast<ssize_t>(data.size());

    while (total < len) {
        ssize_t n = write(sock, data.c_str() + total, len - total);
        if (n <= 0) return false;
        total += n;
    }
    return true;
}

bool recv_json(int sock, nlohmann::json& j) {
    std::string line;
    char c;

    while (true) {
        ssize_t n = read(sock, &c, 1);
        if (n <= 0) return false;
        if (c == '\n') break;
        line += c;
        if (line.size() > 65536) return false;
    }

    try {
        j = nlohmann::json::parse(line);
        return true;
    } catch (...) {
        return false;
    }
}

nlohmann::json build_request(const Options& opts) {
    nlohmann::json req;

    if (opts.command == "status" || opts.command == "stats") {
        req["cmd"] = opts.command;
    } else if (opts.command == "start" || opts.command == "stop" || opts.command == "flush") {
        req["cmd"] = opts.command;
    } else if (opts.command == "log") {
        req["cmd"] = "log";
        if (!opts.args.empty() && opts.args[0] == "enable") {
            req["enable"] = true;
            if (opts.args.size() > 1) {
                req["file"] = opts.args[1];
            }
        } else {
            req["enable"] = false;
        }
    } else if (opts.command == "config") {
        req["cmd"] = "config";
        if (!opts.args.empty()) {
            if (opts.args[0] == "get" && opts.args.size() > 1) {
                req["get"] = opts.args[1];
            } else if (opts.args[0] == "set" && opts.args.size() > 2) {
                req["set"] = opts.args[1];
                // Try to parse as number or boolean
                try {
                    req["value"] = std::stoi(opts.args[2]);
                } catch (...) {
                    if (opts.args[2] == "true") {
                        req["value"] = true;
                    } else if (opts.args[2] == "false") {
                        req["value"] = false;
                    } else {
                        req["value"] = opts.args[2];
                    }
                }
            }
        }
    } else if (opts.command == "clients") {
        req["cmd"] = "clients";
    } else if (opts.command == "gaps") {
        req["cmd"] = "gaps";
        if (!opts.args.empty()) {
            req["count"] = std::stoi(opts.args[0]);
        }
    } else {
        req["cmd"] = opts.command;
    }

    return req;
}

void print_status(const nlohmann::json& resp) {
    std::cout << "DLS Daemon Status\n";
    std::cout << "=================\n";
    std::cout << "Version:          " << resp.value("version", "unknown") << "\n";
    std::cout << "Uptime:           " << resp.value("uptime_sec", 0) << " seconds\n";
    std::cout << "Serial port:      " << resp.value("serial_port", "?") << "\n";
    std::cout << "Baud rate:        " << resp.value("baud_rate", 0) << "\n";
    std::cout << "Serial connected: " << (resp.value("serial_connected", false) ? "yes" : "no") << "\n";
    std::cout << "Packets total:    " << resp.value("packets_total", 0) << "\n";
    std::cout << "Data clients:     " << resp.value("data_clients", 0) << "\n";
    std::cout << "Logging enabled:  " << (resp.value("logging_enabled", false) ? "yes" : "no") << "\n";
}

void print_stats(const nlohmann::json& resp) {
    std::cout << "DLS Daemon Statistics\n";
    std::cout << "=====================\n\n";

    std::cout << "Uptime: " << resp.value("uptime_sec", 0) << " seconds\n\n";

    if (resp.contains("serial")) {
        auto& s = resp["serial"];
        std::cout << "Serial:\n";
        std::cout << "  Port:       " << s.value("port", "?") << "\n";
        std::cout << "  Baud:       " << s.value("baud", 0) << "\n";
        std::cout << "  State:      " << s.value("state", "?") << "\n";
        std::cout << "  Bytes read: " << s.value("bytes_read", 0) << "\n";
        std::cout << "  Reconnects: " << s.value("reconnects", 0) << "\n\n";
    }

    if (resp.contains("packets")) {
        auto& p = resp["packets"];
        std::cout << "Packets:\n";
        std::cout << "  Total:           " << p.value("total", 0) << "\n";
        std::cout << "  Per second:      " << std::fixed << std::setprecision(1)
                  << p.value("per_second", 0.0) << "\n";
        std::cout << "  Parse errors:    " << p.value("parse_errors", 0) << "\n";
        std::cout << "  mcnt gaps:       " << p.value("mcnt_gaps", 0) << "\n";
        std::cout << "  Dropped est.:    " << p.value("dropped_estimate", 0) << "\n\n";
    }

    if (resp.contains("ring_buffer")) {
        auto& b = resp["ring_buffer"];
        std::cout << "Ring Buffer:\n";
        std::cout << "  Size:       " << b.value("size_kb", 0) << " KB\n";
        std::cout << "  Used:       " << b.value("used_kb", 0) << " KB\n";
        std::cout << "  High water: " << b.value("high_water_kb", 0) << " KB\n";
        std::cout << "  Overflows:  " << b.value("overflows", 0) << "\n\n";
    }

    if (resp.contains("clients")) {
        auto& c = resp["clients"];
        std::cout << "Clients:\n";
        std::cout << "  Data:    " << c.value("data_connected", 0) << "\n";
        std::cout << "  Control: " << c.value("control_connected", 0) << "\n\n";
    }

    if (resp.contains("logging")) {
        auto& l = resp["logging"];
        std::cout << "Logging:\n";
        std::cout << "  Enabled: " << (l.value("enabled", false) ? "yes" : "no") << "\n";
        if (l.value("enabled", false)) {
            std::cout << "  File:    " << l.value("file", "") << "\n";
            std::cout << "  Written: " << l.value("bytes_written", 0) << " bytes\n";
        }
        std::cout << "\n";
    }

    if (resp.contains("recent_gaps") && !resp["recent_gaps"].empty()) {
        std::cout << "Recent mcnt gaps:\n";
        for (const auto& g : resp["recent_gaps"]) {
            std::cout << "  Expected " << g.value("expected", 0)
                      << ", got " << g.value("actual", 0)
                      << " (gap: " << g.value("gap", 0) << ")\n";
        }
    }
}

void print_clients(const nlohmann::json& resp) {
    if (!resp.contains("clients") || resp["clients"].empty()) {
        std::cout << "No data clients connected\n";
        return;
    }

    std::cout << "Connected Data Clients\n";
    std::cout << "======================\n";

    for (const auto& c : resp["clients"]) {
        std::cout << "\nClient " << c.value("id", 0) << ":\n";
        std::cout << "  Queue depth:  " << c.value("queue_depth", 0)
                  << " / " << c.value("max_queue_size", 0) << "\n";
        std::cout << "  Received:     " << c.value("received", 0) << "\n";
        std::cout << "  Dropped:      " << c.value("dropped", 0) << "\n";

        if (c.contains("filter")) {
            auto& f = c["filter"];
            if (f.value("all_packets", true)) {
                std::cout << "  Filter:       all packets\n";
            } else {
                if (f.contains("categories") && !f["categories"].empty()) {
                    std::cout << "  Categories:   ";
                    for (const auto& cat : f["categories"]) {
                        std::cout << cat.get<std::string>() << " ";
                    }
                    std::cout << "\n";
                }
                if (f.contains("msg_ids") && !f["msg_ids"].empty()) {
                    std::cout << "  Message IDs:  ";
                    for (const auto& id : f["msg_ids"]) {
                        std::cout << id.get<int>() << " ";
                    }
                    std::cout << "\n";
                }
            }
        }
    }
}

void print_gaps(const nlohmann::json& resp) {
    std::cout << "mcnt Gap Analysis\n";
    std::cout << "=================\n";
    std::cout << "Total gaps:       " << resp.value("total_gaps", 0) << "\n";
    std::cout << "Dropped estimate: " << resp.value("dropped_estimate", 0) << "\n\n";

    if (!resp.contains("gaps") || resp["gaps"].empty()) {
        std::cout << "No recent gaps recorded\n";
        return;
    }

    std::cout << "Recent gaps:\n";
    std::cout << "  Timestamp (us)       Expected  Actual  Gap\n";
    std::cout << "  -------------------------------------------\n";

    for (const auto& g : resp["gaps"]) {
        // Use uint64_t to avoid signed interpretation
        uint64_t ts = g.value("ts", static_cast<uint64_t>(0));
        std::cout << "  " << std::setw(18) << ts
                  << "  " << std::setw(8) << g.value("expected", 0)
                  << "  " << std::setw(6) << g.value("actual", 0)
                  << "  " << std::setw(4) << g.value("gap", 0) << "\n";
    }
}

void print_formatted(const Options& opts, const nlohmann::json& resp) {
    if (resp.value("status", "") != "ok") {
        std::cerr << "Error: " << resp.value("error", "Unknown error") << "\n";
        return;
    }

    if (opts.command == "status") {
        print_status(resp);
    } else if (opts.command == "stats") {
        print_stats(resp);
    } else if (opts.command == "clients") {
        print_clients(resp);
    } else if (opts.command == "gaps") {
        print_gaps(resp);
    } else if (opts.command == "start") {
        std::cout << "Capture started\n";
    } else if (opts.command == "stop") {
        std::cout << "Capture stopped\n";
    } else if (opts.command == "flush") {
        std::cout << "Flushed " << resp.value("clients_flushed", 0) << " client queues\n";
    } else if (opts.command == "log") {
        if (resp.contains("file")) {
            std::cout << "Logging enabled: " << resp["file"].get<std::string>() << "\n";
        } else {
            std::cout << "Logging disabled. Bytes logged: "
                      << resp.value("bytes_logged", 0) << "\n";
        }
    } else if (opts.command == "config") {
        if (resp.contains("value")) {
            std::cout << resp["value"] << "\n";
        } else {
            std::cout << "OK\n";
        }
    } else {
        std::cout << "OK\n";
    }
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    Options opts;
    if (!parse_args(argc, argv, opts)) {
        print_usage(argv[0]);
        return 1;
    }

    int sock = connect_to_daemon(opts.host, opts.port);
    if (sock < 0) {
        return 1;
    }

    nlohmann::json request = build_request(opts);
    if (!send_json(sock, request)) {
        std::cerr << "Error: Failed to send request\n";
        close(sock);
        return 1;
    }

    nlohmann::json response;
    if (!recv_json(sock, response)) {
        std::cerr << "Error: Failed to receive response\n";
        close(sock);
        return 1;
    }

    close(sock);

    if (opts.json_output) {
        std::cout << response.dump(2) << "\n";
    } else {
        print_formatted(opts, response);
    }

    return (response.value("status", "") == "ok") ? 0 : 1;
}
