/**
 * @file main.cpp
 * @brief DLS daemon entry point
 */

#include "common.hpp"
#include "config.hpp"
#include "ring_buffer.hpp"
#include "stats.hpp"
#include "serial_reader.hpp"
#include "dlt_parser.hpp"
#include "packet_dispatcher.hpp"
#include "data_server.hpp"
#include "control_server.hpp"
#include "binary_logger.hpp"

#include <iostream>
#include <fstream>
#include <csignal>
#include <atomic>
#include <thread>

#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

namespace {

std::atomic<bool> g_running{true};

void signal_handler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) {
        std::cout << "\nShutting down...\n";
        g_running.store(false, std::memory_order_relaxed);
    }
}

void setup_signal_handlers() {
    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;

    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    // Ignore SIGPIPE (broken pipe from clients)
    signal(SIGPIPE, SIG_IGN);
}

bool daemonize() {
    pid_t pid = fork();
    if (pid < 0) {
        std::cerr << "Error: fork failed\n";
        return false;
    }
    if (pid > 0) {
        // Parent exits
        _exit(0);
    }

    // Child continues
    if (setsid() < 0) {
        std::cerr << "Error: setsid failed\n";
        return false;
    }

    // Fork again to prevent terminal acquisition
    pid = fork();
    if (pid < 0) {
        return false;
    }
    if (pid > 0) {
        _exit(0);
    }

    // Change to root directory
    if (chdir("/") < 0) {
        // Non-fatal
    }

    // Close standard file descriptors
    close(STDIN_FILENO);
    close(STDOUT_FILENO);
    close(STDERR_FILENO);

    // Redirect to /dev/null
    open("/dev/null", O_RDONLY);  // stdin
    open("/dev/null", O_WRONLY);  // stdout
    open("/dev/null", O_WRONLY);  // stderr

    return true;
}

void write_pid_file(const std::string& path) {
    std::ofstream f(path);
    if (f.is_open()) {
        f << getpid() << "\n";
    }
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    using namespace dls;

    // Parse configuration
    Config config;
    if (!config.parse_args(argc, argv)) {
        Config::print_usage(argv[0]);
        return 1;
    }

    if (config.show_help) {
        Config::print_usage(argv[0]);
        return 0;
    }

    if (config.show_version) {
        Config::print_version();
        return 0;
    }

    // Daemonize if requested
    if (config.daemonize) {
        if (!daemonize()) {
            return 1;
        }
    }

    // Write PID file
    if (!config.pid_file.empty()) {
        write_pid_file(config.pid_file);
    }

    // Setup signal handlers
    setup_signal_handlers();

    // Print startup banner
    if (!config.daemonize) {
        std::cout << "========================================\n";
        std::cout << DLS_DAEMON_NAME << " v" << DLS_DAEMON_VERSION << "\n";
        std::cout << "========================================\n";
        std::cout << "Serial port:    " << config.serial_port << " @ "
                  << config.baud_rate << " baud\n";
        std::cout << "Data socket:    " << config.data_socket_path << "\n";
        if (config.data_tcp_port > 0) {
            std::cout << "Data TCP port:  " << config.data_tcp_port << "\n";
        }
        std::cout << "Control port:   " << config.control_port << "\n";
        std::cout << "Buffer size:    " << config.buffer_size_kb << " KB\n";
        std::cout << "FIBEX messages: " << config.message_count() << "\n";
        std::cout << "========================================\n\n";
    }

    // Create components
    Stats stats;
    stats.set_gap_threshold(config.gap_threshold);
    RingBuffer ring_buffer(config.buffer_size_kb * 1024);
    SerialReader reader(ring_buffer, stats);
    DltParser parser(ring_buffer, stats);
    PacketDispatcher dispatcher(stats);
    BinaryLogger logger;

    DataServer data_server(dispatcher, config, stats);
    ControlServer control_server(stats, config, reader, logger, dispatcher);

    // Setup parser callback
    parser.set_packet_callback([&](DltPacket&& pkt) {
        // Enrich with FIBEX info
        if (auto* info = config.lookup_message(pkt.msg_id)) {
            pkt.category = info->category;
            pkt.name = info->name;
        }

        // Log if enabled
        if (logger.is_open()) {
            logger.log_packet(pkt);
            stats.add_logged_bytes(pkt.raw_data.size());
        }

        // Dispatch to clients
        dispatcher.dispatch(std::move(pkt));
    });

    // Open serial port or replay file
    if (!config.replay_file.empty()) {
        if (!reader.open_replay(config.replay_file, config.replay_speed)) {
            std::cerr << "Error: Failed to open replay file\n";
            return 1;
        }
        std::cout << "Replay mode: " << config.replay_file << "\n";
    } else {
        // Wait for device if requested (helps with boot timing)
        if (config.wait_for_device > 0) {
            int waited = 0;
            while (waited < config.wait_for_device) {
                if (access(config.serial_port.c_str(), F_OK) == 0) {
                    if (!config.daemonize) {
                        std::cout << "Device " << config.serial_port << " found after "
                                  << waited << "s\n";
                    }
                    break;
                }
                if (!config.daemonize && waited == 0) {
                    std::cout << "Waiting for " << config.serial_port << " (max "
                              << config.wait_for_device << "s)...\n";
                }
                std::this_thread::sleep_for(std::chrono::seconds(1));
                waited++;
            }
            if (access(config.serial_port.c_str(), F_OK) != 0) {
                std::cerr << "Error: Device " << config.serial_port
                          << " not found after " << config.wait_for_device << "s\n";
                return 1;
            }
        }

        if (!reader.open(config.serial_port, config.baud_rate)) {
            std::cerr << "Error: Failed to open serial port\n";
            return 1;
        }
    }

    // Enable initial logging if specified
    if (!config.log_file.empty()) {
        if (logger.open(config.log_file, config.serial_port, config.baud_rate)) {
            stats.set_logging_enabled(true, config.log_file);
            std::cout << "Logging to: " << config.log_file << "\n";
        }
    }

    // Start all components
    reader.start();
    parser.start();

    if (!data_server.start()) {
        std::cerr << "Error: Failed to start data server\n";
        return 1;
    }

    if (!control_server.start(config.control_port)) {
        std::cerr << "Error: Failed to start control server\n";
        return 1;
    }

    std::cout << "Daemon running. Press Ctrl+C to stop.\n\n";

    // Main loop - update stats periodically
    while (g_running.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        stats.update_throughput();
        stats.update_buffer_stats(ring_buffer);

        // Optional: print stats in verbose mode
        if (config.verbose && !config.daemonize) {
            std::cout << "\r[" << stats.uptime_seconds() << "s] "
                      << "Packets: " << stats.packets_total()
                      << " (" << static_cast<int>(stats.packets_per_second()) << "/s) "
                      << "Gaps: " << stats.mcnt_gaps_total()
                      << " Buffer: " << stats.buffer_used_kb() << "/" << stats.buffer_size_kb() << " KB"
                      << "     " << std::flush;
        }
    }

    std::cout << "\n";

    // Shutdown in reverse order
    control_server.stop();
    data_server.stop();
    parser.stop();
    reader.stop();
    logger.close();

    // Final stats
    std::cout << "\n========================================\n";
    std::cout << "Final Statistics:\n";
    std::cout << "  Uptime:           " << stats.uptime_seconds() << " seconds\n";
    std::cout << "  Bytes read:       " << stats.bytes_read() << "\n";
    std::cout << "  Packets parsed:   " << stats.packets_total() << "\n";
    std::cout << "  Parse errors:     " << stats.parse_errors() << "\n";
    std::cout << "  mcnt gaps:        " << stats.mcnt_gaps_total() << "\n";
    std::cout << "  Dropped estimate: " << stats.dropped_estimate() << "\n";
    std::cout << "  Buffer overflows: " << stats.buffer_overflows() << "\n";
    std::cout << "  Reconnects:       " << stats.reconnect_count() << "\n";
    if (stats.bytes_logged() > 0) {
        std::cout << "  Bytes logged:     " << stats.bytes_logged() << "\n";
    }
    std::cout << "========================================\n";

    // Remove PID file
    if (!config.pid_file.empty()) {
        unlink(config.pid_file.c_str());
    }

    return 0;
}
