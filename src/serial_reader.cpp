/**
 * @file serial_reader.cpp
 * @brief Serial port reader implementation
 */

#include "serial_reader.hpp"
#include "common.hpp"

#include <iostream>
#include <cstring>
#include <cerrno>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sched.h>
#include <linux/serial.h>

namespace dls {

SerialReader::SerialReader(RingBuffer& buffer, Stats& stats)
    : buffer_(buffer)
    , stats_(stats)
{
}

SerialReader::~SerialReader() {
    stop();
    close();
}

// ============================================================================
// Serial Port Operations
// ============================================================================

bool SerialReader::open(const std::string& port, int baud_rate) {
    if (serial_fd_ >= 0) {
        close();
    }

    port_ = port;
    baud_rate_ = baud_rate;
    replay_mode_ = false;

    // Open serial port
    serial_fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        std::cerr << "Error: Cannot open serial port " << port
                  << ": " << strerror(errno) << "\n";
        return false;
    }

    // Configure serial port
    if (!configure_serial()) {
        ::close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    // Create epoll instance
    epoll_fd_ = epoll_create1(0);
    if (epoll_fd_ < 0) {
        std::cerr << "Error: epoll_create1 failed: " << strerror(errno) << "\n";
        ::close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    // Add serial fd to epoll
    struct epoll_event ev;
    ev.events = EPOLLIN;
    ev.data.fd = serial_fd_;
    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, serial_fd_, &ev) < 0) {
        std::cerr << "Error: epoll_ctl failed: " << strerror(errno) << "\n";
        ::close(epoll_fd_);
        ::close(serial_fd_);
        epoll_fd_ = -1;
        serial_fd_ = -1;
        return false;
    }

    connected_.store(true, std::memory_order_relaxed);
    stats_.set_serial_connected(true);

    if (connection_callback_) {
        connection_callback_(true);
    }

    return true;
}

bool SerialReader::configure_serial() {
    struct termios tty;

    if (tcgetattr(serial_fd_, &tty) != 0) {
        std::cerr << "Error: tcgetattr failed: " << strerror(errno) << "\n";
        return false;
    }

    // Set raw mode (no processing)
    cfmakeraw(&tty);

    // Set baud rate
    speed_t speed = speed_to_baud(baud_rate_);
    if (speed == B0) {
        std::cerr << "Error: Unsupported baud rate: " << baud_rate_ << "\n";
        return false;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // 8N1 (8 data bits, no parity, 1 stop bit)
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;

    // No hardware flow control
    tty.c_cflag &= ~CRTSCTS;

    // Enable receiver, ignore modem control lines
    tty.c_cflag |= (CLOCAL | CREAD);

    // Non-blocking mode settings
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    // Apply settings
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        std::cerr << "Error: tcsetattr failed: " << strerror(errno) << "\n";
        return false;
    }

    // Try to set larger kernel buffer (may fail on some systems)
    int buf_size = 65536;
    if (ioctl(serial_fd_, TIOCSWINSZ, &buf_size) < 0) {
        // Not critical, just use default buffer
    }

    // Set low_latency mode if available
    struct serial_struct serial_info;
    if (ioctl(serial_fd_, TIOCGSERIAL, &serial_info) == 0) {
        serial_info.flags |= ASYNC_LOW_LATENCY;
        ioctl(serial_fd_, TIOCSSERIAL, &serial_info);
    }

    // Flush any pending data
    tcflush(serial_fd_, TCIOFLUSH);

    return true;
}

int SerialReader::speed_to_baud(int baud) const {
    switch (baud) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 500000:  return B500000;
        case 576000:  return B576000;
        case 921600:  return B921600;
        case 1000000: return B1000000;
        case 1152000: return B1152000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        case 2500000: return B2500000;
        case 3000000: return B3000000;
        case 3500000: return B3500000;
        case 4000000: return B4000000;
        default:      return B0;
    }
}

void SerialReader::close() {
    if (epoll_fd_ >= 0) {
        ::close(epoll_fd_);
        epoll_fd_ = -1;
    }
    if (serial_fd_ >= 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
    }
    if (replay_fd_ >= 0) {
        ::close(replay_fd_);
        replay_fd_ = -1;
    }

    connected_.store(false, std::memory_order_relaxed);
    stats_.set_serial_connected(false);
}

bool SerialReader::is_open() const {
    return serial_fd_ >= 0 || replay_fd_ >= 0;
}

// ============================================================================
// Replay Mode
// ============================================================================

bool SerialReader::open_replay(const std::string& path, double speed_factor) {
    if (replay_fd_ >= 0 || serial_fd_ >= 0) {
        close();
    }

    replay_path_ = path;
    replay_speed_ = speed_factor;
    replay_mode_ = true;

    replay_fd_ = ::open(path.c_str(), O_RDONLY);
    if (replay_fd_ < 0) {
        std::cerr << "Error: Cannot open replay file " << path
                  << ": " << strerror(errno) << "\n";
        return false;
    }

    // TODO: Read and validate binary log header

    connected_.store(true, std::memory_order_relaxed);
    stats_.set_serial_connected(true);

    return true;
}

bool SerialReader::is_replay_mode() const {
    return replay_mode_;
}

// ============================================================================
// Thread Control
// ============================================================================

void SerialReader::start() {
    if (running_.load(std::memory_order_relaxed)) {
        return;
    }

    running_.store(true, std::memory_order_relaxed);

    if (replay_mode_) {
        reader_thread_ = std::thread(&SerialReader::replay_loop, this);
    } else {
        reader_thread_ = std::thread(&SerialReader::reader_loop, this);
    }
}

void SerialReader::stop() {
    running_.store(false, std::memory_order_relaxed);

    if (reader_thread_.joinable()) {
        reader_thread_.join();
    }
}

bool SerialReader::is_running() const {
    return running_.load(std::memory_order_relaxed);
}

// ============================================================================
// Reader Thread
// ============================================================================

bool SerialReader::set_realtime_priority() {
    struct sched_param param;
    param.sched_priority = 50;  // Mid-range RT priority

    if (sched_setscheduler(0, SCHED_FIFO, &param) < 0) {
        // May fail without root/CAP_SYS_NICE, not critical
        std::cerr << "Warning: Cannot set SCHED_FIFO priority: "
                  << strerror(errno) << " (continuing with normal priority)\n";
        return false;
    }
    return true;
}

void SerialReader::reader_loop() {
    // Try to set real-time priority (may fail without privileges)
    set_realtime_priority();

    uint8_t local_buf[READ_BUFFER_SIZE];
    struct epoll_event events[1];

    while (running_.load(std::memory_order_relaxed)) {
        if (!connected_.load(std::memory_order_relaxed)) {
            // Try to reconnect
            if (!try_reconnect()) {
                // Wait before retry
                int delay = std::min(100 * (1 << reconnect_attempts_),
                                     MAX_RECONNECT_DELAY_MS);
                std::this_thread::sleep_for(std::chrono::milliseconds(delay));
                reconnect_attempts_++;
                continue;
            }
            reconnect_attempts_ = 0;
        }

        // Wait for data with timeout
        int n = epoll_wait(epoll_fd_, events, 1, 100);

        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::cerr << "Error: epoll_wait failed: " << strerror(errno) << "\n";
            handle_disconnect();
            continue;
        }

        if (n == 0) {
            // Timeout, no data
            continue;
        }

        if (events[0].events & (EPOLLERR | EPOLLHUP)) {
            handle_disconnect();
            continue;
        }

        if (events[0].events & EPOLLIN) {
            ssize_t bytes = ::read(serial_fd_, local_buf, sizeof(local_buf));

            if (bytes > 0) {
                size_t written = buffer_.write(local_buf, static_cast<size_t>(bytes));
                stats_.add_bytes_read(static_cast<size_t>(bytes));

                if (written < static_cast<size_t>(bytes)) {
                    // Buffer overflow - data was dropped
                    // Already tracked by ring buffer overflow counter
                }
            } else if (bytes < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    std::cerr << "Error: read failed: " << strerror(errno) << "\n";
                    handle_disconnect();
                }
            } else {
                // bytes == 0, EOF on serial (shouldn't happen normally)
                handle_disconnect();
            }
        }
    }
}

void SerialReader::replay_loop() {
    // Replay binary log file
    // TODO: Implement proper binary log format parsing

    uint8_t local_buf[READ_BUFFER_SIZE];

    while (running_.load(std::memory_order_relaxed)) {
        ssize_t bytes = ::read(replay_fd_, local_buf, sizeof(local_buf));

        if (bytes > 0) {
            buffer_.write(local_buf, static_cast<size_t>(bytes));
            stats_.add_bytes_read(static_cast<size_t>(bytes));

            // Apply replay speed
            if (replay_speed_ > 0) {
                // Approximate delay based on baud rate
                // At 115200 baud, ~11520 bytes/sec
                double delay_ms = (bytes * 1000.0) / (11520.0 * replay_speed_);
                if (delay_ms > 0.1) {
                    std::this_thread::sleep_for(
                        std::chrono::microseconds(static_cast<int>(delay_ms * 1000)));
                }
            }
        } else if (bytes == 0) {
            // End of file - optionally loop or stop
            std::cerr << "Replay complete\n";
            running_.store(false, std::memory_order_relaxed);
        } else {
            std::cerr << "Error: replay read failed: " << strerror(errno) << "\n";
            running_.store(false, std::memory_order_relaxed);
        }
    }
}

void SerialReader::handle_disconnect() {
    connected_.store(false, std::memory_order_relaxed);
    stats_.set_serial_connected(false);
    stats_.increment_reconnects();

    if (epoll_fd_ >= 0) {
        ::close(epoll_fd_);
        epoll_fd_ = -1;
    }
    if (serial_fd_ >= 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
    }

    if (connection_callback_) {
        connection_callback_(false);
    }
}

bool SerialReader::try_reconnect() {
    if (port_.empty()) {
        return false;
    }

    // Try to reopen
    serial_fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        return false;
    }

    if (!configure_serial()) {
        ::close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    // Recreate epoll
    epoll_fd_ = epoll_create1(0);
    if (epoll_fd_ < 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    struct epoll_event ev;
    ev.events = EPOLLIN;
    ev.data.fd = serial_fd_;
    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, serial_fd_, &ev) < 0) {
        ::close(epoll_fd_);
        ::close(serial_fd_);
        epoll_fd_ = -1;
        serial_fd_ = -1;
        return false;
    }

    connected_.store(true, std::memory_order_relaxed);
    stats_.set_serial_connected(true);

    std::cerr << "Reconnected to " << port_ << "\n";

    if (connection_callback_) {
        connection_callback_(true);
    }

    return true;
}

void SerialReader::set_connection_callback(std::function<void(bool)> callback) {
    connection_callback_ = std::move(callback);
}

} // namespace dls
