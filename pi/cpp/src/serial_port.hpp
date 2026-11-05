// Minimal termios serial port for the tag (J-Link VCOM, 115200 8N1 raw).
#pragma once
#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <string>

namespace uwb {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort() { close(); }
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& path, int baud = 115200);
    void close();
    bool is_open() const { return fd_ >= 0; }

    // Waits up to timeout_ms for data. Returns the byte count (0 on timeout) or -1 if the port failed
    // (unplugged, hang-up), after which the caller should close() and reopen.
    ssize_t read(uint8_t* buf, size_t len, int timeout_ms);
    bool write(const uint8_t* buf, size_t len);

private:
    int fd_ = -1;
};

}  // namespace uwb
