#include "serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>

namespace uwb {
namespace {
speed_t speed_for(int baud)
{
    switch (baud) {
        case 9600: return B9600;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: return B115200;
    }
}
}  // namespace

bool SerialPort::open(const std::string& path, int baud)
{
    close();
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;
    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
        close();
        return false;
    }
    cfmakeraw(&tio);
    cfsetispeed(&tio, speed_for(baud));
    cfsetospeed(&tio, speed_for(baud));
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        close();
        return false;
    }
    tcflush(fd_, TCIOFLUSH);
    return true;
}

void SerialPort::close()
{
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
}

ssize_t SerialPort::read(uint8_t* buf, size_t len, int timeout_ms)
{
    if (fd_ < 0) return -1;
    pollfd p{fd_, POLLIN, 0};
    const int r = ::poll(&p, 1, timeout_ms);
    if (r < 0) return errno == EINTR ? 0 : -1;
    if (r == 0) return 0;
    if (p.revents & (POLLERR | POLLNVAL)) return -1;
    const ssize_t n = ::read(fd_, buf, len);
    if (n < 0) return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
    if (n == 0) return (p.revents & POLLHUP) ? -1 : 0;
    return n;
}

bool SerialPort::write(const uint8_t* buf, size_t len)
{
    return fd_ >= 0 && ::write(fd_, buf, len) == static_cast<ssize_t>(len);
}

}  // namespace uwb
