#include "mySerial.hpp"

#include <cstdint>

#ifdef _WIN32
#include <windows.h>

namespace
{
bool set_timeouts(HANDLE handle, DWORD timeout_ms)
{
  COMMTIMEOUTS timeouts = {};
  timeouts.ReadIntervalTimeout = MAXDWORD;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  timeouts.ReadTotalTimeoutConstant = timeout_ms;
  timeouts.WriteTotalTimeoutMultiplier = 0;
  timeouts.WriteTotalTimeoutConstant = 0;
  return SetCommTimeouts(handle, &timeouts) != 0;
}
}  // namespace

mySerial::mySerial(const std::string & port, int baud)
  : handle_(nullptr), port_(port), baud_(baud), is_open_(false)
{
  std::string win_port = port_;
  if (win_port.rfind("\\.\\", 0) != 0 && win_port.rfind("\\\\?\\", 0) != 0) {
    win_port = "\\\\.\\" + win_port;
  }

  HANDLE handle = CreateFileA(
    win_port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (handle == INVALID_HANDLE_VALUE) {
    return;
  }

  DCB dcb = {};
  dcb.DCBlength = sizeof(DCB);
  if (GetCommState(handle, &dcb) == 0) {
    CloseHandle(handle);
    return;
  }

  dcb.BaudRate = static_cast<DWORD>(baud_);
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fBinary = TRUE;
  dcb.fDtrControl = DTR_CONTROL_DISABLE;
  dcb.fRtsControl = RTS_CONTROL_DISABLE;

  if (SetCommState(handle, &dcb) == 0 || !set_timeouts(handle, 0)) {
    CloseHandle(handle);
    return;
  }

  handle_ = handle;
  is_open_ = true;
}

mySerial::~mySerial()
{
  if (handle_ != nullptr && handle_ != INVALID_HANDLE_VALUE) {
    CloseHandle(static_cast<HANDLE>(handle_));
  }
  handle_ = nullptr;
  is_open_ = false;
}

bool mySerial::send(const uint8_t * data, size_t len)
{
  if (!is_open_ || handle_ == nullptr || data == nullptr) {
    return false;
  }
  if (len > static_cast<size_t>(MAXDWORD)) {
    return false;
  }

  HANDLE handle = static_cast<HANDLE>(handle_);
  size_t total_written = 0;
  while (total_written < len) {
    DWORD bytes_written = 0;
    const DWORD chunk = static_cast<DWORD>(len - total_written);
    if (WriteFile(handle, data + total_written, chunk, &bytes_written, nullptr) == 0 || bytes_written == 0) {
      return false;
    }
    total_written += bytes_written;
  }
  return true;
}

bool mySerial::receive(uint8_t * buf, size_t len, int timeout_ms)
{
  if (!is_open_ || handle_ == nullptr || buf == nullptr) {
    return false;
  }
  if (len > static_cast<size_t>(MAXDWORD)) {
    return false;
  }

  HANDLE handle = static_cast<HANDLE>(handle_);
  const DWORD timeout = timeout_ms < 0 ? 0U : static_cast<DWORD>(timeout_ms);
  if (!set_timeouts(handle, timeout)) {
    return false;
  }

  size_t total_read = 0;
  while (total_read < len) {
    DWORD bytes_read = 0;
    const DWORD chunk = static_cast<DWORD>(len - total_read);
    if (ReadFile(handle, buf + total_read, chunk, &bytes_read, nullptr) == 0) {
      (void)set_timeouts(handle, 0);
      return false;
    }
    if (bytes_read == 0) {
      (void)set_timeouts(handle, 0);
      return false;
    }
    total_read += bytes_read;
  }

  (void)set_timeouts(handle, 0);
  return true;
}
#else
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace
{
speed_t baud_to_speed(int baud)
{
  switch (baud) {
  case 9600:
    return B9600;
  case 19200:
    return B19200;
  case 38400:
    return B38400;
  case 57600:
    return B57600;
  case 115200:
  default:
    return B115200;
  }
}
}  // namespace

mySerial::mySerial(const std::string & port, int baud)
  : handle_(nullptr), port_(port), baud_(baud), is_open_(false)
{
  const int fd = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    return;
  }

  termios tty = {};
  if (tcgetattr(fd, &tty) != 0) {
    close(fd);
    return;
  }

  const speed_t speed = baud_to_speed(baud_);
  (void)cfsetispeed(&tty, speed);
  (void)cfsetospeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= (CREAD | CLOCAL);

  tty.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO | ECHOE | ECHONL | ISIG));
  tty.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY | ICRNL | INLCR));
  tty.c_oflag &= static_cast<tcflag_t>(~OPOST);

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    close(fd);
    return;
  }

  handle_ = reinterpret_cast<void *>(static_cast<intptr_t>(fd));
  is_open_ = true;
}

mySerial::~mySerial()
{
  if (handle_ != nullptr) {
    const int fd = static_cast<int>(reinterpret_cast<intptr_t>(handle_));
    (void)close(fd);
  }
  handle_ = nullptr;
  is_open_ = false;
}

bool mySerial::send(const uint8_t * data, size_t len)
{
  if (!is_open_ || handle_ == nullptr || data == nullptr) {
    return false;
  }

  const int fd = static_cast<int>(reinterpret_cast<intptr_t>(handle_));
  size_t total_written = 0;
  while (total_written < len) {
    const ssize_t written = write(fd, data + total_written, len - total_written);
    if (written < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    if (written == 0) {
      return false;
    }
    total_written += static_cast<size_t>(written);
  }
  return true;
}

bool mySerial::receive(uint8_t * buf, size_t len, int timeout_ms)
{
  if (!is_open_ || handle_ == nullptr || buf == nullptr) {
    return false;
  }

  const int fd = static_cast<int>(reinterpret_cast<intptr_t>(handle_));
  size_t total_read = 0;
  while (total_read < len) {
    pollfd pfd = {};
    pfd.fd = fd;
    pfd.events = POLLIN;

    const int ret = poll(&pfd, 1, timeout_ms < 0 ? 0 : timeout_ms);
    if (ret <= 0) {
      return false;
    }

    const ssize_t n = read(fd, buf + total_read, len - total_read);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    if (n == 0) {
      return false;
    }
    total_read += static_cast<size_t>(n);
  }

  return true;
}
#endif
