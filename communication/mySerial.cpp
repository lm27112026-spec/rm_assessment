#include "mySerial.hpp"

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>
#include <cstring>
#endif

mySerial::mySerial(const std::string& port, int baud)
    : port_(port), baud_(baud) {
    // TODO: implement in Wave 2 Task 7
}

mySerial::~mySerial() {
    // TODO: implement in Wave 2 Task 7
}

bool mySerial::send(const uint8_t* data, size_t len) {
    (void)data; (void)len;
    return false;
}

bool mySerial::receive(uint8_t* buf, size_t len, int timeout_ms) {
    (void)buf; (void)len; (void)timeout_ms;
    return false;
}