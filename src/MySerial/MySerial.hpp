#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

class MySerial {
public:
  MySerial(const std::string & port, int baud);
  ~MySerial();

  MySerial(const MySerial &) = delete;
  MySerial & operator=(const MySerial &) = delete;

  bool isOpen() const;
  bool send(const uint8_t * data, size_t len);
  bool receive(uint8_t * buf, size_t len, int timeout_ms);

private:
  void * handle_ = nullptr;  // Win32 HANDLE / Linux fd
  std::string port_;
  int baud_ = 0;
  bool is_open_ = false;
};
