// ByteTransport implementation on top of LibSerial.
#ifndef SO101_CONTROLLER__LIBSERIAL_TRANSPORT_HPP_
#define SO101_CONTROLLER__LIBSERIAL_TRANSPORT_HPP_

#include <libserial/SerialPort.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "so101_controller/feetech_bus.hpp"

namespace so101_controller
{

class LibSerialTransport : public ByteTransport
{
public:
  static LibSerial::BaudRate toBaud(int baud)
  {
    switch (baud) {
      case 115200: return LibSerial::BaudRate::BAUD_115200;
      case 230400: return LibSerial::BaudRate::BAUD_230400;
      case 460800: return LibSerial::BaudRate::BAUD_460800;
      case 500000: return LibSerial::BaudRate::BAUD_500000;
      case 576000: return LibSerial::BaudRate::BAUD_576000;
      case 921600: return LibSerial::BaudRate::BAUD_921600;
      case 1000000: return LibSerial::BaudRate::BAUD_1000000;
      case 1152000: return LibSerial::BaudRate::BAUD_1152000;
      case 1500000: return LibSerial::BaudRate::BAUD_1500000;
      case 2000000: return LibSerial::BaudRate::BAUD_2000000;
      default: throw std::invalid_argument("Unsupported baud rate " + std::to_string(baud));
    }
  }

  void open(const std::string & path, int baud)
  {
    port_.Open(path);
    port_.SetBaudRate(toBaud(baud));
    port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    port_.SetParity(LibSerial::Parity::PARITY_NONE);
    port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
  }

  bool isOpen() {return port_.IsOpen();}

  void close()
  {
    if (port_.IsOpen()) {port_.Close();}
  }

  void write(const uint8_t * data, size_t n) override
  {
    LibSerial::DataBuffer buf(data, data + n);
    port_.Write(buf);
    port_.DrainWriteBuffer();
  }

  size_t read(uint8_t * buf, size_t max, int timeout_ms) override
  {
    size_t n = 0;
    char c = 0;
    try {
      port_.ReadByte(c, static_cast<size_t>(timeout_ms > 0 ? timeout_ms : 1));
    } catch (const LibSerial::ReadTimeout &) {
      return 0;
    }
    buf[n++] = static_cast<uint8_t>(c);
    try {
      while (n < max && port_.IsDataAvailable()) {
        port_.ReadByte(c, 1);
        buf[n++] = static_cast<uint8_t>(c);
      }
    } catch (const LibSerial::ReadTimeout &) {
    }
    return n;
  }

  void flush_input() override {port_.FlushInputBuffer();}

private:
  LibSerial::SerialPort port_;
};

}  // namespace so101_controller

#endif  // SO101_CONTROLLER__LIBSERIAL_TRANSPORT_HPP_
