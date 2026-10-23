/*
 * SO-101 servo bus bridge for the Waveshare ESP32 servo driver board.
 *
 * What it does: nothing clever. It is a transparent, full-speed pipe between the USB serial
 * port (to the PC running ROS 2) and the Feetech STS3215 servo bus. All protocol handling
 * (ping, sync read, sync write ...) happens on the PC in so101_controller.
 *
 * IMPORTANT - never print anything to Serial (USB) here. Any extra byte corrupts the stream.
 *
 * Board: Arduino IDE -> "ESP32 Dev Module" (esp32 core 2.x or 3.x).
 */
#include <Arduino.h>

// Servo bus UART pins of the Waveshare board.
// !!! VERIFY these against the wiki/schematic of YOUR board revision. !!!
// (They are GPIO18 = RX, GPIO19 = TX on the Waveshare "General Driver for Robots" /
//  "Servo Driver with ESP32" examples.)
#define S_RXD 18
#define S_TXD 19

#define BUS_BAUD 1000000   // ESP32 <-> servos (STS3215 factory default)
#define USB_BAUD 115200    // PC <-> ESP32; must match baud_rate in so101_ros2_control.xacro

static uint8_t buf[512];

void setup()
{
  // RX buffers must be sized BEFORE begin().
  Serial.setRxBufferSize(2048);
  Serial.begin(USB_BAUD);

  Serial1.setRxBufferSize(2048);
  Serial1.begin(BUS_BAUD, SERIAL_8N1, S_RXD, S_TXD);
}

void loop()
{
  // PC -> servos
  int n = Serial.available();
  if (n > 0) {
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    n = Serial.readBytes(buf, n);
    Serial1.write(buf, n);
  }

  // servos -> PC
  n = Serial1.available();
  if (n > 0) {
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    n = Serial1.readBytes(buf, n);
    Serial.write(buf, n);
  }
}
