/*
 commbuffer.cpp
  Created: Jun 10, 2025
  Author: Anton Sergunov

Copyright (c) 2025 Anton Sergunov

Based on https://github.com/collin80/ESP32RET
Copyright (c) 2014-2020 Collin Kidder, Michael Neuweiler

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "commbuffer.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <string>

namespace esphome {
namespace canbus_gvret {

static const char *const TAG = "canbus_gvret.commbuffer";

// a bit faster version that blasts through the copy more efficiently.
void CommBuffer::sendBytesToBuffer(uint8_t *bytes, size_t length) {
  transmitBuffer.insert(transmitBuffer.end(), bytes, bytes + length);
}

void CommBuffer::sendByteToBuffer(uint8_t byt) {
  transmitBuffer.push_back(byt);
}

void CommBuffer::sendString(const std::string &str) {
  transmitBuffer.insert(transmitBuffer.end(), str.begin(), str.end());
  ESP_LOGV(TAG, "Queued %zu bytes", str.size());
}

void CommBuffer::sendCharString(const char *str) {
  const size_t len = strlen(str);
  transmitBuffer.insert(transmitBuffer.end(), str, str + len);
  ESP_LOGV(TAG, "Queued %zu bytes", len);
}

void CommBuffer::sendFrameToBuffer(const CAN_FRAME &frame, int whichBus) {
  if (this->useBinarySerialComm) {
    uint32_t can_id = frame.can_id;
    if (frame.use_extended_id)
      can_id |= 1 << 31;
    transmitBuffer.push_back(0xF1);
    transmitBuffer.push_back(0); // 0 = canbus frame sending
    uint32_t now = micros();
    transmitBuffer.push_back32(now);
    transmitBuffer.push_back32(can_id);
    transmitBuffer.push_back(frame.can_data_length_code +
                             (uint8_t)(whichBus << 4));
    transmitBuffer.insert(transmitBuffer.end(), frame.data,
                          frame.data + frame.can_data_length_code);
    // temp = checksumCalc(buff, 11 + frame.length);
    transmitBuffer.push_back(0);
    // Serial.write(buff, 12 + frame.length);
  } else {
    transmitBuffer.printf("%d - %x", micros(), frame.can_id);
    if (frame.use_extended_id)
      transmitBuffer.printf(" X ");
    else
      transmitBuffer.printf(" S ");
    transmitBuffer.printf("%i %i", whichBus, frame.can_data_length_code);
    for (int c = 0; c < frame.can_data_length_code; c++) {
      transmitBuffer.printf(" %x", frame.data[c]);
    }
    transmitBuffer.printf("\r\n");
  }
}
#ifdef USE_GVRET_CANFD

void CommBuffer::sendFrameToBuffer(const CAN_FRAME_FD &frame, int whichBus) {
  if (settings.useBinarySerialComm) {
    uint32_t id = frame.id;
    if (frame.extended)
      id |= 1 << 31;
    transmitBuffer.push_back(0xF1);
    transmitBuffer.push_back(PROTO_BUILD_FD_FRAME);
    uint32_t now = micros();
    transmitBuffer.push_back32(now);
    transmitBuffer.push_back32(id);
    transmitBuffer.push_back(frame.length);
    transmitBuffer.push_back((uint8_t)(whichBus));
    for (int c = 0; c < frame.length; c++) {
      transmitBuffer.push_back(frame.data.uint8[c]);
    }
    // temp = checksumCalc(buff, 11 + frame.length);
    transmitBuffer.push_back(0);
    // Serial.write(buff, 12 + frame.length);
  } else {
    transmitBuffer.printf("%d - %x", micros(), frame.id);
    if (frame.extended)
      transmitBuffer.printf(" X ");
    else
      transmitBuffer.printf(" S ");
    transmitBuffer.printf("%i %i", whichBus, frame.length);
    for (int c = 0; c < frame.length; c++) {
      transmitBuffer.printf(" %x", frame.data.uint8[c]);
    }
    transmitBuffer.printf("\r\n");
  }
}
#endif
} // namespace canbus_gvret
} // namespace esphome
