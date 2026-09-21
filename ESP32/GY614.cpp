#include "GY614.h"

GY614::GY614(HardwareSerial& serial)
  : _serial(serial),
    _addr(0xA4),
    _ready(false),
    _e(0), _to(0), _ta(0), _bo(0),
    _cnt(0),
    _startReg(0),
    _dataLen(0),
    _frameReady(false),
    _lastQueryMs(0) {}

bool GY614::begin(int rxPin, int txPin, uint32_t baud, uint8_t addr) {
  _addr = addr;
  _ready = false;
  _cnt = 0;
  _frameReady = false;

  _serial.begin(baud, SERIAL_8N1, rxPin, txPin);
  delay(300);
  requestTemp();
  _lastQueryMs = millis();
  return true;
}

void GY614::requestTemp() {
  uint8_t cmd[5] = {_addr, 0x03, 0x07, 0x07, 0x00};
  uint8_t sum = 0;
  for (int i = 0; i < 4; i++) sum += cmd[i];
  cmd[4] = sum;
  _serial.write(cmd, 5);
}

void GY614::parseByte(uint8_t b) {
  _buf[_cnt] = b;

  switch (_cnt) {
    case 0:
      if (b != _addr) {
        _cnt = 0;
        return;
      }
      break;
    case 1:
      if (b != 0x03) {
        _cnt = 0;
        return;
      }
      break;
    case 2:
      if (b < 16) {
        _startReg = b;
      } else {
        _cnt = 0;
        return;
      }
      break;
    case 3:
      if ((_startReg + b) < 16) {
        _dataLen = b;
      } else {
        _cnt = 0;
        return;
      }
      break;
    default:
      if (_cnt == _dataLen + 4) {
        _frameReady = true;
      }
      break;
  }

  if (_frameReady) {
    _frameReady = false;
    uint8_t sum = 0;
    for (uint8_t i = 0; i < _cnt; i++) sum += _buf[i];
    uint8_t chk = _buf[_cnt];
    _cnt = 0;

    if (sum == chk && _startReg == 0x07 && _dataLen >= 7) {
      _e = _buf[4] / 100.0f;
      _to = (int16_t)((_buf[5] << 8) | _buf[6]) / 100.0f;
      _ta = (int16_t)((_buf[7] << 8) | _buf[8]) / 100.0f;
      _bo = (int16_t)((_buf[9] << 8) | _buf[10]) / 100.0f;
      _ready = true;
    }
    return;
  }

  _cnt++;
  if (_cnt >= sizeof(_buf)) _cnt = 0;
}

void GY614::update(uint32_t queryIntervalMs) {
  while (_serial.available()) {
    parseByte((uint8_t)_serial.read());
  }

  if (millis() - _lastQueryMs >= queryIntervalMs) {
    _lastQueryMs = millis();
    requestTemp();
  }
}
