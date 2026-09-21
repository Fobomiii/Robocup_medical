#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>

/**
 * GY614 / GY615V3 红外测温模块（UART）
 *
 * 用法:
 *   GY614 gy;
 *   gy.begin(rxPin, txPin);   // setup
 *   gy.update();              // loop 里周期性调用
 *   float t = gy.bo();        // 体温 °C
 */
class GY614 {
public:
  explicit GY614(HardwareSerial& serial = Serial1);

  // rxPin: ESP32 收脚(接模块TX)  txPin: ESP32 发脚(接模块RX)
  bool begin(int rxPin, int txPin, uint32_t baud = 9600, uint8_t addr = 0xA4);

  // 收数据 + 定时查询，loop 里调用
  void update(uint32_t queryIntervalMs = 500);

  bool ready() const { return _ready; }

  float e() const { return _e; }    // 发射率
  float to() const { return _to; }  // 目标温度
  float ta() const { return _ta; }  // 环境温度
  float bo() const { return _bo; }  // 体温

private:
  void requestTemp();
  void parseByte(uint8_t b);

  HardwareSerial& _serial;
  uint8_t _addr;
  bool _ready;

  float _e, _to, _ta, _bo;

  uint8_t _buf[32];
  uint8_t _cnt;
  uint8_t _startReg;
  uint8_t _dataLen;
  bool _frameReady;
  uint32_t _lastQueryMs;
};
