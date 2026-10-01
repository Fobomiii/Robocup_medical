/*
  ESP32-S3 + SH1106 + UART blood-oxygen module + GY614 + BLE
  OLED : SCL=15 SDA=16
  Blood oxygen: TX -> GPIO42 (ESP32 RX), RX -> GPIO41 (ESP32 TX)
  GY614: TX -> GPIO6 (ESP32 RX), RX -> GPIO7 (ESP32 TX)
  Button: GPIO39 -> GND when pressed
  Active buzzer: GPIO38
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include "GY614.h"

#define OLED_SDA 16
#define OLED_SCL 15
#define OLED_ADDR 0x3C
#define GY614_RX_PIN 6
#define GY614_TX_PIN 7
#define SPO2_RX_PIN 42
#define SPO2_TX_PIN 41
#define BUTTON_PIN 39
#define BUZZER_PIN 38
#define BUZZER_ACTIVE_LEVEL HIGH

static const char* BLE_DEVICE_NAME = "MedicalVitals-S3";
static const char* BLE_SERVICE_UUID = "7d2e1000-5f5b-4f4b-9c61-7f1e9d5a0001";
static const char* BLE_CHARACTERISTIC_UUID = "7d2e1001-5f5b-4f4b-9c61-7f1e9d5a0001";
static const uint8_t HEART_RATE_VALID = 0x01;
static const uint8_t TEMPERATURE_VALID = 0x02;
static const uint8_t TEMPERATURE_SAMPLE_COUNT = 7;
static const uint32_t TEMPERATURE_QUERY_INTERVAL_MS = 250;
static const uint32_t TEMPERATURE_TIMEOUT_MS = 5000;
static const uint32_t TEMPERATURE_LONG_PRESS_MS = 800;
static const float TEMPERATURE_MIN_C = 25.0f;
static const float TEMPERATURE_MAX_C = 45.0f;
static const float TEMPERATURE_OUTLIER_C = 0.50f;
static const uint32_t SPO2_DATA_TIMEOUT_MS = 3000;
static const uint32_t SPO2_MODE_RETRY_MS = 3000;
static const float PPG_EMA_ALPHA = 0.35f;

Adafruit_SH1106G display(128, 64, &Wire, -1);
HardwareSerial temperatureSerial(1);
HardwareSerial spo2Serial(2);
GY614 gy614(temperatureSerial);
BLECharacteristic* healthCharacteristic = nullptr;
volatile bool bleConnected = false;
volatile bool bleRestartAdvertising = false;
uint16_t bleSequence = 0;

// ---------- 波形缓存（128列）----------
static const int WAVE_W = 128;
static const int WAVE_Y0 = 18;
static const int WAVE_H = 46;
int32_t waveBuf[WAVE_W];
int waveIdx = 0;
int waveCount = 0;
float ppgFiltered = 0.0f;
bool ppgFilterInitialized = false;

// ---------- 心率 ----------
enum PpgSource : uint8_t {
  PPG_SOURCE_NONE,
  PPG_SOURCE_S1,
  PPG_SOURCE_U2_SCALAR,
  PPG_SOURCE_U1,
};

PpgSource ppgSource = PPG_SOURCE_NONE;
char spo2Line[64];
size_t spo2LineLength = 0;
int bpm = 0;
bool bpmValid = false;
int oxygenPercent = 0;
bool oxygenValid = false;
int perfusionIndex = 0;

uint32_t lastUiMs = 0;
uint32_t lastBeatShowMs = 0;
uint32_t lastSpO2ResultMs = 0;
uint32_t lastPpgMs = 0;
uint32_t lastSpO2ModeRequestMs = 0;
uint32_t lastBleMs = 0;

// ---------- 按键触发体温测量 ----------
float temperatureSamples[TEMPERATURE_SAMPLE_COUNT];
float filteredTemperatureC = 0.0f;
uint8_t temperatureSampleCount = 0;
uint32_t temperatureStartedMs = 0;
uint32_t lastTemperatureSequence = 0;
bool temperatureMeasuring = false;
bool temperatureResultValid = false;
bool temperatureRealtime = false;
uint32_t lastTemperatureResultMs = 0;

bool buttonRawState = HIGH;
bool buttonStableState = HIGH;
uint32_t buttonChangedMs = 0;
uint32_t buttonPressedMs = 0;
bool buttonLongPressHandled = false;

enum BuzzerPhase : uint8_t {
  BUZZER_IDLE,
  BUZZER_ON,
  BUZZER_GAP,
};

BuzzerPhase buzzerPhase = BUZZER_IDLE;
uint8_t buzzerBeepsRemaining = 0;
uint32_t buzzerDeadlineMs = 0;
uint32_t buzzerOnDurationMs = 0;
uint32_t buzzerGapDurationMs = 0;

struct __attribute__((packed)) HealthPacket {
  uint8_t version;
  uint16_t sequence;
  uint16_t heartRateBpm;
  int16_t temperatureCenti;
  uint8_t flags;
  uint32_t uptimeMs;
  uint16_t crc;
};

static_assert(sizeof(HealthPacket) == 14, "HealthPacket layout changed");

class HealthServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    bleConnected = true;
  }

  void onDisconnect(BLEServer*) override {
    bleConnected = false;
    bleRestartAdvertising = true;
  }
};

uint16_t crc16Ccitt(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t index = 0; index < length; ++index) {
    crc ^= static_cast<uint16_t>(data[index]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

void startBle() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new HealthServerCallbacks());
  BLEService* service = server->createService(BLE_SERVICE_UUID);
  healthCharacteristic = service->createCharacteristic(
      BLE_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  healthCharacteristic->addDescriptor(new BLE2902());
  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
}

void startBuzzerPattern(
    uint8_t beepCount, uint32_t onDurationMs, uint32_t gapDurationMs) {
  if (beepCount == 0) {
    return;
  }
  buzzerBeepsRemaining = beepCount;
  buzzerOnDurationMs = onDurationMs;
  buzzerGapDurationMs = gapDurationMs;
  digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_LEVEL);
  buzzerPhase = BUZZER_ON;
  buzzerDeadlineMs = millis() + buzzerOnDurationMs;
}

void updateBuzzer() {
  if (buzzerPhase == BUZZER_IDLE ||
      static_cast<int32_t>(millis() - buzzerDeadlineMs) < 0) {
    return;
  }
  if (buzzerPhase == BUZZER_ON) {
    digitalWrite(BUZZER_PIN, !BUZZER_ACTIVE_LEVEL);
    --buzzerBeepsRemaining;
    if (buzzerBeepsRemaining == 0) {
      buzzerPhase = BUZZER_IDLE;
    } else {
      buzzerPhase = BUZZER_GAP;
      buzzerDeadlineMs = millis() + buzzerGapDurationMs;
    }
  } else {
    digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_LEVEL);
    buzzerPhase = BUZZER_ON;
    buzzerDeadlineMs = millis() + buzzerOnDurationMs;
  }
}

void startTemperatureMeasurement() {
  temperatureMeasuring = true;
  temperatureRealtime = false;
  temperatureResultValid = false;
  temperatureSampleCount = 0;
  temperatureStartedMs = millis();
  lastTemperatureSequence = gy614.sampleSequence();
  Serial.println("Temperature measurement started");
}

void startRealtimeTemperature() {
  temperatureMeasuring = false;
  temperatureRealtime = true;
  temperatureResultValid = false;
  lastTemperatureSequence = gy614.sampleSequence();
  Serial.println("Realtime temperature started");
}

float filteredTemperature(const float* samples, uint8_t count) {
  float sorted[TEMPERATURE_SAMPLE_COUNT];
  for (uint8_t index = 0; index < count; ++index) {
    sorted[index] = samples[index];
  }
  for (uint8_t index = 1; index < count; ++index) {
    const float value = sorted[index];
    int position = index - 1;
    while (position >= 0 && sorted[position] > value) {
      sorted[position + 1] = sorted[position];
      --position;
    }
    sorted[position + 1] = value;
  }

  const float median = sorted[count / 2];
  float sum = 0.0f;
  uint8_t accepted = 0;
  for (uint8_t index = 0; index < count; ++index) {
    if (fabsf(sorted[index] - median) <= TEMPERATURE_OUTLIER_C) {
      sum += sorted[index];
      ++accepted;
    }
  }
  return accepted >= 3 ? sum / accepted : median;
}

void updateTemperatureMeasurement() {
  if (!temperatureMeasuring && !temperatureRealtime) {
    return;
  }

  const uint32_t sequence = gy614.sampleSequence();
  if (sequence != lastTemperatureSequence) {
    lastTemperatureSequence = sequence;
    const float sample = gy614.bo();
    if (isfinite(sample) && sample >= TEMPERATURE_MIN_C &&
        sample <= TEMPERATURE_MAX_C) {
      if (temperatureRealtime) {
        filteredTemperatureC = sample;
        temperatureResultValid = true;
        lastTemperatureResultMs = millis();
        Serial.printf("Realtime temperature: %.2f C\n", sample);
      } else {
        temperatureSamples[temperatureSampleCount++] = sample;
        Serial.printf(
            "Temperature sample %u/%u: %.2f C\n",
            temperatureSampleCount, TEMPERATURE_SAMPLE_COUNT, sample);
      }
    }
  }

  if (temperatureRealtime) {
    if (temperatureResultValid &&
        millis() - lastTemperatureResultMs > 1000) {
      temperatureResultValid = false;
    }
  } else if (temperatureSampleCount >= TEMPERATURE_SAMPLE_COUNT) {
    filteredTemperatureC = filteredTemperature(
        temperatureSamples, TEMPERATURE_SAMPLE_COUNT);
    temperatureResultValid = true;
    lastTemperatureResultMs = millis();
    temperatureMeasuring = false;
    startBuzzerPattern(1, 200, 0);
    Serial.printf("Temperature result: %.2f C\n", filteredTemperatureC);
  } else if (millis() - temperatureStartedMs >= TEMPERATURE_TIMEOUT_MS) {
    temperatureMeasuring = false;
    temperatureResultValid = false;
    startBuzzerPattern(2, 100, 100);
    Serial.println("Temperature measurement timed out");
  }
}

void updateButton() {
  const bool rawState = digitalRead(BUTTON_PIN);
  if (rawState != buttonRawState) {
    buttonRawState = rawState;
    buttonChangedMs = millis();
  }
  if (millis() - buttonChangedMs >= 30 && rawState != buttonStableState) {
    buttonStableState = rawState;
    if (buttonStableState == LOW) {
      buttonPressedMs = millis();
      buttonLongPressHandled = false;
      startTemperatureMeasurement();
    } else if (buttonLongPressHandled) {
      temperatureRealtime = false;
      Serial.println("Realtime temperature stopped");
    }
  }
  if (buttonStableState == LOW && !buttonLongPressHandled &&
      millis() - buttonPressedMs >= TEMPERATURE_LONG_PRESS_MS) {
    buttonLongPressHandled = true;
    startRealtimeTemperature();
  }
}

void publishHealthPacket() {
  const bool heartValid = bpmValid && (millis() - lastBeatShowMs < 3000);
  const bool temperatureValid = temperatureResultValid && !temperatureMeasuring;
  HealthPacket packet{};
  packet.version = 1;
  packet.sequence = bleSequence++;
  packet.heartRateBpm = heartValid ? static_cast<uint16_t>(bpm) : 0;
  packet.temperatureCenti = temperatureValid
      ? static_cast<int16_t>(lroundf(filteredTemperatureC * 100.0f))
      : 0;
  packet.flags = (heartValid ? HEART_RATE_VALID : 0) |
                 (temperatureValid ? TEMPERATURE_VALID : 0);
  packet.uptimeMs = millis();
  packet.crc = crc16Ccitt(
      reinterpret_cast<const uint8_t*>(&packet), sizeof(packet) - sizeof(packet.crc));
  healthCharacteristic->setValue(
      reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
  if (bleConnected) {
    healthCharacteristic->notify();
  }
}

void resetPpgBuffer() {
  waveIdx = 0;
  waveCount = 0;
  ppgFiltered = 0.0f;
  ppgFilterInitialized = false;
}

bool selectPpgSource(PpgSource source) {
  if (source < ppgSource) {
    return false;
  }
  if (source != ppgSource) {
    ppgSource = source;
    resetPpgBuffer();
  }
  return true;
}

void pushPpgSample(int32_t raw, PpgSource source) {
  if (!selectPpgSource(source)) {
    return;
  }

  if (!ppgFilterInitialized) {
    ppgFiltered = static_cast<float>(raw);
    ppgFilterInitialized = true;
  } else {
    ppgFiltered += PPG_EMA_ALPHA * (static_cast<float>(raw) - ppgFiltered);
  }

  waveBuf[waveIdx] = static_cast<int32_t>(lroundf(ppgFiltered));
  waveIdx = (waveIdx + 1) % WAVE_W;
  if (waveCount < WAVE_W) {
    ++waveCount;
  }
  lastPpgMs = millis();
}

void processSpO2Result(const char* payload) {
  int oxygen = 0;
  int pulse = 0;
  int perfusion = 0;
  if (sscanf(payload, "%d,%d,%d", &oxygen, &pulse, &perfusion) != 3) {
    return;
  }

  lastSpO2ResultMs = millis();
  oxygenValid = oxygen >= 50 && oxygen <= 100;
  bpmValid = pulse >= 30 && pulse <= 250;
  if (oxygenValid) {
    oxygenPercent = oxygen;
  }
  if (bpmValid) {
    bpm = pulse;
    lastBeatShowMs = lastSpO2ResultMs;
  }
  perfusionIndex = perfusion;
  Serial.printf(
      "SpO2 %s%d%%  BPM %s%d  PI %d\n",
      oxygenValid ? "" : "invalid:", oxygen,
      bpmValid ? "" : "invalid:", pulse,
      perfusionIndex);
}

void processSpO2Line(char* line) {
  if (strncmp(line, "U1:", 3) == 0) {
    char* end = nullptr;
    const long raw = strtol(line + 3, &end, 10);
    if (end != line + 3 && *end == '\0') {
      pushPpgSample(static_cast<int32_t>(raw), PPG_SOURCE_U1);
    }
    return;
  }

  if (strncmp(line, "U2:", 3) == 0) {
    char* payload = line + 3;
    if (strchr(payload, ',') != nullptr) {
      processSpO2Result(payload);
    } else {
      char* end = nullptr;
      const long raw = strtol(payload, &end, 10);
      if (end != payload && *end == '\0') {
        pushPpgSample(static_cast<int32_t>(raw), PPG_SOURCE_U2_SCALAR);
      }
    }
    return;
  }

  // Some module revisions label their only waveform stream as S1.
  if (strncmp(line, "S1:", 3) == 0) {
    char* end = nullptr;
    const long raw = strtol(line + 3, &end, 10);
    if (end != line + 3 && *end == '\0') {
      pushPpgSample(static_cast<int32_t>(raw), PPG_SOURCE_S1);
    }
  }
}

void requestSpO2Mode0() {
  spo2Serial.print("AT+MD:0\r\n");
  lastSpO2ModeRequestMs = millis();
  Serial.println("Requested blood-oxygen module mode 0");
}

void updateSpO2() {
  while (spo2Serial.available() > 0) {
    const char value = static_cast<char>(spo2Serial.read());
    if (value == '\r' || value == '\n') {
      if (spo2LineLength > 0) {
        spo2Line[spo2LineLength] = '\0';
        processSpO2Line(spo2Line);
        spo2LineLength = 0;
      }
    } else if (spo2LineLength + 1 < sizeof(spo2Line)) {
      spo2Line[spo2LineLength++] = value;
    } else {
      spo2LineLength = 0;
    }
  }

  const uint32_t now = millis();
  if (bpmValid && now - lastBeatShowMs > SPO2_DATA_TIMEOUT_MS) {
    bpmValid = false;
  }
  if (oxygenValid && now - lastSpO2ResultMs > SPO2_DATA_TIMEOUT_MS) {
    oxygenValid = false;
  }
  if (now - lastPpgMs > SPO2_DATA_TIMEOUT_MS &&
      now - lastSpO2ModeRequestMs >= SPO2_MODE_RETRY_MS) {
    resetPpgBuffer();
    ppgSource = PPG_SOURCE_NONE;
    requestSpO2Mode0();
  }
}

void drawScreen() {
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);

  display.print("H:");
  if (bpmValid) {
    display.print(bpm);
  } else {
    display.print("--");
  }
  display.print("  T:");
  if (temperatureRealtime) {
    if (temperatureResultValid) {
      display.print(filteredTemperatureC, 2);
    } else {
      display.print("--");
    }
  } else if (temperatureMeasuring) {
    display.print(temperatureSampleCount);
    display.print("/");
    display.print(TEMPERATURE_SAMPLE_COUNT);
  } else if (temperatureResultValid) {
    display.print(filteredTemperatureC, 2);
  } else {
    display.print("BTN");
  }

  // 波形区分隔线
  display.drawFastHLine(0, WAVE_Y0 - 2, 128, SH110X_WHITE);

  if (waveCount >= 2 && millis() - lastPpgMs <= SPO2_DATA_TIMEOUT_MS) {
    const int oldest = (waveIdx - waveCount + WAVE_W) % WAVE_W;
    int32_t minimum = waveBuf[oldest];
    int32_t maximum = waveBuf[oldest];
    for (int sample = 1; sample < waveCount; ++sample) {
      const int32_t value = waveBuf[(oldest + sample) % WAVE_W];
      if (value < minimum) minimum = value;
      if (value > maximum) maximum = value;
    }

    float low = static_cast<float>(minimum);
    float high = static_cast<float>(maximum);
    const float center = (low + high) * 0.5f;
    const float minimumSpan = fmaxf(8.0f, fabsf(center) * 0.02f);
    if (high - low < minimumSpan) {
      low = center - minimumSpan * 0.5f;
      high = center + minimumSpan * 0.5f;
    }
    const float padding = (high - low) * 0.08f;
    low -= padding;
    high += padding;

    int previousX = WAVE_W - waveCount;
    int previousY = WAVE_Y0 + WAVE_H / 2;
    for (int sample = 0; sample < waveCount; ++sample) {
      const int32_t value = waveBuf[(oldest + sample) % WAVE_W];
      const float normalized = (high - static_cast<float>(value)) / (high - low);
      const int x = WAVE_W - waveCount + sample;
      const int y = constrain(
          WAVE_Y0 + 1 + static_cast<int>(lroundf(normalized * (WAVE_H - 3))),
          WAVE_Y0 + 1,
          WAVE_Y0 + WAVE_H - 2);
      if (sample > 0) {
        display.drawLine(previousX, previousY, x, y, SH110X_WHITE);
      }
      previousX = x;
      previousY = y;
    }
  } else {
    display.setCursor(39, WAVE_Y0 + WAVE_H / 2 - 4);
    display.print("PPG WAIT");
  }

  display.display();
}

void setup() {
  Serial.begin(115200);
  resetPpgBuffer();
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, !BUZZER_ACTIVE_LEVEL);

  Wire.begin(OLED_SDA, OLED_SCL);
  gy614.begin(GY614_RX_PIN, GY614_TX_PIN);
  spo2Serial.begin(115200, SERIAL_8N1, SPO2_RX_PIN, SPO2_TX_PIN);
  display.begin(OLED_ADDR, true);
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("Vitals starting...");
  display.display();
  startBle();
  delay(200);
  requestSpO2Mode0();
}

void loop() {
  gy614.update(TEMPERATURE_QUERY_INTERVAL_MS);
  updateSpO2();
  updateButton();
  updateBuzzer();
  updateTemperatureMeasurement();

  if (millis() - lastUiMs >= 50) {
    lastUiMs = millis();
    drawScreen();
  }

  if (millis() - lastBleMs >= 200) {
    lastBleMs = millis();
    publishHealthPacket();
  }

  if (bleRestartAdvertising) {
    bleRestartAdvertising = false;
    BLEDevice::startAdvertising();
  }
}
