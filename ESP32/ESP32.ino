/*
  ESP32-S3 + SH1106 + RT7015
  OLED : SCL=15 SDA=16
  RT7015 SOUT -> GPIO8
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define OLED_SDA 16
#define OLED_SCL 15
#define OLED_ADDR 0x3C
#define SOUT_PIN 8

Adafruit_SH1106G display(128, 64, &Wire, -1);

// ---------- 波形缓存（128列）----------
static const int WAVE_W = 128;
static const int WAVE_Y0 = 18;
static const int WAVE_H = 46;
uint8_t waveBuf[WAVE_W];
int waveIdx = 0;

// ---------- 心率 ----------
volatile uint32_t lastBeatUs = 0;
volatile uint32_t beatIntervalUs = 0;
volatile bool newBeat = false;

static const int AVG_N = 8;
uint32_t intervals[AVG_N];
int intervalCnt = 0;
int intervalIdx = 0;
int bpm = 0;
bool bpmValid = false;

uint32_t lastSampleMs = 0;
uint32_t lastUiMs = 0;
uint32_t lastBeatShowMs = 0;

void IRAM_ATTR onSoutRise() {
  uint32_t now = micros();
  uint32_t dt = now - lastBeatUs;
  // 消抖：对应约 40~200 bpm → 300ms~1500ms
  if (dt < 300000UL || dt > 1500000UL) {
    if (dt < 300000UL) return;  // 毛刺
    // 超时则重新锚定，不记入
    lastBeatUs = now;
    return;
  }
  lastBeatUs = now;
  beatIntervalUs = dt;
  newBeat = true;
}

void pushInterval(uint32_t us) {
  intervals[intervalIdx] = us;
  intervalIdx = (intervalIdx + 1) % AVG_N;
  if (intervalCnt < AVG_N) intervalCnt++;

  uint64_t sum = 0;
  for (int i = 0; i < intervalCnt; i++) sum += intervals[i];
  float avgMs = (sum / (float)intervalCnt) / 1000.0f;
  int v = (int)(60000.0f / avgMs + 0.5f);
  if (v >= 40 && v <= 200) {
    bpm = v;
    bpmValid = true;
  }
}

void sampleWave() {
  // HIGH 画在偏上，LOW 偏下
  uint8_t y = digitalRead(SOUT_PIN) ? (WAVE_Y0 + 8) : (WAVE_Y0 + WAVE_H - 8);
  waveBuf[waveIdx] = y;
  waveIdx = (waveIdx + 1) % WAVE_W;
}

void drawScreen() {
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);

  if (bpmValid && (millis() - lastBeatShowMs < 3000)) {
    display.print("HR ");
    display.setTextSize(2);
    display.print(bpm);
    display.setTextSize(1);
    display.print(" bpm");
  } else {
    display.print("HR -- bpm");
    bpmValid = false;
  }

  // 波形区分隔线
  display.drawFastHLine(0, WAVE_Y0 - 2, 128, SH110X_WHITE);

  // 按时间顺序画折线（最旧在左）
  for (int x = 1; x < WAVE_W; x++) {
    int i0 = (waveIdx + x - 1) % WAVE_W;
    int i1 = (waveIdx + x) % WAVE_W;
    display.drawLine(x - 1, waveBuf[i0], x, waveBuf[i1], SH110X_WHITE);
  }

  display.display();
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < WAVE_W; i++) {
    waveBuf[i] = WAVE_Y0 + WAVE_H / 2;
  }

  pinMode(SOUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SOUT_PIN), onSoutRise, RISING);

  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(OLED_ADDR, true);
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("RT7015 SOUT:8");
  display.display();
  delay(500);
}

void loop() {
  // 处理心拍
  if (newBeat) {
    noInterrupts();
    uint32_t dt = beatIntervalUs;
    newBeat = false;
    interrupts();
    pushInterval(dt);
    lastBeatShowMs = millis();
    Serial.printf("BPM %d  dt=%lu ms\n", bpm, dt / 1000UL);
  }

  // ~50Hz 采样方波，滚动显示
  if (millis() - lastSampleMs >= 20) {
    lastSampleMs = millis();
    sampleWave();
  }

  if (millis() - lastUiMs >= 50) {
    lastUiMs = millis();
    drawScreen();
  }
}
