#include <SPI.h>
#include <mcp_can.h>
#include <TFT_eSPI.h>
#include <math.h>

// ---------------- MCP2515 on HSPI ----------------
#define HSPI_SCK 22
#define HSPI_MISO 17
#define HSPI_MOSI 25
#define CAN_CS 27

SPIClass hspi(HSPI);
MCP_CAN *CAN0 = nullptr;
TFT_eSPI tft;

// Real car CAN
#define CAN_SPEED CAN_500KBPS
#define CAN_CLOCK MCP_8MHZ

// ---------------- Gauge geometry ----------------
const int GAUGE_CX = 157;
const int GAUGE_CY = 112;
const int GAUGE_R = 109;
const int RPM_MAX = 6000;

// ---------------- CAN IDs ----------------
const uint16_t ID_ENGINE_RPM = 0x2C4;

// Generic OBD for coolant
const uint16_t OBD_REQ_ID = 0x7DF;
const uint16_t OBD_RSP_MIN = 0x7E8;
const uint16_t OBD_RSP_MAX = 0x7EF;

// Camry hybrid battery current, found working:
// TX 7E2: 02 21 CE 00 00 00 00 00
// RX 7EA: 10 xx 61 CE A B C D
const uint16_t HV_REQ_ID = 0x7E2;
const uint16_t HV_RSP_ID = 0x7EA;

// ---------------- RPM ----------------
const float RPM_SCALE = 1.0f;

float rpm = 0;
float rpmFilt = 0;
uint32_t lastRpmMs = 0;
char lastRpmBuf[] = "    ";
const uint32_t RPM_STALE_MS = 1000;

// ---------------- Coolant + HV battery current ----------------
int coolantC = -999;

float hvCurrentA = 0.0f;
bool haveHvCurrent = false;
uint32_t lastHvGoodMs = 0;

const uint32_t COOLANT_POLL_MS = 5000;
const uint32_t HV_POLL_MS = 250;
const uint32_t REQ_TIMEOUT_MS = 100;

uint32_t lastCoolantPoll = 0;
uint32_t lastHvPoll = 0;

// Non-blocking request state
enum ReqType {
  REQ_NONE,
  REQ_COOLANT,
  REQ_HV_CURRENT
};

ReqType pendingReq = REQ_NONE;
uint32_t requestStartMs = 0;

// ---------------- Display state ----------------
uint32_t lastDraw = 0;
float lastNeedleRpm = -1;
float lastRpmText = -9999;
int lastCoolantDraw = -9999;
float lastHvDraw = 99999.0f;
bool lastHaveHvDraw = false;

// ---------------- Helpers ----------------
static uint16_t be16(const uint8_t *p) {
  return ((uint16_t)p[0] << 8) | p[1];
}

float rpmToAngleDeg(float r) {
  r = constrain(r, 0, RPM_MAX);
  return 225.0f - (r / RPM_MAX) * 270.0f;
}

float decodeHvCurrent21CE(uint8_t B, uint8_t C) {
  // Prius/Toyota formula:
  // current = ((256 * B) + C) / 100 - 327.68
  // On this Camry: positive = charging/regen, negative = discharging/load.
  return ((256.0f * B) + C) / 100.0f - 327.68f;
}
// ------------------------------------------------------------
// Color correction for displays with red/blue swapped.
//
// TFT_eSPI colors are RGB565:
// RRRRR GGGGGG BBBBB
//
// This swaps red and blue while leaving green unchanged.
// ------------------------------------------------------------
uint16_t fixColor(uint16_t c) {
  uint16_t r = (c & 0xF800) >> 11;  // red   5 bits
  uint16_t g = (c & 0x07E0);        // green 6 bits, already in place
  uint16_t b = (c & 0x001F);        // blue  5 bits

  return (b << 11) | g | r;
}

void setTextColorFixed(uint16_t fg, uint16_t bg = TFT_BLACK) {
  tft.setTextColor(fixColor(fg), fixColor(bg));
}

void fillScreenFixed(uint16_t c) {
  tft.fillScreen(fixColor(c));
}

void fillRectFixed(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t c) {
  tft.fillRect(x, y, w, h, fixColor(c));
}

void drawLineFixed(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint16_t c) {
  tft.drawLine(x0, y0, x1, y1, fixColor(c));
}

void drawCircleFixed(int32_t x, int32_t y, int32_t r, uint16_t c) {
  tft.drawCircle(x, y, r, fixColor(c));
}

void fillCircleFixed(int32_t x, int32_t y, int32_t r, uint16_t c) {
  tft.fillCircle(x, y, r, fixColor(c));
}

void drawStringFixed(const String &s, int32_t x, int32_t y, uint8_t font) {
  tft.drawString(s, x, y, font);
}

void drawRightStringFixed(const String &s, int32_t x, int32_t y, uint8_t font) {
  tft.drawRightString(s, x, y, font);
}
void fillTriangleFixed(int32_t x0, int32_t y0,
                       int32_t x1, int32_t y1,
                       int32_t x2, int32_t y2,
                       uint16_t c) {
  tft.fillTriangle(x0, y0, x1, y1, x2, y2, fixColor(c));
}
// ---------------- Non-blocking requests ----------------
bool sendCoolantRequest() {
  if (pendingReq != REQ_NONE) return false;

  uint8_t req[8] = { 0x02, 0x01, 0x05, 0, 0, 0, 0, 0 };

  byte tx = CAN0->sendMsgBuf(OBD_REQ_ID, 0, 8, req);
  if (tx != CAN_OK) return false;

  pendingReq = REQ_COOLANT;
  requestStartMs = millis();
  return true;
}

bool sendHvCurrentRequest() {
  if (pendingReq != REQ_NONE) return false;

  uint8_t req[8] = { 0x02, 0x21, 0xCE, 0, 0, 0, 0, 0 };

  byte tx = CAN0->sendMsgBuf(HV_REQ_ID, 0, 8, req);
  if (tx != CAN_OK) return false;

  pendingReq = REQ_HV_CURRENT;
  requestStartMs = millis();
  return true;
}

void sendHvFlowControl() {
  uint8_t fc[8] = { 0x30, 0, 0, 0, 0, 0, 0, 0 };
  CAN0->sendMsgBuf(HV_REQ_ID, 0, 8, fc);
}

void handleCoolantResponse(unsigned long id, unsigned char len, unsigned char *buf) {
  if (id < OBD_RSP_MIN || id > OBD_RSP_MAX) return;
  if (len < 4) return;

  // Negative response
  if (buf[1] == 0x7F) {
    pendingReq = REQ_NONE;
    return;
  }

  if (buf[1] == 0x41 && buf[2] == 0x05) {
    coolantC = (int)buf[3] - 40;
    pendingReq = REQ_NONE;
  }
}

void handleHvCurrentResponse(unsigned long id, unsigned char len, unsigned char *buf) {
  if (id != HV_RSP_ID) return;
  if (len < 4) return;

  uint8_t frameType = buf[0] & 0xF0;

  // Negative response: 03 7F 21 xx
  if ((buf[0] & 0x0F) >= 3 && buf[1] == 0x7F) {
    pendingReq = REQ_NONE;
    return;
  }

  // Multi-frame first frame:
  // 10 xx 61 CE A B C D
  if (frameType == 0x10) {
    if (len >= 7 && buf[2] == 0x61 && buf[3] == 0xCE) {
      uint8_t B = buf[5];
      uint8_t C = buf[6];

      hvCurrentA = decodeHvCurrent21CE(B, C);
      haveHvCurrent = true;
      lastHvGoodMs = millis();

      // We already have current, but flow control keeps ISO-TP polite.
      sendHvFlowControl();

      pendingReq = REQ_NONE;
    }
    return;
  }

  // Possible single-frame response:
  // 05 61 CE A B C ...
  if (frameType == 0x00) {
    uint8_t sfLen = buf[0] & 0x0F;

    if (sfLen >= 5 && len >= 6 && buf[1] == 0x61 && buf[2] == 0xCE) {
      uint8_t B = buf[4];
      uint8_t C = buf[5];

      hvCurrentA = decodeHvCurrent21CE(B, C);
      haveHvCurrent = true;
      lastHvGoodMs = millis();

      pendingReq = REQ_NONE;
    }
    return;
  }
}

void serviceRequests() {
  uint32_t now = millis();

  if (pendingReq != REQ_NONE && now - requestStartMs > REQ_TIMEOUT_MS) {
    pendingReq = REQ_NONE;
  }

  if (pendingReq != REQ_NONE) return;

  // HV current gets priority because it is the dynamic display value.
  if (now - lastHvPoll >= HV_POLL_MS) {
    lastHvPoll = now;
    if (sendHvCurrentRequest()) return;
  }

  if (now - lastCoolantPoll >= COOLANT_POLL_MS) {
    lastCoolantPoll = now;
    sendCoolantRequest();
  }
}

// ---------------- CAN frame handling ----------------
void handleFrame(unsigned long id, unsigned char len, unsigned char *buf) {
  // Passive tachometer RPM
  if (id == ID_ENGINE_RPM && len >= 2) {
    uint16_t raw = be16(&buf[0]);
    rpm = raw * RPM_SCALE;
    lastRpmMs = millis();
  }

  // Replies for whichever request is pending
  if (pendingReq == REQ_COOLANT) {
    handleCoolantResponse(id, len, buf);
  } else if (pendingReq == REQ_HV_CURRENT) {
    handleHvCurrentResponse(id, len, buf);
  }
}

// ---------------- Drawing ----------------
void drawWideNeedle(float rpmValue, uint16_t color) {
  float a = rpmToAngleDeg(rpmValue) * DEG_TO_RAD;

  int tipR = GAUGE_R - 20;
  int tailR = 14;
  int halfWidth = 5;

  float ca = cos(a);
  float sa = sin(a);

  int tipX = GAUGE_CX + ca * tipR;
  int tipY = GAUGE_CY - sa * tipR;

  int tailX = GAUGE_CX - ca * tailR;
  int tailY = GAUGE_CY + sa * tailR;

  float px = sin(a);
  float py = cos(a);

  int x1 = tailX + px * halfWidth;
  int y1 = tailY + py * halfWidth;

  int x2 = tailX - px * halfWidth;
  int y2 = tailY - py * halfWidth;

  fillTriangleFixed(tipX, tipY, x1, y1, x2, y2, color);
  drawLineFixed(GAUGE_CX, GAUGE_CY, tipX, tipY, color);
}

void redrawGaugeOverlays() {
  drawCircleFixed(GAUGE_CX, GAUGE_CY, GAUGE_R, TFT_LIGHTGREY);
  drawCircleFixed(GAUGE_CX, GAUGE_CY, GAUGE_R - 1, TFT_DARKGREY);

  for (int i = 0; i <= 6; i++) {
    float val = i * 1000.0f;
    float a = rpmToAngleDeg(val) * DEG_TO_RAD;

    int x1 = GAUGE_CX + cos(a) * (GAUGE_R - 13);
    int y1 = GAUGE_CY - sin(a) * (GAUGE_R - 13);
    int x2 = GAUGE_CX + cos(a) * GAUGE_R;
    int y2 = GAUGE_CY - sin(a) * GAUGE_R;

    tft.drawWideLine(x1, y1, x2, y2, 4, TFT_WHITE);

    int xt = GAUGE_CX + cos(a) * (GAUGE_R - 34);
    int yt = GAUGE_CY - sin(a) * (GAUGE_R - 34);

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawCentreString(String(i), xt, yt - 9, 4);
  }

  for (int i = 0; i <= 30; i++) {
    if (i % 5 == 0) continue;

    float val = i * 200.0f;
    float a = rpmToAngleDeg(val) * DEG_TO_RAD;

    int x1 = GAUGE_CX + cos(a) * (GAUGE_R - 8);
    int y1 = GAUGE_CY - sin(a) * (GAUGE_R - 8);
    int x2 = GAUGE_CX + cos(a) * GAUGE_R;
    int y2 = GAUGE_CY - sin(a) * GAUGE_R;

    tft.drawLine(x1, y1, x2, y2, TFT_WHITE);
  }
}

void drawGaugeStatic() {
  fillScreenFixed(TFT_BLACK);
  redrawGaugeOverlays();

  setTextColorFixed(TFT_CYAN, TFT_BLACK);
  tft.drawString("COOLANT", 8, 202, 2);

  // Keep label simple. Actual number color follows sign:
  // red = discharge/load, green = charge/regen.
  setTextColorFixed(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("HV AMP", 245, 202, 2);
}

void drawBottomValuesIfChanged() {
  if (coolantC != lastCoolantDraw) {
    fillRectFixed(8, 220, 105, 20, TFT_BLACK);
    setTextColorFixed(TFT_GREEN, TFT_BLACK);

    if (coolantC > -100) tft.drawString(String(coolantC) + " C", 8, 220, 4);
    else tft.drawString("--- C", 8, 220, 4);

    lastCoolantDraw = coolantC;
  }

  // Draw HV amps if value changed enough, if first draw, or if data availability changed.
  bool hvFresh = haveHvCurrent && (millis() - lastHvGoodMs < 1500);

  if (hvFresh != lastHaveHvDraw || fabs(hvCurrentA - lastHvDraw) >= 0.1f) {
    fillRectFixed(198, 220, 122, 20, TFT_BLACK);
//hvFresh = true; // test only
//hvCurrentA = -4.8;   // should DISPLAY as +4.8A in green for charging
    if (hvFresh) {
      // User reported the decoded-current sign needed to be flipped for display.
      // Color now follows the DISPLAYED sign:
      // + amps = green, - amps = red.
      // No white/neutral deadband, so even +/-0.8 keeps its sign color.
      float shownHvCurrentA = -hvCurrentA;

      if (shownHvCurrentA < 0.0f) {
        setTextColorFixed(TFT_RED, TFT_BLACK);
      } else {
        setTextColorFixed(TFT_GREEN, TFT_BLACK);
      }

      char buf[16];
      snprintf(buf, sizeof(buf), "%+.1fA", shownHvCurrentA);
      tft.drawRightString(String(buf), 318, 220, 4);
    } else {
      setTextColorFixed(TFT_DARKGREY, TFT_BLACK);
      tft.drawRightString("--.-A", 318, 220, 4);
    }

    lastHvDraw = hvCurrentA;
    lastHaveHvDraw = hvFresh;
  }
}

void redrawGaugeNumeralsOnly() {
  for (int i = 0; i <= 6; i++) {
    float val = i * 1000.0f;
    float a = rpmToAngleDeg(val) * DEG_TO_RAD;

    int xt = GAUGE_CX + cos(a) * (GAUGE_R - 34);
    int yt = GAUGE_CY - sin(a) * (GAUGE_R - 34);

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawCentreString(String(i), xt, yt - 9, 4);
  }
}

void drawDynamic() {
  if (millis() - lastRpmMs > RPM_STALE_MS) {
    rpm = 0;
  }

  float targetRpm = rpm;

  if (targetRpm < 100.0f) {
    targetRpm = 0.0f;
  }

  rpmFilt += 0.18f * (targetRpm - rpmFilt);

  if (rpmFilt < 30.0f && targetRpm == 0.0f) {
    rpmFilt = 0.0f;
  }

  if (lastNeedleRpm < 0 || fabs(rpmFilt - lastNeedleRpm) > 35) {
    if (lastNeedleRpm >= 0) {
      drawWideNeedle(lastNeedleRpm, TFT_BLACK);
      redrawGaugeNumeralsOnly();
    }

    drawWideNeedle(rpmFilt, TFT_RED);

    fillCircleFixed(GAUGE_CX, GAUGE_CY, 9, TFT_RED);
    drawCircleFixed(GAUGE_CX, GAUGE_CY, 9, TFT_RED);

    lastNeedleRpm = rpmFilt;
  }

  // Numeric RPM display.
  // Important: force an update when RPM falls below display threshold,
  // otherwise the old number can remain even though the needle is at zero.
  bool rpmShouldBlank = (rpmFilt < 250.0f);
  bool rpmTextIsNotBlank =
    (lastRpmBuf[0] != ' ' || lastRpmBuf[1] != ' ' ||
     lastRpmBuf[2] != ' ' || lastRpmBuf[3] != ' ');

  if (fabs(rpmFilt - lastRpmText) > 250 || (rpmShouldBlank && rpmTextIsNotBlank)) {
    char buf[5];

    if (rpmShouldBlank) {
      rpmFilt = 0.0f;
      strcpy(buf, "    ");      // Four spaces = blank RPM text
    } else {
      snprintf(buf, sizeof(buf), "%4d", (int)(rpmFilt + 0.5f));
    }

    // Character positions
    const int x0 = GAUGE_CX - 30;
    const int y0 = GAUGE_CY + 65;
    const int dx = 16;  // spacing between digits

    for (int i = 0; i < 4; i++) {
      if (buf[i] != lastRpmBuf[i]) {
        // Erase ONLY this character
        tft.setTextColor(TFT_BLACK, TFT_BLACK);
        tft.drawChar(lastRpmBuf[i], x0 + i * dx, y0, 4);

        // Draw new character only when it is not a blank space.
        // Drawing a space does not reliably clear with all TFT fonts,
        // so the black draw above is the actual erase operation.
        if (buf[i] != ' ') {
          setTextColorFixed(TFT_YELLOW, TFT_BLACK);
          tft.drawChar(buf[i], x0 + i * dx, y0, 4);
        }

        lastRpmBuf[i] = buf[i];
      }
    }

    lastRpmText = rpmFilt;
  }

  drawBottomValuesIfChanged();
}

// ---------------- Setup / loop ----------------
void setup() {
  // Serial monitor not needed.
  delay(300);

  tft.init();
  tft.setRotation(1);
  fillScreenFixed(TFT_BLACK);

  pinMode(CAN_CS, OUTPUT);
  digitalWrite(CAN_CS, HIGH);

  hspi.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, CAN_CS);
  CAN0 = new MCP_CAN(CAN_CS, &hspi);

  byte st = CAN0->begin(MCP_ANY, CAN_SPEED, CAN_CLOCK);
  if (st != CAN_OK) {
    setTextColorFixed(TFT_RED, TFT_BLACK);
    tft.drawString("CAN INIT FAIL", 8, 8, 2);
    while (1) delay(1000);
  }

  CAN0->setMode(MCP_NORMAL);

  drawGaugeStatic();

  lastNeedleRpm = -1;
  lastRpmText = -9999;
  lastCoolantDraw = -9999;
  lastHvDraw = 99999.0f;
  lastHaveHvDraw = false;

  // Stagger initial polling
  lastCoolantPoll = millis() - COOLANT_POLL_MS;
  lastHvPoll = millis() - HV_POLL_MS;
}

void loop() {
  while (CAN0->checkReceive() == CAN_MSGAVAIL) {
    unsigned long id;
    unsigned char len;
    unsigned char buf[8];

    CAN0->readMsgBuf(&id, &len, buf);
    handleFrame(id, len, buf);
  }

  serviceRequests();

  uint32_t now = millis();

  if (now - lastDraw > 25) {
    lastDraw = now;
    drawDynamic();
  }
}

