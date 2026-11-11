// Audi A4 B6 1.8T (Bosch ME7.5, 8E0909518AL) gauge for M5StickC Plus2 + ELM327 Bluetooth
// Reads VAG measuring blocks over KWP2000 (5-baud init) - this ECU has no boost via OBD2 mode 01.
// Build: ESP32 Arduino core 2.0.17 + M5Unified.
//
// Button A (front, M5)  short = next page, hold 1 s = reset peaks
// Button B (side)       short = previous page
//
// Pages / blocks used (field numbers 1..4):
//   1 MAIN      : 115.4 boost act, 115.1 rpm, ATRV battery, 4.3 coolant
//   2 ZAPLON    : 4.4 IAT, 3.4 ign timing, 20.1-4 knock retard cyl 1-4
//   3 TURBO     : 115.3/115.4 boost spec/act, 118.3 N75 duty, 115.2 load, 3.2 MAF, 4.4 IAT
//   4 MIESZANKA : 5.3 speed, 1.3 lambda ctrl, 32.1/32.2 fuel trims idle/part, 3.3 throttle, 1.2 coolant
//   baro: 113.4 (read once at connect)

#include <M5Unified.h>
#include "BluetoothSerial.h"

BluetoothSerial BT;
M5Canvas cv(&M5.Display);

bool ready = false;
float baro = 1000;
float bv[256][4];          // last decoded value of every block/field
float volt = NAN;
float peakBoost = 0, peakRetard[4] = {0, 0, 0, 0};
int page = 0;
const int PAGES = 4;

// polling lists per page; 0x00 = ATRV (adapter voltage)
const uint8_t poll0[] = {0x73, 0x73, 0x04, 0x73, 0x73, 0x00};
const uint8_t poll1[] = {0x14, 0x03, 0x14, 0x04};
const uint8_t poll2[] = {0x73, 0x76, 0x73, 0x03, 0x04};
const uint8_t poll3[] = {0x05, 0x01, 0x20, 0x03};
const uint8_t *polls[] = {poll0, poll1, poll2, poll3};
const uint8_t pollLen[] = {sizeof(poll0), sizeof(poll1), sizeof(poll2), sizeof(poll3)};

#define F(b, f) bv[b][(f) - 1]

// ---------- status screen ----------
void status(const String &l1, const String &l2 = "", uint16_t color = TFT_WHITE) {
  auto &d = M5.Display;
  d.fillScreen(TFT_BLACK);
  d.setTextColor(color, TFT_BLACK);
  d.setTextSize(2);
  d.setCursor(4, 10);
  d.println(l1);
  d.setTextSize(1);
  d.setCursor(4, 50);
  d.println(l2);
  Serial.println(l1 + " | " + l2);
}

// ---------- ELM327 ----------
String elm(const String &cmd, uint32_t timeout = 3000) {
  while (BT.available()) BT.read();
  BT.print(cmd + "\r");
  String r;
  uint32_t t0 = millis();
  while (millis() - t0 < timeout) {
    while (BT.available()) {
      char c = BT.read();
      if (c == '>') { r.trim(); Serial.println(cmd + " -> " + r); return r; }
      r += c;
    }
    delay(2);
  }
  r.trim();
  Serial.println(cmd + " -> (timeout) " + r);
  return r;
}

String compact(String r) {
  r.replace(" ", ""); r.replace("\r", ""); r.replace("\n", "");
  r.toUpperCase();
  return r;
}

// VAG measuring-block formula (type, a, b)
float vagValue(uint8_t t, uint8_t a, uint8_t b) {
  switch (t) {
    case 0x01: return 0.2f * a * b;                      // rpm
    case 0x02: return a * 0.002f * b;                    // %
    case 0x03: return 0.002f * a * b;                    // deg
    case 0x05: return a * (b - 100) * 0.1f;              // °C
    case 0x06: return 0.001f * a * b;                    // V
    case 0x07: return 0.01f * a * b;                     // km/h
    case 0x0F: return 0.01f * a * b;                     // ms
    case 0x12: return 0.04f * a * b;                     // mbar
    case 0x14: return a * (b - 128) / 128.0f;            // % (lambda / trims)
    case 0x15: return 0.001f * a * b;                    // V
    case 0x17: return b / 256.0f * a;                    // %
    case 0x19: return b * 1.421f + a / 182.0f;           // g/s
    case 0x1A: return b - a;                             // °C
    case 0x1B: return fabsf(b - 128) * 0.01f * a;        // deg
    case 0x1E: return b / 12.0f * a;                     // deg (crank)
    case 0x21: return a ? 100.0f * b / a : 100.0f * b;   // %
    case 0x22: return (b - 128) * 0.01f * a;             // deg crank (knock retard)
    case 0x34: return (b - 128) * 0.01f * a;             // Nm
    default:   return NAN;
  }
}

bool readBlock(uint8_t block) {
  char cmd[5], hdr[5];
  snprintf(cmd, sizeof(cmd), "21%02X", block);
  snprintf(hdr, sizeof(hdr), "61%02X", block);
  String r = compact(elm(cmd, 2500));
  int i = r.indexOf(hdr);
  if (i < 0) return false;
  i += 4;
  for (int f = 0; f < 4; f++) {
    int p = i + f * 6;
    if (p + 6 > (int)r.length()) break;
    uint8_t t = strtol(r.substring(p, p + 2).c_str(), nullptr, 16);
    uint8_t a = strtol(r.substring(p + 2, p + 4).c_str(), nullptr, 16);
    uint8_t b = strtol(r.substring(p + 4, p + 6).c_str(), nullptr, 16);
    bv[block][f] = vagValue(t, a, b);
  }
  return true;
}

bool startVagSession() {
  for (int attempt = 0; attempt < 3; attempt++) {
    if (attempt > 0) {           // ECU may still hold an old session: close and wait > P3max
      elm("ATPC");
      status("Laczenie z ECU...", "proba " + String(attempt + 1) + "/3, czekam 6 s", TFT_CYAN);
      delay(6000);
    } else {
      status("Laczenie z ECU...", "KWP2000 5-baud, adres 01", TFT_CYAN);
    }
    elm("ATSP4");                // ISO 14230 KWP, 5-baud init
    elm("ATIIA01");              // init address 01 = engine
    elm("ATSH8110F1");           // physical addressing ECU 0x10, tester F1
    elm("ATWM8110F13E");         // keep-alive (TesterPresent) to same ECU
    elm("ATSTFF");
    String r = compact(elm("1089", 15000)); // VAG diagnostic session
    if (r.indexOf("5089") >= 0) return true;
  }
  return false;
}

// ---------- Bluetooth ----------
BTAddress elmAddr;
bool haveElmAddr = false;

bool isObdName(String n) {
  n.toUpperCase();
  return n.indexOf("OBD") >= 0 || n.indexOf("ELM") >= 0 || n.indexOf("V-LINK") >= 0 || n.indexOf("VLINK") >= 0;
}

// connect to address and confirm it is an ELM327 (flush junk left in its input first)
bool tryElm(BTAddress addr) {
  BT.connect(addr, 0, ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE);
  if (!BT.connected(8000)) { BT.disconnect(); return false; }
  delay(300);
  for (int i = 0; i < 3; i++) {
    BT.print("\r");
    delay(300);
    String up = elm("ATZ", 3000);
    up.toUpperCase();
    if (up.indexOf("ELM") >= 0) { elmAddr = addr; haveElmAddr = true; return true; }
  }
  BT.disconnect();
  return false;
}

bool connectBT() {
  if (haveElmAddr) {
    status("Lacze z ELM...", elmAddr.toString().c_str(), TFT_CYAN);
    if (tryElm(elmAddr)) return true;
  }
  status("Skanuje BT...", "ok. 10 s");
  BTScanResults *res = BT.discover(10000);
  if (!res || res->getCount() == 0) { status("Brak urzadzen BT", "ponawiam za 5 s", TFT_RED); return false; }

  // order: named OBD/ELM, then nameless (cheap clones), then everything else
  String list;
  BTAddress cand[16];
  String candName[16];
  int n = res->getCount(), nc = 0;
  for (int pass = 0; pass < 3; pass++)
    for (int i = 0; i < n && nc < 16; i++) {
      BTAdvertisedDevice *dev = res->getDevice(i);
      String name = dev->getName().c_str();
      if (pass == 0) list += (name.length() ? name : "(brak nazwy)") + " " + dev->getAddress().toString().c_str() + "\n";
      if ((pass == 0 && isObdName(name)) || (pass == 1 && name.length() == 0) || (pass == 2 && name.length() && !isObdName(name))) {
        cand[nc] = dev->getAddress();
        candName[nc++] = name.length() ? name : "(brak nazwy)";
      }
    }
  Serial.print(list);

  for (int i = 0; i < nc; i++) {
    status("Probuje:", candName[i] + "\n" + cand[i].toString().c_str(), TFT_CYAN);
    if (tryElm(cand[i])) return true;
  }
  status("Nie znaleziono ELM", list, TFT_ORANGE);
  return false;
}

void connectAll() {
  ready = false;
  if (BT.connected()) BT.disconnect();
  if (!connectBT()) return;
  elm("ATE0"); elm("ATL0"); elm("ATH0"); elm("ATAL");
  if (!startVagSession()) { status("ECU nie odpowiada", "Zaplon ON?\nponawiam za 5 s", TFT_ORANGE); return; }
  if (readBlock(0x71) && F(0x71, 4) > 800 && F(0x71, 4) < 1100) baro = F(0x71, 4);
  status("ECU OK!", "Cisnienie atm: " + String(baro, 0) + " mbar", TFT_GREEN);
  delay(1000);
  ready = true;
}

// ---------- drawing ----------
String fmt(float v, int dec, const char *suffix = "") {
  if (isnan(v)) return "--";
  return String(v, dec) + suffix;
}

void header(const char *title) {
  cv.setTextSize(1);
  cv.setTextColor(TFT_DARKGREY);
  cv.setTextDatum(top_left);
  cv.drawString(title, 4, 2);
  for (int i = 0; i < PAGES; i++) {   // page dots
    int x = 236 - (PAGES - i) * 9;
    if (i == page) cv.fillCircle(x, 6, 3, TFT_WHITE);
    else cv.drawCircle(x, 6, 3, TFT_DARKGREY);
  }
}

// label + value tile
void tile(int x, int y, const char *label, const String &value, uint16_t color = TFT_WHITE, int size = 3) {
  cv.setTextDatum(top_left);
  cv.setTextSize(1);
  cv.setTextColor(TFT_DARKGREY);
  cv.drawString(label, x, y);
  cv.setTextSize(size);
  cv.setTextColor(color);
  cv.drawString(value, x, y + 11);
}

void bar(int x, int y, int w, int h, float frac, uint16_t color, float zeroFrac = -1) {
  frac = isnan(frac) ? 0 : constrain(frac, 0.0f, 1.0f);
  cv.fillRect(x, y, w, h, 0x2104);
  cv.fillRect(x, y, w * frac, h, color);
  if (zeroFrac >= 0) cv.drawFastVLine(x + w * zeroFrac, y - 2, h + 4, TFT_WHITE);
}

uint16_t voltColor(float v) { return isnan(v) ? TFT_WHITE : v >= 13.2f ? TFT_GREEN : v >= 12.4f ? TFT_YELLOW : TFT_RED; }
uint16_t boostColor(float b) { return isnan(b) ? TFT_WHITE : b > 1.2f ? TFT_RED : b > 0.2f ? TFT_ORANGE : TFT_CYAN; }

void drawMain() {
  float boost = (F(0x73, 4) - baro) / 1000.0f;
  header("BOOST bar");
  cv.setTextColor(TFT_DARKGREY);
  cv.drawString("max " + fmt(peakBoost, 2), 110, 2);

  uint16_t bc = boostColor(boost);
  cv.setTextSize(5);
  cv.setTextColor(bc);
  char buf[12];
  snprintf(buf, sizeof(buf), "%+.2f", boost);
  cv.drawString(isnan(boost) ? "--.--" : buf, 4, 14);
  bar(4, 58, 232, 10, (boost + 0.8f) / 2.4f, bc, 0.8f / 2.4f);

  tile(4, 76, "RPM", fmt(F(0x73, 1), 0));
  tile(92, 76, "AKU V", fmt(volt, 1), voltColor(volt));
  tile(170, 76, "WODA C", fmt(F(0x04, 3), 0), F(0x04, 3) > 110 ? TFT_RED : TFT_WHITE);
  cv.setTextSize(1);
  cv.setTextColor(voltColor(volt));
  cv.drawString(isnan(volt) ? "" : volt >= 13.2f ? "LADUJE" : volt >= 12.4f ? "BEZ LADOWANIA" : "NISKIE!", 92, 122);
}

void drawIgnition() {
  header("ZAPLON / STUKI");
  float iat = F(0x04, 4);
  tile(4, 14, "IAT C", fmt(iat, 0), iat > 60 ? TFT_RED : iat > 45 ? TFT_ORANGE : TFT_WHITE);
  tile(124, 14, "KAT ZAPLONU", fmt(F(0x03, 4), 1));

  cv.setTextSize(1);
  cv.setTextColor(TFT_DARKGREY);
  cv.drawString("COFANIE ZAPLONU (stuki) cyl 1-4", 4, 52);
  for (int c = 0; c < 4; c++) {
    float r = fabsf(F(0x14, c + 1));
    if (!isnan(r) && r > peakRetard[c]) peakRetard[c] = r;
    uint16_t col = isnan(r) ? TFT_WHITE : r >= 3 ? TFT_RED : r >= 1 ? TFT_ORANGE : TFT_GREEN;
    int x = 4 + c * 59;
    cv.setTextSize(2);
    cv.setTextColor(col);
    cv.drawString(fmt(r, 1), x, 64);
    bar(x, 84, 54, 6, r / 6.0f, col);
    cv.setTextSize(1);
    cv.setTextColor(TFT_DARKGREY);
    cv.drawString("max " + fmt(peakRetard[c], 1), x, 96);
  }
  cv.drawString("A-przytrzymaj: zeruj max", 4, 122);
}

void drawTurbo() {
  header("TURBO");
  float spec = (F(0x73, 3) - baro) / 1000.0f;
  float act = (F(0x73, 4) - baro) / 1000.0f;
  tile(4, 14, "ZADANE bar", fmt(spec, 2), TFT_DARKGREY);
  tile(124, 14, "RZECZYW. bar", fmt(act, 2), boostColor(act));
  // underboost warning: actual much lower than requested under load
  bool under = !isnan(spec) && !isnan(act) && spec > 0.3f && act < spec - 0.2f;
  if (under) { cv.setTextSize(1); cv.setTextColor(TFT_RED); cv.drawString("NIEDOBOR!", 124, 2); }

  tile(4, 52, "N75 %", fmt(F(0x76, 3), 0));
  tile(84, 52, "LOAD %", fmt(F(0x73, 2), 0));
  tile(164, 52, "MAF g/s", fmt(F(0x03, 2), 0));
  tile(4, 92, "IAT C", fmt(F(0x04, 4), 0), TFT_WHITE, 2);
  tile(84, 92, "RPM", fmt(F(0x73, 1), 0), TFT_WHITE, 2);
  tile(164, 92, "MAX bar", fmt(peakBoost, 2), TFT_WHITE, 2);
}

void drawMixture() {
  header("MIESZANKA");
  tile(4, 14, "KM/H", fmt(F(0x05, 3), 0));
  tile(124, 14, "PRZEPUSTN. %", fmt(F(0x03, 3), 0));
  tile(4, 52, "LAMBDA %", fmt(F(0x01, 3), 1), TFT_WHITE, 2);
  tile(84, 52, "KOR.WOLNE %", fmt(F(0x20, 1), 1), fabsf(F(0x20, 1)) > 8 ? TFT_ORANGE : TFT_WHITE, 2);
  tile(164, 52, "KOR.CZESC %", fmt(F(0x20, 2), 1), fabsf(F(0x20, 2)) > 8 ? TFT_ORANGE : TFT_WHITE, 2);
  tile(4, 92, "WODA C", fmt(F(0x01, 2), 0), TFT_WHITE, 2);
  tile(84, 92, "AKU V", fmt(volt, 1), voltColor(volt), 2);
  tile(164, 92, "ZAPLON", fmt(F(0x03, 4), 1), TFT_WHITE, 2);
}

void draw() {
  cv.fillSprite(TFT_BLACK);
  switch (page) {
    case 0: drawMain(); break;
    case 1: drawIgnition(); break;
    case 2: drawTurbo(); break;
    case 3: drawMixture(); break;
  }
  cv.pushSprite(0, 0);
}

// ---------- buttons (read GPIO directly: A=37, B=39, active low) ----------
void handleButtons() {
  static bool lastA = true, lastB = true;
  static uint32_t downA = 0;
  static bool heldA = false;
  bool a = digitalRead(37), b = digitalRead(39);
  if (!a && lastA) { downA = millis(); heldA = false; }
  if (!a && !heldA && millis() - downA > 1000) {          // hold = reset peaks
    heldA = true;
    peakBoost = 0;
    for (auto &p : peakRetard) p = 0;
  }
  if (a && !lastA && !heldA && millis() - downA > 30) page = (page + 1) % PAGES;
  if (b && !lastB) page = (page + PAGES - 1) % PAGES;
  lastA = a; lastB = b;
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);   // landscape 240x135
  cv.createSprite(240, 135);
  pinMode(37, INPUT);
  pinMode(39, INPUT);
  Serial.begin(115200);
  for (auto &blk : bv) for (auto &f : blk) f = NAN;
  BT.begin("M5 Audi Gauge", true);
  BT.setPin("1234");
  connectAll();
}

void loop() {
  static uint32_t lastTry = 0;
  static uint8_t idx = 0, fails = 0;
  handleButtons();

  if (!ready) {                // auto-retry every 5 s
    if (millis() - lastTry > 5000) { connectAll(); lastTry = millis(); }
    delay(20);
    return;
  }
  if (!BT.connected()) { status("Rozlaczono", "ponawiam...", TFT_RED); ready = false; lastTry = millis(); return; }

  const uint8_t *list = polls[page];
  uint8_t blk = list[idx++ % pollLen[page]];
  bool ok = true;
  if (blk == 0x00) {
    float rv = elm("ATRV", 1000).toFloat();
    if (rv > 5 && rv < 20) volt = rv;
  } else {
    ok = readBlock(blk);
    if (ok && blk == 0x73) {
      float b = (F(0x73, 4) - baro) / 1000.0f;
      if (b > peakBoost) peakBoost = b;
    }
  }
  fails = ok ? 0 : fails + 1;
  if (fails >= 5) {            // session lost (e.g. ignition off) -> reopen
    if (!startVagSession()) { status("ECU nie odpowiada", "Zaplon ON?\nponawiam za 5 s", TFT_ORANGE); ready = false; lastTry = millis(); return; }
    fails = 0;
  }
  handleButtons();
  draw();
}
