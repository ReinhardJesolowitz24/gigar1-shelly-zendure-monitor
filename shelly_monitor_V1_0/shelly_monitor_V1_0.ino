/*
 * Shelly Pro 3EM + Zendure SolarFlow  – MONITOR & WATCHDOG  (read-only)
 * Board  : Arduino Giga R1 WiFi
 * Shield : Arduino Giga Display Shield (800x480, Querformat)
 *
 * NUR LESEN. Schreibt NICHTS an den Zendure -> stoert die Cloud-/HEMS-Regelung nicht.
 *
 * Anzeige:
 *   - Uhrzeit (vom Shelly, lokal/DST-korrekt) oben rechts + Heartbeat-Punkt
 *   - Netz gesamt (gross, gruen=Einspeisung / rot=Bezug)
 *   - pro Phase L1/L2/L3: Wirkleistung [W], Spannung [V], Strom [A]
 *   - Zendure (read-only): SoC, Abgabe, acStatus
 *   - Tagessaldo [kWh] seit 0:00 Uhr (+ = mehr Einspeisung, - = mehr Bezug),
 *     Reset automatisch um Mitternacht (aus echten Shelly-Energiezaehlern)
 *   - Waechter: roter Alarm bei Netz-Dumping / Tiefentladung / BMS-Fehler,
 *     mit Ereigniszaehlern. Plus Hardware-Watchdog (Selbstueberwachung).
 *
 * Vorzeichen (invertiert): negativ = Bezug, positiv = Einspeisung.
 *   (Je nach Montagerichtung der Stromzangen ggf. das *-1 anpassen.)
 *
 * EINRICHTUNG:
 *   arduino_secrets.example.h  ->  nach  arduino_secrets.h  kopieren
 *   und WLAN + Geraete-IPs eintragen. arduino_secrets.h ist .gitignore't.
 *
 * HAFTUNG: Anzeige-/Monitorwerkzeug, KEIN zertifiziertes Schutzgeraet.
 *   Der eigentliche Zellschutz ist das BMS des Akkus. Nutzung auf eigene Gefahr.
 *
 * MIT License - Copyright (c) 2026 Reinhard Jesolowitz
 */

#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include "Arduino_GigaDisplay_GFX.h"
#include "mbed.h"             // Hardware-Watchdog (STM32H7 IWDG)
#include "arduino_secrets.h"  // WLAN + Geraete-IPs (NICHT eingecheckt)

GigaDisplay_GFX display;

#define ROTATION   1
#define SCREEN_W   800
#define SCREEN_H   480

#define COL_BG       0x0841
#define COL_TITLE    0x2C9F
#define COL_EINSPEIS 0x07E0   // gruen  (Einspeisung, positiv)
#define COL_BEZUG    0xF800   // rot    (Bezug, negativ)
#define COL_VAL      0xFFFF   // weiss
#define COL_UNIT     0x8C51   // grau
#define COL_ZEN      0xFFE0   // gelb
#define COL_LINE     0x39E7

// ── Zugangsdaten / Geraete-IPs aus arduino_secrets.h ──────────────────────────
const char* WIFI_SSID     = SECRET_SSID;
const char* WIFI_PASSWORD = SECRET_PASS;

const char* SHELLY_HOST = SECRET_SHELLY_HOST;
const int   SHELLY_PORT = 80;
const char* SHELLY_PATH = "/rpc/EM.GetStatus?id=0";

const char* ZEN_HOST = SECRET_ZEN_HOST;
const int   ZEN_PORT = 80;

// ── Timing ───────────────────────────────────────────────────────────────────
const unsigned long POLL_MS = 1000;    // Shelly Leistung/Phasen
const unsigned long ZEN_MS  = 3000;    // Zendure (read-only)
const unsigned long SLOW_MS = 15000;   // Zeit + Energiezaehler
unsigned long lastPoll = 0, lastZen = 0, lastSlow = 0;
const unsigned long BEAT_MS = 1000;    // Heartbeat-Blinken (zeigt: Sketch laeuft)
unsigned long lastBeat = 0;
bool beatOn = false;
bool wdtActive = false;   // Watchdog erst nach WLAN-Erstverbindung aktiv

// ── Messwerte ────────────────────────────────────────────────────────────────
float gTotal = 0;
float pA=0,pB=0,pC=0, uA=0,uB=0,uC=0, iA=0,iB=0,iC=0;
int   zSoc=-1, zOut=0, zAc=-1;

// Zeit + Tagessaldo
String curTime = "--:--";
int    prevMod = -1;               // vorheriger Minute-im-Tag-Wert (Mitternacht-Erkennung)
double impWh=0, retWh=0;           // aktuelle Zaehlerstaende (Wh)
double impBase=0, retBase=0;       // Basis seit 0:00 Uhr
bool   baseSet=false;
double bezugKwh=0, einspKwh=0, saldoKwh=0;

// Waechter / Alarm: Akku entlaedt UND Netz speist ein = HEMS-Fehlbetrieb
const float         ALARM_OUT  = 200.0;    // W Zendure-Abgabe-Schwelle
const float         ALARM_EXP  = 300.0;    // W Netz-Einspeisung (Monitor positiv)
const unsigned long ALARM_HOLD = 30000;    // ms anhaltend, bevor Alarm ausloest
const unsigned long BLINK_MS   = 600;
const int           SOC_MIN_ALARM = 30;    // % Tiefentladungs-Warnschwelle (wie im Zendure)
unsigned long anomSince = 0, lastBlink = 0;
bool alarmActive = false, prevAlarm = false, blinkOn = false;
int  zFault = 0, zErr = 0;                  // BMS faultLevel / is_error
const char* alarmText = "";
bool dumpPrev = false, socPrev = false, faultPrev = false;   // Flankenerkennung
unsigned int cntDump = 0, cntSoc = 0, cntFault = 0;          // Ereigniszaehler (Reset nur bei Neustart)

// Differenzierung: gelbe Warnung (faultLevel) vs roter Kritisch-Alarm
bool warnActive = false, warnPrev = false;
const char* warnText = "";
unsigned int cntWarn = 0;
int cellMax = 0, cellMin = 9999, tempMax = 0;   // worst-case Zellwerte ueber alle Packs
const int CELL_MAX_CRIT = 360;    // 3.60 V Ueberspannung (Einheit 0.01 V)
const int CELL_MIN_CRIT = 260;    // 2.60 V Unterspannung
const int TEMP_MAX_CRIT = 3231;   // ~50 C (Einheit Kelvin*10)

// Spalten fuer die Phasen-Tabelle
const int CX_LBL = 25;
const int CX_L1  = 250;
const int CX_L2  = 440;
const int CX_L3  = 630;

// ───────────────────────────────────────────────────────────────────────────
void printAt(int x, int y, const char* t, uint8_t sz, uint16_t col) {
  display.setTextSize(sz);
  display.setTextColor(col, COL_BG);
  display.setTextWrap(false);
  display.setCursor(x, y);
  display.print(t);
}
void printCentered(const char* text, int y, uint8_t sz, uint16_t color) {
  int16_t tw = strlen(text) * 6 * sz;
  int16_t x  = (SCREEN_W - tw) / 2;
  if (x < 0) x = 0;
  printAt(x, y, text, sz, color);
}

// ───────────────────────────────────────────────────────────────────────────
void drawStaticLayout() {
  display.fillScreen(COL_BG);
  printAt(20, 8, "Shelly Pro 3EM - Monitor", 3, COL_TITLE);
  display.drawFastHLine(10, 42, SCREEN_W - 20, COL_LINE);

  printCentered("Netz gesamt [W]   (gruen=Einspeisung / rot=Bezug)", 50, 2, COL_UNIT);
  display.drawFastHLine(10, 150, SCREEN_W - 20, COL_LINE);

  printAt(CX_L1, 162, "L1", 2, COL_TITLE);
  printAt(CX_L2, 162, "L2", 2, COL_TITLE);
  printAt(CX_L3, 162, "L3", 2, COL_TITLE);
  printAt(CX_LBL, 192, "P [W]", 2, COL_UNIT);
  printAt(CX_LBL, 224, "U [V]", 2, COL_UNIT);
  printAt(CX_LBL, 256, "I [A]", 2, COL_UNIT);
  display.drawFastHLine(10, 292, SCREEN_W - 20, COL_LINE);

  display.drawFastHLine(10, 322, SCREEN_W - 20, COL_LINE);
  printCentered("Tagessaldo [kWh] seit 0:00   (+ Einspeisung / - Bezug)", 330, 2, COL_UNIT);
  display.drawFastHLine(10, 420, SCREEN_W - 20, COL_LINE);
}

void drawTime() {
  display.fillRect(630, 6, SCREEN_W - 630, 34, COL_BG);
  printAt(660, 12, curTime.c_str(), 3, COL_VAL);
}

void drawHeartbeat(bool on) {
  display.fillCircle(615, 22, 6, on ? COL_EINSPEIS : COL_BG);   // gruener Punkt blinkt (Sketch lebt)
}

void drawTotal(float p) {
  display.fillRect(0, 78, SCREEN_W, 68, COL_BG);
  char buf[20]; snprintf(buf, sizeof(buf), "%.0f", p);
  printCentered(buf, 82, 7, (p < 0) ? COL_BEZUG : COL_EINSPEIS);
}

void drawCell(int x, int y, float val, const char* fmt, bool colorBySign) {
  char buf[16]; snprintf(buf, sizeof(buf), fmt, val);
  uint16_t col = colorBySign ? ((val < 0) ? COL_BEZUG : COL_EINSPEIS) : COL_VAL;
  printAt(x, y, buf, 2, col);
}

void drawPhases() {
  display.fillRect(CX_L1 - 5, 185, SCREEN_W - CX_L1, 95, COL_BG);
  drawCell(CX_L1, 192, pA, "%.0f", true);  drawCell(CX_L2, 192, pB, "%.0f", true);  drawCell(CX_L3, 192, pC, "%.0f", true);
  drawCell(CX_L1, 224, uA, "%.1f", false); drawCell(CX_L2, 224, uB, "%.1f", false); drawCell(CX_L3, 224, uC, "%.1f", false);
  drawCell(CX_L1, 256, iA, "%.1f", false); drawCell(CX_L2, 256, iB, "%.1f", false); drawCell(CX_L3, 256, iC, "%.1f", false);
}

void drawZendure() {
  display.fillRect(0, 300, SCREEN_W, 20, COL_BG);
  char buf[70];
  if (zSoc < 0) snprintf(buf, sizeof(buf), "Zendure: (keine Antwort)");
  else          snprintf(buf, sizeof(buf), "Zendure (HEMS):  SoC %d%%   Abgabe %d W   acStatus %d", zSoc, zOut, zAc);
  printCentered(buf, 302, 2, COL_ZEN);
}

void drawSaldo() {
  display.fillRect(0, 358, SCREEN_W, 38, COL_BG);
  char buf[24]; snprintf(buf, sizeof(buf), "%+.3f kWh", saldoKwh);
  printCentered(buf, 360, 4, (saldoKwh < 0) ? COL_BEZUG : COL_EINSPEIS);
  display.fillRect(0, 398, SCREEN_W, 18, COL_BG);
  char c2[64]; snprintf(c2, sizeof(c2), "Bezug %.3f kWh    Einspeisung %.3f kWh", bezugKwh, einspKwh);
  printCentered(c2, 398, 2, COL_UNIT);
}

void drawStatus(const char* text, uint16_t color) {
  display.fillRect(0, 448, SCREEN_W, 30, COL_BG);
  printCentered(text, 452, 2, color);
}

void drawCounters() {
  display.fillRect(0, 424, SCREEN_W, 16, COL_BG);
  char buf[80];
  snprintf(buf, sizeof(buf), "Alarme seit Start:   BMS %u   Tief %u   Netz %u   Warn %u", cntFault, cntSoc, cntDump, cntWarn);
  uint16_t col = COL_UNIT;
  if (cntFault || cntSoc || cntDump) col = COL_BEZUG;   // es gab einen Rot-Alarm
  else if (cntWarn) col = COL_ZEN;                       // nur Warnungen
  printCentered(buf, 424, 2, col);
}

void drawAlarmOverlay(bool on) {
  uint16_t c = on ? COL_BEZUG : COL_BG;
  for (int k = 0; k < 6; k++) display.drawRect(k, k, SCREEN_W - 2 * k, SCREEN_H - 2 * k, c);
  display.fillRect(0, 440, SCREEN_W, 40, on ? COL_BEZUG : COL_BG);
  if (on) printCentered(alarmText, 450, 2, COL_VAL);
}

void evalAlarm(unsigned long now) {
  // 1) Netz-Dumping: Akku entlaedt UND Netz speist ein (30 s entprellt)
  bool dumping = (zOut > ALARM_OUT) && (gTotal > ALARM_EXP);
  if (dumping) { if (anomSince == 0) anomSince = now; } else anomSince = 0;
  bool dumpAlarm = (anomSince != 0) && (now - anomSince >= ALARM_HOLD);

  // 2) Tiefentladung: SoC unter Schwelle
  bool socAlarm = (zSoc >= 0) && (zSoc < SOC_MIN_ALARM);

  // 3) BMS kritisch: harter Fehler ODER unabhaengig gepruefte Zell-/Temp-Grenzwerte
  bool overV  = (cellMax > CELL_MAX_CRIT);
  bool underV = (cellMin > 0 && cellMin < CELL_MIN_CRIT);
  bool overT  = (tempMax > TEMP_MAX_CRIT);
  bool bmsCrit = (zErr != 0) || overV || underV || overT;

  // 4) GELB: faultLevel-Warnflag, aber kein harter/kritischer Fehler
  bool warn = (zFault != 0) && !bmsCrit;

  // Ereignisse zaehlen: nur bei Flanke inaktiv -> aktiv
  if (dumpAlarm && !dumpPrev)  cntDump++;
  if (socAlarm  && !socPrev)   cntSoc++;
  if (bmsCrit   && !faultPrev) cntFault++;
  if (warn      && !warnPrev)  cntWarn++;
  dumpPrev = dumpAlarm; socPrev = socAlarm; faultPrev = bmsCrit; warnPrev = warn;

  // Rot-Alarm (Prioritaet): BMS-kritisch > Tiefentladung > Dumping
  if (bmsCrit) {
    alarmActive = true;
    if      (zErr != 0) alarmText = "!! BMS HARD-FEHLER !!";
    else if (overV)     alarmText = "!! ZELLE UEBERSPANNUNG !!";
    else if (underV)    alarmText = "!! ZELLE UNTERSPANNUNG !!";
    else                alarmText = "!! BATTERIE UEBERTEMPERATUR !!";
  }
  else if (socAlarm)  { alarmActive = true; alarmText = "!! SoC UNTER 30% - TIEFENTLADUNG !!"; }
  else if (dumpAlarm) { alarmActive = true; alarmText = "!! AKKU SPEIST INS NETZ - HEMS-FEHLER !!"; }
  else                { alarmActive = false; }

  // Gelb-Warnung (Info, nicht blockierend)
  warnActive = warn;
  if (warn) { static char wbuf[40]; snprintf(wbuf, sizeof(wbuf), "BMS-Warnung (faultLevel=%d)", zFault); warnText = wbuf; }
}

void repaintAll() {
  drawStaticLayout(); drawTime(); drawTotal(gTotal); drawPhases(); drawZendure(); drawSaldo(); drawCounters();
}

// ───────────────────────────────────────────────────────────────────────────
void connectWiFi() {
  drawStatus("Verbinde mit WLAN...", COL_TITLE);
  unsigned long t0 = millis();        // Start dieses Verbindungsversuchs
  unsigned long tBegin = millis();    // Zeitpunkt des letzten WiFi.begin()
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (wdtActive) mbed::Watchdog::get_instance().kick();

    // Alle 12 s frischer Versuch (Modul aus haengendem Zustand holen)
    if (millis() - tBegin >= 12000) {
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      tBegin = millis();
    }

    // Im Betrieb: nach 90 s erfolglosem Reconnect Board neu starten lassen
    // (Watchdog NICHT mehr fuettern -> automatischer Reset, wie ein HW-Reset)
    if (wdtActive && (millis() - t0 >= 90000)) {
      drawStatus("Reconnect fehlgeschlagen -> Neustart", COL_BEZUG);
      while (true) { }   // Watchdog loest den Reboot aus
    }
  }
  drawStatus("WLAN verbunden", COL_EINSPEIS);
}

bool httpGetBody(const char* host, int port, const char* path, String& body) {
  WiFiClient client; client.setTimeout(5000);
  if (!client.connect(host, port)) return false;
  client.print(String("GET ") + path + " HTTP/1.0\r\nHost: " + host + "\r\nConnection: close\r\n\r\n");
  unsigned long t = millis();
  while (!client.available() && millis() - t < 5000) delay(10);
  if (!client.available()) { client.stop(); return false; }
  String s = client.readStringUntil('\n');
  if (s.indexOf("200") == -1) { client.stop(); return false; }
  while (client.connected()) { String l = client.readStringUntil('\n'); l.trim(); if (l.length() == 0) break; }
  body = ""; unsigned long t2 = millis();
  while (client.connected() || client.available()) {
    if (client.available()) body += (char)client.read();
    else if (millis() - t2 > 3000) break;
  }
  client.stop();
  return body.length() > 0;
}

int parseMinuteOfDay(const String& t) {
  int c = t.indexOf(':'); if (c < 1) return -1;
  int h = t.substring(0, c).toInt();
  int m = t.substring(c + 1).toInt();
  if (h < 0 || h > 23 || m < 0 || m > 59) return -1;
  return h * 60 + m;
}

bool fetchShelly() {
  String body; if (!httpGetBody(SHELLY_HOST, SHELLY_PORT, SHELLY_PATH, body)) return false;
  JsonDocument doc; if (deserializeJson(doc, body)) return false;
  gTotal = doc["total_act_power"].as<float>() * -1.0f;   // <0=Bezug, >0=Einspeisung
  pA = doc["a_act_power"].as<float>() * -1.0f;
  pB = doc["b_act_power"].as<float>() * -1.0f;
  pC = doc["c_act_power"].as<float>() * -1.0f;
  uA = doc["a_voltage"].as<float>(); uB = doc["b_voltage"].as<float>(); uC = doc["c_voltage"].as<float>();
  iA = doc["a_current"].as<float>(); iB = doc["b_current"].as<float>(); iC = doc["c_current"].as<float>();
  return true;
}

bool fetchZendure() {
  String body; if (!httpGetBody(ZEN_HOST, ZEN_PORT, "/properties/report", body)) return false;
  JsonDocument doc; if (deserializeJson(doc, body)) return false;
  JsonObject pr = doc["properties"]; if (pr.isNull()) return false;
  zSoc   = pr["electricLevel"] | -1;
  zOut   = pr["outputHomePower"] | 0;
  zAc    = pr["acStatus"] | -1;
  zFault = pr["faultLevel"] | 0;
  zErr   = pr["is_error"] | 0;

  // Pack-Werte (worst-case ueber alle Packs) fuer unabhaengigen Kritisch-Check
  cellMax = 0; cellMin = 9999; tempMax = 0;
  JsonArray pd = doc["packData"];
  if (!pd.isNull()) {
    for (JsonObject pk : pd) {
      int mv = pk["maxVol"] | 0;
      int nv = pk["minVol"] | 0;
      int mt = pk["maxTemp"] | 0;
      if (mv > cellMax) cellMax = mv;
      if (nv > 0 && nv < cellMin) cellMin = nv;
      if (mt > tempMax) tempMax = mt;
    }
  }
  return true;
}

// Zeit (Sys.GetStatus) + Energiezaehler (EMData.GetStatus), Tagessaldo
bool fetchSlow() {
  String body;
  if (httpGetBody(SHELLY_HOST, SHELLY_PORT, "/rpc/Sys.GetStatus", body)) {
    JsonDocument d; if (!deserializeJson(d, body)) {
      const char* tm = d["time"] | "";
      if (strlen(tm) >= 4) curTime = String(tm);
    }
  }
  if (!httpGetBody(SHELLY_HOST, SHELLY_PORT, "/rpc/EMData.GetStatus?id=0", body)) return false;
  JsonDocument d2; if (deserializeJson(d2, body)) return false;
  impWh = d2["total_act"]     | impWh;   // Bezug gesamt (Wh)
  retWh = d2["total_act_ret"] | retWh;   // Einspeisung gesamt (Wh)

  if (!baseSet) { impBase = impWh; retBase = retWh; baseSet = true; }

  // Mitternacht: Minute-im-Tag springt von ~23:xx (>1380) auf <01:00 (<60)
  int mod = parseMinuteOfDay(curTime);
  if (mod >= 0) {
    if (prevMod > 1380 && mod < 60) { impBase = impWh; retBase = retWh; }
    prevMod = mod;
  }

  bezugKwh = (impWh - impBase) / 1000.0;
  einspKwh = (retWh - retBase) / 1000.0;
  saldoKwh = einspKwh - bezugKwh;
  return true;
}

// ───────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  display.begin();
  display.setRotation(ROTATION);
  drawStaticLayout();
  drawTime();
  drawTotal(0);
  drawSaldo();
  drawCounters();
  connectWiFi();   // OHNE Watchdog -> beliebig viel Zeit fuer die Erstverbindung

  // Hardware-Watchdog ERST JETZT starten (nach erfolgreicher WLAN-Verbindung),
  // damit ein langsamer Verbindungsaufbau keinen Reboot-Loop ausloest.
  // Angefordert 40 s; HW-Maximum (~32 s) wird automatisch geklemmt.
  uint32_t toMs = 40000;
  uint32_t maxMs = mbed::Watchdog::get_instance().get_max_timeout();
  if (toMs > maxMs) toMs = maxMs;
  mbed::Watchdog::get_instance().start(toMs);
  wdtActive = true;
  Serial.print("Watchdog aktiv, Timeout (ms): "); Serial.println(toMs);
}

void loop() {
  mbed::Watchdog::get_instance().kick();   // Watchdog fuettern (jeden Durchlauf)
  unsigned long now = millis();

  if (now - lastBeat >= BEAT_MS) { lastBeat = now; beatOn = !beatOn; drawHeartbeat(beatOn); }

  if (now - lastPoll >= POLL_MS) {
    lastPoll = now;
    if (WiFi.status() != WL_CONNECTED) { drawStatus("WLAN getrennt...", COL_BEZUG); connectWiFi(); }
    else if (fetchShelly()) {
      drawTotal(gTotal); drawPhases();
      evalAlarm(now);
      drawCounters();
      if (!alarmActive) {
        if (warnActive) drawStatus(warnText, COL_ZEN);   // gelbe BMS-Warnung (nicht blockierend)
        else { char st[40]; snprintf(st, sizeof(st), "Monitor  |  Laufzeit %lus", now / 1000); drawStatus(st, COL_UNIT); }
      }
    } else drawStatus("Shelly-Fehler!", COL_BEZUG);
  }

  if (now - lastZen >= ZEN_MS) {
    lastZen = now;
    if (WiFi.status() == WL_CONNECTED) { fetchZendure(); drawZendure(); }
  }

  if (now - lastSlow >= SLOW_MS) {
    lastSlow = now;
    if (WiFi.status() == WL_CONNECTED) { if (fetchSlow()) { drawTime(); drawSaldo(); } }
  }

  // Waechter-Anzeige: blinkender roter Rahmen + Banner bei Fehlbetrieb
  if (alarmActive) {
    if (now - lastBlink >= BLINK_MS) { lastBlink = now; blinkOn = !blinkOn; drawAlarmOverlay(blinkOn); }
  } else if (prevAlarm) {
    repaintAll();   // Rahmen/Banner wieder entfernen, normale Anzeige zurueck
  }
  prevAlarm = alarmActive;
}
