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
#include <malloc.h>           // mallinfo() -> Heap-Diagnose (Leck/Fragmentierung sichtbar)
#include "arduino_secrets.h"  // WLAN + Geraete-IPs (NICHT eingecheckt)

GigaDisplay_GFX display;

#define ROTATION   1
#define FW_VERSION "giga-1.0"   // in /status gemeldet (Feld "fw"); "build" = Compile-Zeit erkennt veraltete Flashes
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
// Getaktete Abfrage-Sequenz: immer nur EINE Verbindung gleichzeitig, mit Pausen
// -> minimaler TCP-Churn, kein Ueberlappen. Schritt 0=Shelly EM, 1=Zendure, 2=Slow (Zeit+Energie).
// (Identisch zur ESP32-Variante, damit beide Repos moeglichst gleich bleiben.)
const uint8_t  POLL_SEQ[] = {0, 1, 0, 1, 2};   // 2x (EM, Zendure), dann 1x Slow(Sys+EMData) -> 5 Schritte/Zyklus
const int      SEQ_LEN    = sizeof(POLL_SEQ);
const unsigned long STEP_MS = 3000;            // 3 s Pause zwischen den Anfragen
int            seqIdx   = 0;
unsigned long  lastStep = 0;
char           httpBuf[4096];                  // fester HTTP-Antwort-Puffer (statt String -> kein Heap-Churn)
// Zendure haengt oefter (V2.0.0): nach Fehlversuch 5 min Pause -> entlastet WLAN-Stack (weniger Churn gegen toten Endpunkt)
// (Identisch zur ESP32-Variante, damit beide Repos moeglichst gleich bleiben.)
unsigned long znPauseUntil = 0;
const unsigned long ZEN_PAUSE_MS = 300000UL;   // 5 min
const unsigned long BEAT_MS = 1000;    // Heartbeat-Blinken (zeigt: Sketch laeuft)
unsigned long lastBeat = 0;
bool beatOn = false;
bool wdtActive = false;   // Watchdog erst nach WLAN-Erstverbindung aktiv
// --- Software-Fenster-Watchdog (effektiv ~9 min ueber den 32,76s-HW-Watchdog) ---
// Ein Ticker fuettert den HW-Watchdog NUR, solange der Haupt-Loop in den letzten
// WD_WINDOW_MS Fortschritt machte. Stuck >9min -> Ticker stoppt -> HW-Watchdog (~32s)
// resettet. So werden Router-Neustart/Ein-Aus/SW-Update (Minuten) abgefangen; nur
// echte lange Haenger -> Reboot. NUR fuer PASSIVEN Monitor ok, NICHT fuer einen Regler!
const unsigned long WD_WINDOW_MS = 540000;   // 9 min Toleranzfenster
volatile uint32_t   g_lastLoopMs = 0;        // Zeitstempel des letzten Loop-Fortschritts
mbed::Ticker        wdTicker;
void wdFeed() { if ((uint32_t)(millis() - g_lastLoopMs) < WD_WINDOW_MS) mbed::Watchdog::get_instance().kick(); }

// ── Messwerte ────────────────────────────────────────────────────────────────
float gTotal = 0;
float pA=0,pB=0,pC=0, uA=0,uB=0,uC=0, iA=0,iB=0,iC=0;
int   zSoc=-1, zOut=0, zAc=-1;

// Zeit + Tagessaldo
String curTime = "--:--";
String myIp = "0.0.0.0";            // eigene IP der GIGA (fuer Anzeige)
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
const int DEBOUNCE_N = 3;          // Alarme erst nach N aufeinanderfolgenden Messungen (gegen Reboot-Transienten)
int dbBms = 0, dbSoc = 0, dbBal = 0, dbWarn = 0;

// Geraete-Health: 0=ok, 1=haengt (TCP ok, API nein), 2=offline (kein TCP)
int shState = 0, znState = 0;
int shFailCount = 0;                 // aufeinanderfolgende Shelly-Fehlversuche (Entprellung)
const int SH_FAIL_N = 3;             // "Shelly-Fehler" erst nach 3 Fehlern in Folge (transiente Blips ignorieren)
unsigned int shOut = 0, znOut = 0;        // Ausfall-Episoden kumuliert (ok -> nicht-ok)
unsigned int pshOut = 0, pznOut = 0;      // Snapshot der letzten Mitternacht
int  zFault = 0, zErr = 0;                  // BMS faultLevel / is_error
const char* alarmText = "";
bool dumpPrev = false, socPrev = false, faultPrev = false;   // Flankenerkennung
unsigned int cntDump = 0, cntSoc = 0, cntFault = 0;          // Ereigniszaehler (Reset nur bei Neustart)

// Differenzierung: gelbe Warnung (faultLevel) vs roter Kritisch-Alarm
bool warnActive = false, warnPrev = false;
const char* warnText = "";
unsigned int cntWarn = 0;
bool balanceActive = false, balancePrev = false;
unsigned int cntBalance = 0;
int balSpread = 0, balSpreadMax = 0;   // Zellspreizung beim Balancing (mV): letzte + groesste je gesehene

// JSON-API (eigener HTTP-Server)
WiFiServer apiServer(80);
struct BalEvent { unsigned int id; char tm[8]; int soc; int cMax; int cMin; int spread; };
const int BAL_HIST = 64;
BalEvent balHist[BAL_HIST];
int balHead = 0;      // naechster Schreibindex im Ring
int balStored = 0;    // belegte Eintraege (max BAL_HIST)

// Tagessaldo-Historie (Ringpuffer der letzten 30 Tage)
unsigned long sysEpoch = 0;   // Unixzeit vom Shelly (Datums-Stempel)
struct DayRec { unsigned int id; unsigned long epoch; float saldo; float bezug; float einsp; unsigned int bms, tief, netz, bal, warn, shout, znout; };
const int DAY_HIST = 30;
DayRec dayHist[DAY_HIST];
int dayHead = 0, dayStored = 0;
unsigned int dayCount = 0;
unsigned int pbms = 0, ptief = 0, pnetz = 0, pbal = 0, pwarn = 0;   // Zaehler-Snapshot der letzten Mitternacht
int cellMax = 0, cellMin = 9999, tempMax = 0;   // worst-case Zellwerte ueber alle Packs
const int CELL_MAX_CRIT = 370;    // 3.70 V echte Ueberspannung (normale LFP-Vollladung ~3.5-3.65 V; Einheit 0.01 V)
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
// ── Dirty-Cache: nur neu zeichnen, wenn sich der Wert geaendert hat ───────────
// (weniger Framebuffer-Schreibzugriffe -> ruhigeres Bild, weniger Last; identisch zur ESP32-Variante)
long dcTotal = -999999L;
long dcP[3] = {-999999L,-999999L,-999999L}, dcU[3] = {-999999L,-999999L,-999999L}, dcI[3] = {-999999L,-999999L,-999999L};
int  dcSoc = -999, dcZOut = -999, dcZAc = -999, dcZnState = -999;
long dcSaldo = -999999L, dcBezug = -999999L, dcEinsp = -999999L;
char dcTime[8] = "";
long dcCntF=-1, dcCntS=-1, dcCntD=-1, dcCntB=-1, dcCntW=-1;
char dcStatus[64] = ""; uint16_t dcStatusCol = 0xFFFF;
void invalidateDirtyCache() {   // erzwingt Neuzeichnen aller dynamischen Felder (nach Vollbild-Repaint)
  dcTotal = -999999L;
  for (int k=0;k<3;k++){ dcP[k]=dcU[k]=dcI[k]=-999999L; }
  dcSoc=dcZOut=dcZAc=dcZnState=-999;
  dcSaldo=dcBezug=dcEinsp=-999999L;
  dcTime[0]=0;
  dcCntF=dcCntS=dcCntD=dcCntB=dcCntW=-1;
  dcStatus[0]=0;
}

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
  if (strcmp(curTime.c_str(), dcTime) == 0) return;     // unveraendert -> nicht neu zeichnen
  strncpy(dcTime, curTime.c_str(), sizeof(dcTime) - 1); dcTime[sizeof(dcTime) - 1] = 0;
  display.fillRect(630, 6, SCREEN_W - 630, 34, COL_BG);
  printAt(660, 12, curTime.c_str(), 3, COL_VAL);
}

void drawHeartbeat(bool on) {
  display.fillCircle(615, 22, 6, on ? COL_EINSPEIS : COL_BG);   // gruener Punkt blinkt (Sketch lebt)
}

void drawTotal(float p) {
  long v = lroundf(p);
  if (v == dcTotal) return;                              // gleicher gerundeter Wert -> nicht neu zeichnen
  dcTotal = v;
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
  long p[3] = {lroundf(pA), lroundf(pB), lroundf(pC)};
  long u[3] = {lroundf(uA*10), lroundf(uB*10), lroundf(uC*10)};   // 0,1-V-Aufloesung
  long c[3] = {lroundf(iA*10), lroundf(iB*10), lroundf(iC*10)};   // 0,1-A-Aufloesung
  bool same = true;
  for (int k=0;k<3;k++) if (p[k]!=dcP[k] || u[k]!=dcU[k] || c[k]!=dcI[k]) same = false;
  if (same) return;                                     // nichts geaendert -> nicht neu zeichnen
  for (int k=0;k<3;k++){ dcP[k]=p[k]; dcU[k]=u[k]; dcI[k]=c[k]; }
  display.fillRect(CX_L1 - 5, 185, SCREEN_W - CX_L1, 95, COL_BG);
  drawCell(CX_L1, 192, pA, "%.0f", true);  drawCell(CX_L2, 192, pB, "%.0f", true);  drawCell(CX_L3, 192, pC, "%.0f", true);
  drawCell(CX_L1, 224, uA, "%.1f", false); drawCell(CX_L2, 224, uB, "%.1f", false); drawCell(CX_L3, 224, uC, "%.1f", false);
  drawCell(CX_L1, 256, iA, "%.1f", false); drawCell(CX_L2, 256, iB, "%.1f", false); drawCell(CX_L3, 256, iC, "%.1f", false);
}

void drawZendure() {
  if (zSoc==dcSoc && zOut==dcZOut && zAc==dcZAc && znState==dcZnState) return;   // unveraendert
  dcSoc=zSoc; dcZOut=zOut; dcZAc=zAc; dcZnState=znState;
  display.fillRect(0, 300, SCREEN_W, 20, COL_BG);
  char buf[70];
  if      (znState == 2) snprintf(buf, sizeof(buf), "Zendure: OFFLINE (kein TCP)");
  else if (znState == 1) snprintf(buf, sizeof(buf), "Zendure: API haengt (Geraet laeuft)");
  else if (zSoc < 0)     snprintf(buf, sizeof(buf), "Zendure: (keine Antwort)");
  else                   snprintf(buf, sizeof(buf), "Zendure (HEMS):  SoC %d%%   Abgabe %d W   acStatus %d", zSoc, zOut, zAc);
  printCentered(buf, 302, 2, COL_ZEN);
}

void drawSaldo() {
  long s=lroundf(saldoKwh*1000), b=lroundf(bezugKwh*1000), e=lroundf(einspKwh*1000);
  if (s==dcSaldo && b==dcBezug && e==dcEinsp) return;   // unveraendert (mWh-Aufloesung)
  dcSaldo=s; dcBezug=b; dcEinsp=e;
  display.fillRect(0, 358, SCREEN_W, 38, COL_BG);
  char buf[24]; snprintf(buf, sizeof(buf), "%+.3f kWh", saldoKwh);
  printCentered(buf, 360, 4, (saldoKwh < 0) ? COL_BEZUG : COL_EINSPEIS);
  display.fillRect(0, 398, SCREEN_W, 18, COL_BG);
  char c2[64]; snprintf(c2, sizeof(c2), "Bezug %.3f kWh    Einspeisung %.3f kWh", bezugKwh, einspKwh);
  printCentered(c2, 398, 2, COL_UNIT);
}

void drawStatus(const char* text, uint16_t color) {
  if (strcmp(text, dcStatus) == 0 && color == dcStatusCol) return;   // unveraendert
  strncpy(dcStatus, text, sizeof(dcStatus) - 1); dcStatus[sizeof(dcStatus) - 1] = 0; dcStatusCol = color;
  display.fillRect(0, 448, SCREEN_W, 30, COL_BG);
  printCentered(text, 452, 2, color);
}

void drawCounters() {
  if ((long)cntFault==dcCntF && (long)cntSoc==dcCntS && (long)cntDump==dcCntD && (long)cntBalance==dcCntB && (long)cntWarn==dcCntW) return;  // unveraendert
  dcCntF=cntFault; dcCntS=cntSoc; dcCntD=cntDump; dcCntB=cntBalance; dcCntW=cntWarn;
  display.fillRect(0, 424, SCREEN_W, 16, COL_BG);
  char buf[96];
  snprintf(buf, sizeof(buf), "Ereignisse:  BMS %u  Tief %u  Netz %u  Bal %u  Warn %u", cntFault, cntSoc, cntDump, cntBalance, cntWarn);
  uint16_t col = COL_UNIT;
  if (cntFault || cntSoc || cntDump) col = COL_BEZUG;   // Rot-Alarm gab es
  else if (cntWarn)                  col = COL_ZEN;      // unbekannte Warnung (gelb)
  else if (cntBalance)               col = COL_TITLE;    // nur harmloses Balancing (blau)
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
  bool socRaw  = (zSoc >= 0) && (zSoc < SOC_MIN_ALARM);

  // 3) BMS kritisch: harter Fehler ODER unabhaengig gepruefte Zell-/Temp-Grenzwerte
  bool overV  = (cellMax > CELL_MAX_CRIT);
  bool underV = (cellMin > 0 && cellMin < CELL_MIN_CRIT);
  bool overT  = (tempMax > TEMP_MAX_CRIT);
  bool bmsRaw  = (zErr != 0) || overV || underV || overT;

  // 4) Nicht-kritische faultLevel-Flags differenzieren:
  //    faultLevel == 2 = bekanntes Zell-Balancing am Ladeschluss (harmlos -> blau, "Bal")
  //    sonstiges  != 0 = unbekanntes Flag -> gelbe Warnung ("Warn")
  bool balRaw  = (zFault == 2) && !bmsRaw;
  bool warnRaw = (zFault != 0) && (zFault != 2) && !bmsRaw;

  // Entprellung: erst nach DEBOUNCE_N aufeinanderfolgenden Messungen gueltig
  // (unterdrueckt Reboot-Transienten des Zendure, z.B. kurz SoC=0 / wirres faultLevel)
  dbBms  = bmsRaw  ? dbBms  + 1 : 0;
  dbSoc  = socRaw  ? dbSoc  + 1 : 0;
  dbBal  = balRaw  ? dbBal  + 1 : 0;
  dbWarn = warnRaw ? dbWarn + 1 : 0;
  bool bmsCrit   = dbBms  >= DEBOUNCE_N;
  bool socAlarm  = dbSoc  >= DEBOUNCE_N;
  bool isBalance = dbBal  >= DEBOUNCE_N;
  bool isWarn    = dbWarn >= DEBOUNCE_N;

  // Ereignisse zaehlen: nur bei Flanke inaktiv -> aktiv
  if (dumpAlarm  && !dumpPrev)    cntDump++;
  if (socAlarm   && !socPrev)     cntSoc++;
  if (bmsCrit    && !faultPrev)   cntFault++;
  if (isBalance  && !balancePrev) {
    cntBalance++;
    balSpread = (cellMax - cellMin) * 10;                 // mV (Einheit 0.01 V -> *10)
    if (balSpread > balSpreadMax) balSpreadMax = balSpread;
    char cb[80]; snprintf(cb, sizeof(cb), "CSV,BAL,%s,%d,%d,%d,%d", curTime.c_str(), zSoc, cellMax, cellMin, balSpread);
    Serial.println(cb);                                   // Langzeit-Log: Zeit, SoC, maxZelle, minZelle, Spreizung_mV
    // Ring-Puffer fuer die JSON-API
    balHist[balHead].id     = cntBalance;
    strncpy(balHist[balHead].tm, curTime.c_str(), sizeof(balHist[balHead].tm) - 1);
    balHist[balHead].tm[sizeof(balHist[balHead].tm) - 1] = 0;
    balHist[balHead].soc    = zSoc;
    balHist[balHead].cMax   = cellMax;
    balHist[balHead].cMin   = cellMin;
    balHist[balHead].spread = balSpread;
    balHead = (balHead + 1) % BAL_HIST;
    if (balStored < BAL_HIST) balStored++;
  }
  if (isWarn     && !warnPrev)    cntWarn++;
  dumpPrev = dumpAlarm; socPrev = socAlarm; faultPrev = bmsCrit; balancePrev = isBalance; warnPrev = isWarn;

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

  // Nicht-blockierende Info-Zeilen
  balanceActive = isBalance;
  warnActive    = isWarn;
  if (isWarn) { static char wbuf[40]; snprintf(wbuf, sizeof(wbuf), "BMS-Warnung (faultLevel=%d)", zFault); warnText = wbuf; }
}

void repaintAll() {
  invalidateDirtyCache();   // nach Vollbild-Loeschung muessen alle dynamischen Felder neu gezeichnet werden
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

    // Alle 12 s frischer Versuch (Modul aus haengendem Zustand holen)
    if (millis() - tBegin >= 12000) {
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      tBegin = millis();
    }

    // Im Betrieb: erst nach 10 min erfolglosem Reconnect Board neu starten lassen.
    // (Grosszuegig, damit ein Router-SW-Update / Netz-Ausfall von mehreren min KEINEN
    //  Neustart ausloest - ein Ein/Aus dauert je nach Router schon 3-4 min. Watchdog
    //  wird bis dahin weiter gefuettert; erst dann NICHT mehr -> Reset wie HW-Reset.)
    if (wdtActive && (millis() - t0 >= 600000)) {
      drawStatus("Reconnect fehlgeschlagen -> Neustart", COL_BEZUG);
      Serial.println("RECONNECT FEHLGESCHLAGEN -> NEUSTART (Watchdog)");   // im Serial-Log sichtbar
      while (true) { }   // Watchdog loest den Reboot aus
    }
  }
  myIp = WiFi.localIP().toString();
  drawStatus(("WLAN verbunden  " + myIp).c_str(), COL_EINSPEIS);
}

// Liest den HTTP-Body in einen FESTEN char-Puffer (kein String -> kein Heap-Churn).
bool httpGetBody(const char* host, int port, const char* path, char* out, size_t outSize) {
  if (WiFi.status() != WL_CONNECTED) return false;          // kein Connect auf totem Link (verhindert blockierenden Connect)
  WiFiClient client; client.setTimeout(5000);
  IPAddress ipAddr; bool gotIp = ipAddr.fromString(host);   // numerische IP direkt -> KEIN DNS/gethostbyname-Hang
  bool conOk = gotIp ? client.connect(ipAddr, port) : client.connect(host, port);
  if (!conOk) return false;
  char req[160];
  snprintf(req, sizeof(req), "GET %s HTTP/1.0\r\nHost: %s\r\nConnection: close\r\n\r\n", path, host);
  client.print(req);
  unsigned long t = millis();
  while (!client.available() && millis() - t < 5000) delay(10);
  if (!client.available()) { client.stop(); return false; }
  String s = client.readStringUntil('\n');                       // Statuszeile (1x pro Fetch)
  if (s.indexOf("200") == -1) { client.stop(); return false; }
  while (client.connected()) { String l = client.readStringUntil('\n'); l.trim(); if (l.length() == 0) break; }
  size_t n = 0; unsigned long t2 = millis();
  while (client.connected() || client.available()) {
    if (client.available()) { char ch = (char)client.read(); if (n < outSize - 1) out[n++] = ch; }  // Puffer voll -> Rest verwerfen
    else if (millis() - t2 > 3000) break;
  }
  out[n] = 0;
  client.stop();
  return n > 0;
}

int parseMinuteOfDay(const String& t) {
  int c = t.indexOf(':'); if (c < 1) return -1;
  int h = t.substring(0, c).toInt();
  int m = t.substring(c + 1).toInt();
  if (h < 0 || h > 23 || m < 0 || m > 59) return -1;
  return h * 60 + m;
}

bool fetchShelly() {
  if (!httpGetBody(SHELLY_HOST, SHELLY_PORT, SHELLY_PATH, httpBuf, sizeof(httpBuf))) return false;
  JsonDocument doc; if (deserializeJson(doc, httpBuf)) return false;
  gTotal = doc["total_act_power"].as<float>() * -1.0f;   // <0=Bezug, >0=Einspeisung
  pA = doc["a_act_power"].as<float>() * -1.0f;
  pB = doc["b_act_power"].as<float>() * -1.0f;
  pC = doc["c_act_power"].as<float>() * -1.0f;
  uA = doc["a_voltage"].as<float>(); uB = doc["b_voltage"].as<float>(); uC = doc["c_voltage"].as<float>();
  iA = doc["a_current"].as<float>(); iB = doc["b_current"].as<float>(); iC = doc["c_current"].as<float>();
  return true;
}

bool fetchZendure() {
  if (!httpGetBody(ZEN_HOST, ZEN_PORT, "/properties/report", httpBuf, sizeof(httpBuf))) return false;
  JsonDocument doc; if (deserializeJson(doc, httpBuf)) return false;
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
  if (httpGetBody(SHELLY_HOST, SHELLY_PORT, "/rpc/Sys.GetStatus", httpBuf, sizeof(httpBuf))) {
    JsonDocument d; if (!deserializeJson(d, httpBuf)) {
      const char* tm = d["time"] | "";
      if (strlen(tm) >= 4) curTime = String(tm);
      sysEpoch = d["unixtime"] | sysEpoch;
    }
  }
  if (!httpGetBody(SHELLY_HOST, SHELLY_PORT, "/rpc/EMData.GetStatus?id=0", httpBuf, sizeof(httpBuf))) return false;
  JsonDocument d2; if (deserializeJson(d2, httpBuf)) return false;
  impWh = d2["total_act"]     | impWh;   // Bezug gesamt (Wh)
  retWh = d2["total_act_ret"] | retWh;   // Einspeisung gesamt (Wh)

  if (!baseSet) { impBase = impWh; retBase = retWh; baseSet = true; }

  // Mitternacht: Minute-im-Tag springt von ~23:xx (>1380) auf <01:00 (<60)
  int mod = parseMinuteOfDay(curTime);
  if (mod >= 0) {
    if (prevMod > 1380 && mod < 60) {
      // Tag zu Ende: Tages-Saldo in den Ringpuffer sichern, DANN Basis zuruecksetzen
      dayCount++;
      dayHist[dayHead].id    = dayCount;
      dayHist[dayHead].epoch = sysEpoch;
      dayHist[dayHead].bezug = (impWh - impBase) / 1000.0;
      dayHist[dayHead].einsp = (retWh - retBase) / 1000.0;
      dayHist[dayHead].saldo = dayHist[dayHead].einsp - dayHist[dayHead].bezug;
      dayHist[dayHead].bms  = cntFault   - pbms;     // Ereignisse an DIESEM Tag (Delta)
      dayHist[dayHead].tief = cntSoc     - ptief;
      dayHist[dayHead].netz = cntDump    - pnetz;
      dayHist[dayHead].bal  = cntBalance - pbal;
      dayHist[dayHead].warn = cntWarn    - pwarn;
      dayHist[dayHead].shout = shOut - pshOut;
      dayHist[dayHead].znout = znOut - pznOut;
      pbms = cntFault; ptief = cntSoc; pnetz = cntDump; pbal = cntBalance; pwarn = cntWarn;
      pshOut = shOut; pznOut = znOut;
      dayHead = (dayHead + 1) % DAY_HIST;
      if (dayStored < DAY_HIST) dayStored++;
      impBase = impWh; retBase = retWh;
    }
    prevMod = mod;
  }

  bezugKwh = (impWh - impBase) / 1000.0;
  einspKwh = (retWh - retBase) / 1000.0;
  saldoKwh = einspKwh - bezugKwh;
  return true;
}

// ── JSON-API (HTTP-Server) ────────────────────────────────────────────────────
int classifyFail(const char* host) {     // 1 = haengt (TCP ok, API nein), 2 = offline (kein TCP)
  WiFiClient c; c.setTimeout(2000);
  bool tcp = c.connect(host, 80);
  c.stop();
  return tcp ? 1 : 2;
}
void updateHealth(int& state, unsigned int& outCnt, bool apiOk, const char* host) {
  int ns = apiOk ? 0 : classifyFail(host);
  if (state == 0 && ns != 0) outCnt++;   // neue Ausfall-Episode (ok -> nicht-ok)
  state = ns;
}

void sendJsonStatus(WiFiClient& c) {
  c.print("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\nAccess-Control-Allow-Origin: *\r\n\r\n");
  c.print("{\"fw\":\"" FW_VERSION "\",\"build\":\"" __DATE__ " " __TIME__ "\",\"uptime_s\":");      c.print(millis() / 1000);
  c.print(",\"time\":\"");        c.print(curTime); c.print("\"");
  c.print(",\"netz_w\":");        c.print(gTotal, 0);
  c.print(",\"saldo_kwh\":");     c.print(saldoKwh, 3);
  c.print(",\"bezug_kwh\":");     c.print(bezugKwh, 3);
  c.print(",\"einsp_kwh\":");     c.print(einspKwh, 3);
  c.print(",\"soc\":");           c.print(zSoc);
  c.print(",\"zout_w\":");        c.print(zOut);
  c.print(",\"faultlevel\":");    c.print(zFault);
  c.print(",\"is_error\":");      c.print(zErr);
  c.print(",\"spread_max_mv\":"); c.print(balSpreadMax);
  c.print(",\"cnt_bms\":");       c.print(cntFault);
  c.print(",\"cnt_tief\":");      c.print(cntSoc);
  c.print(",\"cnt_netz\":");      c.print(cntDump);
  c.print(",\"cnt_bal\":");       c.print(cntBalance);
  c.print(",\"cnt_warn\":");      c.print(cntWarn);
  c.print(",\"shelly_state\":");  c.print(shState);
  c.print(",\"zendure_state\":"); c.print(znState);
  c.print(",\"shelly_out\":");    c.print(shOut);
  c.print(",\"zendure_out\":");   c.print(znOut);
  struct mallinfo mi = mallinfo();
  c.print(",\"heap_used\":");     c.print(mi.uordblks);   // belegt (Bytes) -> steigt bei Leck
  c.print(",\"heap_free\":");     c.print(mi.fordblks);   // frei im Arena (Bytes) -> faellt bei Fragmentierung/Druck
  c.print("}");
}

void sendJsonBalance(WiFiClient& c) {
  c.print("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\nAccess-Control-Allow-Origin: *\r\n\r\n");
  c.print("{\"count\":"); c.print(cntBalance);
  c.print(",\"stored\":"); c.print(balStored);
  c.print(",\"events\":[");
  int start = (balStored < BAL_HIST) ? 0 : balHead;     // aeltester Eintrag zuerst
  for (int i = 0; i < balStored; i++) {
    int idx = (start + i) % BAL_HIST;
    if (i) c.print(",");
    c.print("{\"id\":");        c.print(balHist[idx].id);
    c.print(",\"time\":\"");    c.print(balHist[idx].tm); c.print("\"");
    c.print(",\"soc\":");       c.print(balHist[idx].soc);
    c.print(",\"cellmax\":");   c.print(balHist[idx].cMax);
    c.print(",\"cellmin\":");   c.print(balHist[idx].cMin);
    c.print(",\"spread_mv\":"); c.print(balHist[idx].spread);
    c.print("}");
  }
  c.print("]}");
}

void sendJsonDaily(WiFiClient& c) {
  c.print("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\nAccess-Control-Allow-Origin: *\r\n\r\n");
  c.print("{\"count\":"); c.print(dayCount);
  c.print(",\"stored\":"); c.print(dayStored);
  c.print(",\"days\":[");
  int start = (dayStored < DAY_HIST) ? 0 : dayHead;     // aeltester Tag zuerst
  for (int i = 0; i < dayStored; i++) {
    int idx = (start + i) % DAY_HIST;
    if (i) c.print(",");
    c.print("{\"id\":");          c.print(dayHist[idx].id);
    c.print(",\"epoch\":");       c.print(dayHist[idx].epoch);
    c.print(",\"saldo_kwh\":");   c.print(dayHist[idx].saldo, 3);
    c.print(",\"bezug_kwh\":");   c.print(dayHist[idx].bezug, 3);
    c.print(",\"einsp_kwh\":");   c.print(dayHist[idx].einsp, 3);
    c.print(",\"bms\":");   c.print(dayHist[idx].bms);
    c.print(",\"tief\":");  c.print(dayHist[idx].tief);
    c.print(",\"netz\":");  c.print(dayHist[idx].netz);
    c.print(",\"bal\":");   c.print(dayHist[idx].bal);
    c.print(",\"warn\":");  c.print(dayHist[idx].warn);
    c.print(",\"shout\":"); c.print(dayHist[idx].shout);
    c.print(",\"znout\":"); c.print(dayHist[idx].znout);
    c.print("}");
  }
  c.print("]}");
}

void handleApi() {
  WiFiClient c = apiServer.available();
  if (!c) return;
  c.setTimeout(800);
  unsigned long t = millis();
  while (!c.available() && millis() - t < 800) delay(1);
  String req = c.readStringUntil('\n');                 // "GET /pfad HTTP/1.1"
  while (c.connected() && c.available()) { String l = c.readStringUntil('\n'); if (l.length() <= 1) break; }
  if      (req.indexOf("/balance") >= 0) sendJsonBalance(c);
  else if (req.indexOf("/daily")   >= 0) sendJsonDaily(c);
  else if (req.indexOf("/status")  >= 0) sendJsonStatus(c);
  else {
    c.print("HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\n");
    c.print("ShellyMonitor API:  /status   /balance   /daily");
  }
  delay(2);
  c.stop();
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
  apiServer.begin();   // JSON-API starten (nach WLAN-Verbindung)

  // Hardware-Watchdog ERST JETZT starten (nach erfolgreicher WLAN-Verbindung),
  // damit ein langsamer Verbindungsaufbau keinen Reboot-Loop ausloest.
  // Angefordert 40 s; HW-Maximum (~32 s) wird automatisch geklemmt.
  uint32_t toMs = 40000;
  uint32_t maxMs = mbed::Watchdog::get_instance().get_max_timeout();
  if (toMs > maxMs) toMs = maxMs;
  mbed::Watchdog::get_instance().start(toMs);
  g_lastLoopMs = millis();              // Fenster-Watchdog: Startzeitpunkt
  wdTicker.attach(wdFeed, 8.0f);        // alle 8 s fuettern, solange Loop-Fortschritt < WD_WINDOW_MS
  wdtActive = true;
  Serial.print("Watchdog aktiv, HW-Timeout (ms): "); Serial.print(toMs);
  Serial.print("  | SW-Fenster (ms): "); Serial.println(WD_WINDOW_MS);
  Serial.println("CSV,BAL,time,soc,cellMax,cellMin,spread_mV");   // Kopfzeile fuers Balancing-Log
}

void loop() {
  g_lastLoopMs = millis();   // Fenster-Watchdog: Loop-Fortschritt melden (Ticker fuettert daraufhin den HW-Watchdog)
  handleApi();                             // JSON-API bedienen (nicht-blockierend)
  unsigned long now = millis();

  if (now - lastBeat >= BEAT_MS) { lastBeat = now; beatOn = !beatOn; drawHeartbeat(beatOn); }

  // Getaktete Abfrage: pro STEP_MS genau EINE Anfrage (nie ueberlappend -> wenig TCP-Churn)
  if (now - lastStep >= STEP_MS) {
    lastStep = now;
    if (WiFi.status() != WL_CONNECTED) { drawStatus("WLAN getrennt...", COL_BEZUG); connectWiFi(); }
    else {
      uint8_t step = POLL_SEQ[seqIdx];
      seqIdx = (seqIdx + 1) % SEQ_LEN;

      if (step == 0) {                         // Shelly Leistung/Phasen (Live)
        bool shOk = fetchShelly();
        if (shOk) {
          shFailCount = 0;
          updateHealth(shState, shOut, true, SHELLY_HOST);
          drawTotal(gTotal); drawPhases();
          evalAlarm(now);
          drawCounters();
          if (!alarmActive) {
            if (warnActive)         drawStatus(warnText, COL_ZEN);                           // gelb: unbekanntes Flag
            else if (balanceActive) { int sp=(cellMax-cellMin)*10; char bs[56]; snprintf(bs,sizeof(bs),"Zell-Balancing  Spreizung %d mV (Rekord %d)", sp, balSpreadMax); drawStatus(bs, COL_TITLE); } // blau: harmlos
            else { char st[64]; unsigned long up=now/1000; snprintf(st, sizeof(st), "IP %s   |   Laufzeit %lud %luh %lum", myIp.c_str(), up/86400, (up%86400)/3600, (up%3600)/60); drawStatus(st, COL_UNIT); }
          }
        } else {
          shFailCount++;
          if (shFailCount >= SH_FAIL_N) {       // erst nach N Fehlern in Folge als echter Ausfall werten
            updateHealth(shState, shOut, false, SHELLY_HOST);
            drawStatus("Shelly-Fehler!", COL_BEZUG);
          }
          // sonst: transienter Blip -> ignorieren, bisherige Anzeige bleibt stehen
        }
      }
      else if (step == 1) {                    // Zendure (read-only)
        if (millis() >= znPauseUntil) {        // nur abfragen, wenn keine Zendure-Pause laeuft
          bool zk = fetchZendure();
          updateHealth(znState, znOut, zk, ZEN_HOST);
          if (!zk) znPauseUntil = millis() + ZEN_PAUSE_MS;   // haengt/offline -> 5 min Pause
          else     znPauseUntil = 0;                          // antwortet wieder -> normal
          drawZendure();
        }
        // sonst: Zendure-Pause aktiv -> nicht abfragen (entlastet Loop/WLAN-Stack), Anzeige bleibt
      }
      else {                                   // Slow: Zeit + Energie/Tagessaldo (kWh kommt vom Shelly)
        if (fetchSlow()) { drawTime(); drawSaldo(); }
      }
    }
  }

  // Waechter-Anzeige: blinkender roter Rahmen + Banner bei Fehlbetrieb
  if (alarmActive) {
    if (now - lastBlink >= BLINK_MS) { lastBlink = now; blinkOn = !blinkOn; drawAlarmOverlay(blinkOn); }
  } else if (prevAlarm) {
    repaintAll();   // Rahmen/Banner wieder entfernen, normale Anzeige zurueck
  }
  prevAlarm = alarmActive;
}
