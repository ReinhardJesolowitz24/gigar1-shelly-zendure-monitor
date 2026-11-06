#pragma once
/*
 * dayrec.h  -  Persistenter Tages-Datensatz der Waechter (Ring 1000 Tage x 192 Byte)
 *
 * Datei ist in BEIDEN Repos IDENTISCH (ESP32 + GIGA). Plattformabhaengig ist nur das Speicher-Backend:
 *   ESP32: LittleFS auf der bisher ungenutzten "spiffs"-Partition von huge_app (896 KB), 20 Records je Datei
 *          (20 x 192 B = 3840 B < 1 Flash-Block -> ein Schreibzugriff schreibt hoechstens 1 Block um)
 *   GIGA : mbed KVStore (TDBStore auf QSPI-Partition 3), 1 Key je Record
 *
 * Ablauf: laufender Tag im RAM (DrAcc), jede Minute fortgeschrieben, stuendlich als Checkpoint gesichert.
 * Beim Tageswechsel wird er als DayRecV1 (mit CRC32) in Slot seq % 1000 geschrieben. Kopfzeiger = hoechste
 * gueltige seq (Boot-Scan) -> kein eigener Zeiger, der bei Stromausfall kaputtgehen kann.
 * Nach Neustart: Checkpoint vom selben Tag -> fortsetzen; von einem frueheren Tag -> nachtraeglich abschliessen.
 *
 * Der Sketch liefert die Live-Werte ueber drLive() und ruft die drOn...()-Haken auf. NUR LESEND gegenueber
 * Zendure/Regler/Broker: Regler-/Broker-Daten kommen aus den ohnehin laufenden Control-Abfragen.
 *
 * DATEN BLEIBEN NUR ERHALTEN, wenn ESP32 immer mit PartitionScheme=huge_app geflasht wird und auf dem GIGA
 * NIE QSPIFormat laeuft. Normales Flashen laesst beide Bereiche unberuehrt.
 */
#include <Arduino.h>
#include <ArduinoJson.h>
#include <stddef.h>

#ifndef DR_TEST_DAY_MIN
#define DR_TEST_DAY_MIN 0      // >0 = TESTMODUS: "Tag" = DR_TEST_DAY_MIN Minuten, eigener Speicherbereich (echter Ring bleibt unberuehrt)
#endif

static const uint16_t DR_MAGIC   = 0x5744;   // "WD"
static const uint8_t  DR_FMT     = 1;
static const uint16_t DR_CAP     = 1000;     // Ring: 1000 Tage
static const uint16_t DR_DAY_MIN = (DR_TEST_DAY_MIN > 0) ? DR_TEST_DAY_MIN : 1440;
static const int      DR_TARGET_W     = -25;   // Regler-Ziel 25 W Bezug (Waechter-Vorzeichen: negativ = Bezug)
static const int      DR_BAND_BEZUG_W = 100;   // Regelguete: Bezug > Ziel + 100 W, obwohl Entladen erlaubt
static const int      DR_BAND_EINSP_W = 50;    // Regelguete: Einspeisung > 50 W, obwohl Laden erlaubt
static const unsigned long DR_REGLER_FRESH_MS = 150000;   // Regler-Flags gelten bis 2,5 min nach dem letzten vollen Parse

enum : uint16_t {              // DayRecV1.flags
  DRF_INCOMPLETE = 1 << 0,     // weniger als (Tageslaenge - 10) Minuten mit Daten
  DRF_REBOOT     = 1 << 1,     // Waechter-Neustart an diesem Tag
  DRF_FWCHANGE   = 1 << 2,     // SW-Version am Tagesende != Version beim Tagesbeginn
  DRF_CATCHUP    = 1 << 3,     // beim Booten nachgeholt (Mitternacht verpasst)
  DRF_TIME       = 1 << 4,     // Uhrzeit unsicher (Tagesnummer lief rueckwaerts)
  DRF_NOBASE     = 1 << 5,     // Tagesbasis fehlte -> Energie nur ab Neustart
  DRF_CHECKPOINT = 1 << 6,     // Teile des Tages aus stuendlichem Checkpoint (bis ~1 h fehlen)
  DRF_TEST       = 1 << 7      // Testmodus (DR_TEST_DAY_MIN > 0)
};

// ── Record: fest 192 Byte, little-endian, gepackt (Layout siehe konzept_tagesdatensatz.md) ──
// "Keine Daten": u8 255, u16 0xFFFF (Minima), rssi 127/-128, drift 0x7FFF -> in JSON als null
#pragma pack(push, 1)
struct DayRecV1 {
  uint16_t magic;  uint8_t fmt;  uint8_t platform;
  uint32_t seq;    uint32_t epochClose;
  uint16_t year;   uint8_t month; uint8_t day;
  char     fw[12];
  uint16_t flags;  uint16_t minutesCovered;
  uint32_t bezugWh, einspWh, zenChgWh, zenDisWh;
  uint8_t  socMin, socMax;  uint16_t socMinMinute, socMaxMinute;  uint8_t socStart, socEnd;
  uint16_t netzBezugMaxW, netzEinspMaxW, zenOutMaxW, zenChgMaxW;
  uint16_t cellMax, cellMin, spreadMax, tempMax;
  uint16_t cnt[12];            // Tageszuwachs: regler_down, broker_down, control_down, mqtt_down, zen_stale,
                               //               shelly_out, zendure_out, bms, tief, netz, bal, warn
  uint16_t minZendureWeg, minShellyWeg, minControlWeg, minSocGrenze;
  uint8_t  bootsToday;  int8_t rssiMin, rssiMax, rssiAvg;
  uint32_t minFreeHeap; uint32_t bootCountTotal;
  int16_t  driftPpmX10; uint16_t zenHttpMaxMs;
  // Regelkreis-Block (ab Offset 120)
  uint8_t  reglerFw[3]; uint8_t reglerBoots; uint8_t reglerResetReason;
  uint8_t  brokerBoots; uint8_t brokerFw[3]; uint8_t brokerResetReason;
  uint16_t reglerHeapMinKb, reglerStackMin, brokerHeapMinKb, brokerStackMin;
  uint16_t reglerShellyFail, reglerShellyRetry, reglerRetreat;
  uint8_t  reglerMqttReconn, reglerMonTrip;
  uint16_t brokerEnforce;
  uint8_t  brokerMonitorTrip, brokerBypassTrip, brokerEthDown, brokerLastTripInv, brokerClientsMin, reserveA;
  uint32_t brokerMsg;
  uint16_t minReglerWeg, minBrokerWeg, minMqttDown, minZenStale;
  uint16_t minSaturated, minDischargeBlocked, minChargeBlocked;
  uint16_t minBezugBand, minEinspBand, netzAbwMittelW;
  uint8_t  reserve[10];
  uint32_t crc32;              // ueber Bytes 0..187
};
#pragma pack(pop)
static_assert(sizeof(DayRecV1) == 192, "DayRecV1 muss 192 Byte haben");
static_assert(offsetof(DayRecV1, reglerFw) == 120, "Offset Regelkreis-Block");
static_assert(offsetof(DayRecV1, crc32) == 188, "Offset CRC");

// Laufender Tag (= Checkpoint-Inhalt). Geraete-lokal, daher nicht gepackt.
struct DrAcc {
  uint16_t magic; uint8_t ver; uint8_t started;
  int32_t  dayNo;              // lokale Tagesnummer (Tage seit 1970) bzw. Test-Tagesnummer
  DayRecV1 rec;
  char     fwStart[12];
  uint8_t  baseOk, seenW, seenR, seenB;
  uint32_t eOffBezug, eOffEinsp;          // Energie vor Neustart (falls Tagesbasis nicht wiederhergestellt)
  double   eStartBezug, eStartEinsp;      // nur Testmodus: Energie zu Testtag-Beginn
  double   zenChgWh, zenDisWh;            // genaue Integrale
  int32_t  rssiSum;  uint16_t rssiN;
  uint32_t abwSum;   uint16_t abwN;
  uint32_t lastW[12];                     // letzter gesehener Stand der Waechter-Zaehler
  uint32_t lastR[6];                      // Regler: boot_count, shelly_fail, shelly_retry, retreat_count, mqtt_reconn, mon_trip_count
  uint32_t lastB[6];                      // Broker: boot_count, enforce_count, monitor_trip_count, bypass_trip_count, eth_down_count, msg_count
  uint32_t crc32;
};

// Live-Werte, die der Sketch in drLive() liefert
struct DrLive {
  long     today;              // lokale Tagesnummer (localDayNumber), -1 = keine Zeit
  int      minuteOfDay;        // lokale Minute im Tag, -1 = unbekannt
  uint32_t epoch;              // Unixzeit (vom Shelly)
  bool     baseFull;           // Tagesbasis gilt fuer den ganzen bisherigen Tag (wiederhergestellt oder um Mitternacht gesetzt)
  double   bezugWh, einspWh;   // seit Tagesbasis
  int      netzW;  bool shellyOk;
  int      soc;    bool zenOk;  int socGrenze;
  int      znState, shState, ctrlState;
  uint32_t cnt[12];            // aktuelle Waechter-Zaehler (Reihenfolge wie DayRecV1.cnt)
  int8_t   rssi;               // 0 = nicht verbunden
  uint32_t freeHeap;
  uint32_t uptimeS, uptimeMcuS;
};
void drLive(DrLive& l);        // im Sketch implementiert

// ── Speicher-Backend ─────────────────────────────────────────────────────────
#if defined(ARDUINO_ARCH_ESP32)
#include <LittleFS.h>
#define DR_PLATFORM 1
#define DR_STORE_NAME "littlefs"
static const uint16_t DR_PER_FILE = 20;
#if DR_TEST_DAY_MIN > 0
static const char* DR_DIR = "/drt";
#else
static const char* DR_DIR = "/dr";
#endif
static bool drFsOk = false;
static bool drStoreBegin() {
  drFsOk = LittleFS.begin(true, "/littlefs", 5, "spiffs");   // beim allerersten Mal wird die Partition formatiert (einige s)
  if (drFsOk && !LittleFS.exists(DR_DIR)) LittleFS.mkdir(DR_DIR);
  return drFsOk;
}
static void drPath(char* p, size_t n, uint16_t file) { snprintf(p, n, "%s/%02u.bin", DR_DIR, (unsigned)file); }
static bool drStoreRead(uint16_t slot, void* buf, size_t len) {
  if (!drFsOk) return false;
  char p[24]; drPath(p, sizeof(p), slot / DR_PER_FILE);
  if (!LittleFS.exists(p)) return false;
  File f = LittleFS.open(p, "r"); if (!f) return false;
  size_t off = (size_t)(slot % DR_PER_FILE) * len;
  bool ok = (f.size() >= off + len) && f.seek(off) && (f.read((uint8_t*)buf, len) == len);
  f.close();
  return ok;
}
static bool drStoreWrite(uint16_t slot, const void* buf, size_t len) {
  if (!drFsOk) return false;
  char p[24]; drPath(p, sizeof(p), slot / DR_PER_FILE);
  if (!LittleFS.exists(p)) { File c = LittleFS.open(p, "w"); if (!c) return false; c.close(); }
  File f = LittleFS.open(p, "r+"); if (!f) return false;
  size_t off = (size_t)(slot % DR_PER_FILE) * len;
  size_t sz = f.size();
  if (sz < off) {                                   // Luecke bis zum Slot mit Nullen fuellen
    static const uint8_t z[64] = {0};
    f.seek(sz);
    size_t miss = off - sz;
    while (miss) { size_t k = miss > sizeof(z) ? sizeof(z) : miss; f.write(z, k); miss -= k; }
  }
  bool ok = f.seek(off) && (f.write((const uint8_t*)buf, len) == len);
  f.close();
  return ok;
}
static bool drCurRead(void* buf, size_t len) {
  if (!drFsOk) return false;
  char p[24]; snprintf(p, sizeof(p), "%s/cur.bin", DR_DIR);
  if (!LittleFS.exists(p)) return false;
  File f = LittleFS.open(p, "r"); if (!f) return false;
  bool ok = (f.read((uint8_t*)buf, len) == len);
  f.close();
  return ok;
}
static bool drCurWrite(const void* buf, size_t len) {
  if (!drFsOk) return false;
  char p[24]; snprintf(p, sizeof(p), "%s/cur.bin", DR_DIR);
  File f = LittleFS.open(p, "w"); if (!f) return false;   // LittleFS: neuer Inhalt wird erst beim close() gueltig (atomar)
  bool ok = (f.write((const uint8_t*)buf, len) == len);
  f.close();
  return ok;
}
static void drScanOne(const void* p);
static void drScan() {                              // dateiweise lesen (50 Dateien statt 1000 Einzelzugriffe)
  static uint8_t buf[DR_PER_FILE * sizeof(DayRecV1)];
  for (uint16_t fi = 0; fi < DR_CAP / DR_PER_FILE; fi++) {
    char p[24]; drPath(p, sizeof(p), fi);
    if (!LittleFS.exists(p)) continue;
    File f = LittleFS.open(p, "r"); if (!f) continue;
    size_t n = f.read(buf, sizeof(buf));
    f.close();
    for (size_t o = 0; o + sizeof(DayRecV1) <= n; o += sizeof(DayRecV1)) drScanOne(buf + o);
  }
}
static void drStoreInfo(Print& p) {
  p.print(",\"fs_total\":"); p.print((unsigned long)LittleFS.totalBytes());
  p.print(",\"fs_used\":");  p.print((unsigned long)LittleFS.usedBytes());
}

#elif defined(ARDUINO_ARCH_MBED)
#include "kvstore_global_api.h"
#define DR_PLATFORM 2
#define DR_STORE_NAME "kvstore"
#if DR_TEST_DAY_MIN > 0
#define DR_KEYPFX "dt"
#else
#define DR_KEYPFX "dr"
#endif
static bool drStoreBegin() { return true; }        // KVStore nutzt der Sketch ohnehin schon (Saldo, Zaehler)
static bool drStoreRead(uint16_t slot, void* buf, size_t len) {
  char k[12]; snprintf(k, sizeof(k), DR_KEYPFX "%03u", (unsigned)slot);
  size_t a = 0;
  return kv_get(k, buf, len, &a) == 0 && a == len;
}
static bool drStoreWrite(uint16_t slot, const void* buf, size_t len) {
  char k[12]; snprintf(k, sizeof(k), DR_KEYPFX "%03u", (unsigned)slot);
  return kv_set(k, buf, len, 0) == 0;
}
static bool drCurRead(void* buf, size_t len)        { size_t a = 0; return kv_get(DR_KEYPFX "cur", buf, len, &a) == 0 && a == len; }
static bool drCurWrite(const void* buf, size_t len) { return kv_set(DR_KEYPFX "cur", buf, len, 0) == 0; }
static void drScanOne(const void* p);
static void drScan() {
  DayRecV1 r;
  for (uint16_t s = 0; s < DR_CAP; s++) if (drStoreRead(s, &r, sizeof(r))) drScanOne(&r);
}
static void drStoreInfo(Print& p) { p.print(",\"fs_total\":null,\"fs_used\":null"); }   // Global-API liefert keine Gesamtgroesse

#else
#error "dayrec.h: Plattform nicht unterstuetzt (nur ESP32 / mbed GIGA)"
#endif

// ── Zustand ──────────────────────────────────────────────────────────────────
static DrAcc    drA;                      // laufender Tag
static DrAcc    drCp;                     // beim Boot geladener Checkpoint
static bool     drCpOk = false;
static bool     drStoreOk = false;
static uint32_t drNewest = 0, drNextSeq = 1, drWriteErr = 0;
static uint16_t drStored = 0;
static uint16_t drMinTicks = 0;           // Minuten seit letztem Checkpoint
static const char* drFwStr = "";
static uint32_t drBoots = 0;
static unsigned long drReglerMs = 0;      // letzter voller Regler-Parse (millis)
static bool     drRSat = false, drRDis = false, drRChg = false;
static unsigned long drLastZenMs = 0;     // letzte erfolgreiche Zendure-Abfrage (fuer Energie-Integral)

// ── Hilfen ───────────────────────────────────────────────────────────────────
static uint32_t drCrc(const void* data, size_t n) {
  const uint8_t* p = (const uint8_t*)data; uint32_t c = 0xFFFFFFFFUL;
  while (n--) { c ^= *p++; for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320UL & (0UL - (c & 1UL))); }
  return ~c;
}
static inline uint16_t drAdd16(uint16_t a, uint32_t b) { uint32_t s = (uint32_t)a + b; return s > 0xFFFF ? 0xFFFF : (uint16_t)s; }
static inline uint8_t  drAdd8 (uint8_t a,  uint32_t b) { uint32_t s = (uint32_t)a + b; return s > 0xFF ? 0xFF : (uint8_t)s; }
static inline uint32_t drDelta(uint32_t cur, uint32_t last) { return cur >= last ? cur - last : cur; }   // Zaehler-Reset -> neuer Stand zaehlt

static void drCivil(long z, uint16_t& y, uint8_t& m, uint8_t& d) {   // Tage seit 1970-01-01 -> Datum (H. Hinnant)
  z += 719468;
  long era = (z >= 0 ? z : z - 146096) / 146097;
  unsigned long doe = (unsigned long)(z - era * 146097);
  unsigned long yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  unsigned long doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  unsigned long mp  = (5 * doy + 2) / 153;
  d = (uint8_t)(doy - (153 * mp + 2) / 5 + 1);
  m = (uint8_t)(mp < 10 ? mp + 3 : mp - 9);
  y = (uint16_t)((long)yoe + era * 400 + (m <= 2 ? 1 : 0));
}
static void drParseVer(const char* s, uint8_t* v) {   // "regler-1.24.0" -> {1,24,0}
  if (!s) return;
  const char* q = strrchr(s, '-'); q = q ? q + 1 : s;
  unsigned a = 0, b = 0, c = 0;
  if (sscanf(q, "%u.%u.%u", &a, &b, &c) >= 2) { v[0] = (uint8_t)a; v[1] = (uint8_t)b; v[2] = (uint8_t)c; }
}
static long drDayOf(const DrLive& l) {
#if DR_TEST_DAY_MIN > 0
  return l.epoch ? (long)(l.epoch / 60UL / DR_TEST_DAY_MIN) : -1;
#else
  return l.today;
#endif
}
static int drMinuteOf(const DrLive& l) {
#if DR_TEST_DAY_MIN > 0
  return l.epoch ? (int)((l.epoch / 60UL) % DR_TEST_DAY_MIN) : -1;
#else
  return l.minuteOfDay;
#endif
}

static void drScanOne(const void* p) {
  DayRecV1 r; memcpy(&r, p, sizeof(r));
  if (r.magic != DR_MAGIC || r.fmt != DR_FMT || r.seq == 0) return;
  if (r.crc32 != drCrc(&r, offsetof(DayRecV1, crc32))) return;
  if (drStored < DR_CAP) drStored++;
  if (r.seq > drNewest) drNewest = r.seq;
}
static bool drReadSeq(uint32_t seq, DayRecV1& r) {
  if (!drStoreRead(seq % DR_CAP, &r, sizeof(r))) return false;
  return r.magic == DR_MAGIC && r.fmt == DR_FMT && r.seq == seq && r.crc32 == drCrc(&r, offsetof(DayRecV1, crc32));
}

static void drInitRec(DayRecV1& r) {
  memset(&r, 0, sizeof(r));
  r.socMin = r.socMax = r.socStart = r.socEnd = 255;
  r.socMinMinute = r.socMaxMinute = 0xFFFF;
  r.cellMin = 0xFFFF;
  r.rssiMin = 127; r.rssiMax = -128; r.rssiAvg = 127;
  r.minFreeHeap = 0xFFFFFFFFUL;
  r.driftPpmX10 = 0x7FFF;
  r.reglerResetReason = r.brokerResetReason = 255;
  r.reglerHeapMinKb = r.reglerStackMin = r.brokerHeapMinKb = r.brokerStackMin = 0xFFFF;
  r.brokerClientsMin = 255;
  r.netzAbwMittelW = 0xFFFF;
}

static void drSaveCp() {
  drA.crc32 = drCrc(&drA, offsetof(DrAcc, crc32));
  if (!drCurWrite(&drA, sizeof(drA))) drWriteErr++;
}

// Fertigen Record aus dem laufenden Tag bilden (ohne Schreiben)
static void drFinalize(const DrAcc& a, DayRecV1& r, long ymdDay, uint32_t seq) {
  r = a.rec;
  r.magic = DR_MAGIC; r.fmt = DR_FMT; r.platform = DR_PLATFORM; r.seq = seq;
  if (ymdDay >= 0) { uint16_t y; uint8_t m, d; drCivil(ymdDay, y, m, d); r.year = y; r.month = m; r.day = d; }
  r.zenChgWh = (uint32_t)(a.zenChgWh + 0.5);
  r.zenDisWh = (uint32_t)(a.zenDisWh + 0.5);
  if (a.rssiN) r.rssiAvg = (int8_t)(a.rssiSum / (int32_t)a.rssiN);
  if (a.abwN)  r.netzAbwMittelW = (uint16_t)(a.abwSum / a.abwN);
  if ((uint32_t)r.minutesCovered + 10 < DR_DAY_MIN) r.flags |= DRF_INCOMPLETE;
  if (strncmp(a.fwStart, r.fw, sizeof(r.fw)) != 0) r.flags |= DRF_FWCHANGE;
  if (!a.baseOk) r.flags |= DRF_NOBASE;
  r.crc32 = drCrc(&r, offsetof(DayRecV1, crc32));
}
static void drClose(const DrAcc& a, uint16_t extraFlags, long ymdDay) {
  DayRecV1 r;
  drFinalize(a, r, ymdDay, drNextSeq);
  if (extraFlags) { r.flags |= extraFlags; r.crc32 = drCrc(&r, offsetof(DayRecV1, crc32)); }
  if (drStoreWrite((uint16_t)(r.seq % DR_CAP), &r, sizeof(r))) { drNewest = r.seq; if (drStored < DR_CAP) drStored++; }
  else drWriteErr++;
  drNextSeq++;                                      // auch bei Fehler weiterzaehlen -> seq bleibt eindeutig
}

// Neuen Tag beginnen. prev = Vortag/Checkpoint (Zaehlerstaende laufen weiter), bootStart = nach Neustart
static void drStartNew(const DrLive& l, const DrAcc* prev, bool bootStart) {
  static DrAcc n;
  memset(&n, 0, sizeof(n));
  n.magic = DR_MAGIC; n.ver = 1; n.started = 1;
  n.dayNo = (int32_t)drDayOf(l);
  drInitRec(n.rec);
  strncpy(n.fwStart, drFwStr, sizeof(n.fwStart) - 1);
  strncpy(n.rec.fw,  drFwStr, sizeof(n.rec.fw) - 1);
  if (prev) {
    memcpy(n.lastW, prev->lastW, sizeof(n.lastW)); n.seenW = prev->seenW;
    memcpy(n.lastR, prev->lastR, sizeof(n.lastR)); n.seenR = prev->seenR;
    memcpy(n.lastB, prev->lastB, sizeof(n.lastB)); n.seenB = prev->seenB;
  }
  if (bootStart) {
    n.rec.bootsToday = 1; n.rec.flags |= DRF_REBOOT;
    n.baseOk = l.baseFull ? 1 : 0;
    for (int i = 5; i < 12; i++) n.lastW[i] = 0;    // nicht-persistente Zaehler beginnen nach Boot bei 0
  } else {
    n.baseOk = 1;                                   // Start um Mitternacht: Tagesbasis wird direkt danach neu gesetzt
  }
#if DR_TEST_DAY_MIN > 0
  n.baseOk = 1; n.eStartBezug = l.bezugWh; n.eStartEinsp = l.einspWh;
  n.rec.flags |= DRF_TEST;
#endif
  drA = n;
  drMinTicks = 0;
  drSaveCp();
}
static void drResume(const DrLive& l) {             // Checkpoint vom selben Tag fortsetzen
  drA = drCp;
  drA.rec.bootsToday = drAdd8(drA.rec.bootsToday, 1);
  drA.rec.flags |= DRF_REBOOT | DRF_CHECKPOINT;
  if (l.baseFull) { drA.eOffBezug = 0; drA.eOffEinsp = 0; }   // Live-Energie deckt den ganzen Tag ab
  else { drA.eOffBezug = drA.rec.bezugWh; drA.eOffEinsp = drA.rec.einspWh; }   // sonst Stand vor Neustart als Sockel
  for (int i = 5; i < 12; i++) drA.lastW[i] = 0;
  drMinTicks = 0;
  drSaveCp();
}

// Zaehler, Energie, Versionen nachfuehren; tick = Minuten-Takt (Minuten-Zaehler, Min/Max, Mittelwerte)
static void drUpdate(const DrLive& l, bool tick) {
  DayRecV1& r = drA.rec;
  for (int i = 0; i < 12; i++) {
    if (drA.seenW) r.cnt[i] = drAdd16(r.cnt[i], drDelta(l.cnt[i], drA.lastW[i]));
    drA.lastW[i] = l.cnt[i];
  }
  drA.seenW = 1;
  double b = l.bezugWh, e = l.einspWh;
#if DR_TEST_DAY_MIN > 0
  b -= drA.eStartBezug; e -= drA.eStartEinsp;
#endif
  if (b < 0) b = 0;
  if (e < 0) e = 0;
  r.bezugWh = drA.eOffBezug + (uint32_t)(b + 0.5);
  r.einspWh = drA.eOffEinsp + (uint32_t)(e + 0.5);
  memset(r.fw, 0, sizeof(r.fw)); strncpy(r.fw, drFwStr, sizeof(r.fw) - 1);
  r.bootCountTotal = drBoots;
  if (l.epoch) r.epochClose = l.epoch;
  if (l.uptimeS >= 21600) {   // erst ab 6 h: Shelly-Zeit hat 1-s-Aufloesung -> nach 6 h < +-50 ppm Rauschen
    double ppm10 = ((double)l.uptimeMcuS - (double)l.uptimeS) * 1e7 / (double)l.uptimeS;
    if (ppm10 > 32000) ppm10 = 32000;
    if (ppm10 < -32000) ppm10 = -32000;
    r.driftPpmX10 = (int16_t)ppm10;
  }
  if (!tick) return;

  if (r.minutesCovered < 0xFFFF) r.minutesCovered++;
  if (l.znState != 0)  r.minZendureWeg = drAdd16(r.minZendureWeg, 1);
  if (l.shState != 0)  r.minShellyWeg  = drAdd16(r.minShellyWeg, 1);
  if (l.ctrlState >= 1) r.minControlWeg = drAdd16(r.minControlWeg, 1);
  if (l.ctrlState == 1 || l.ctrlState == 3) r.minReglerWeg = drAdd16(r.minReglerWeg, 1);
  if (l.ctrlState == 2 || l.ctrlState == 3) r.minBrokerWeg = drAdd16(r.minBrokerWeg, 1);
  if (l.ctrlState == 4) r.minMqttDown = drAdd16(r.minMqttDown, 1);
  if (l.ctrlState == 5) r.minZenStale = drAdd16(r.minZenStale, 1);

  int mod = drMinuteOf(l);
  if (l.zenOk && l.soc >= 0 && l.soc <= 100) {
    uint8_t s = (uint8_t)l.soc;
    if (r.socMin == 255 || s < r.socMin) { r.socMin = s; r.socMinMinute = (mod >= 0) ? (uint16_t)mod : 0xFFFF; }
    if (r.socMax == 255 || s > r.socMax) { r.socMax = s; r.socMaxMinute = (mod >= 0) ? (uint16_t)mod : 0xFFFF; }
    if (r.socStart == 255) r.socStart = s;
    r.socEnd = s;
    if (l.soc <= l.socGrenze) r.minSocGrenze = drAdd16(r.minSocGrenze, 1);
  }
  if (l.rssi < 0) {
    if (l.rssi < r.rssiMin) r.rssiMin = l.rssi;
    if (l.rssi > r.rssiMax) r.rssiMax = l.rssi;
    drA.rssiSum += l.rssi; if (drA.rssiN < 0xFFFF) drA.rssiN++;
  }
  if (l.freeHeap && l.freeHeap < r.minFreeHeap) r.minFreeHeap = l.freeHeap;

  if (drReglerMs && millis() - drReglerMs < DR_REGLER_FRESH_MS) {   // nur mit frischen Regler-Flags werten
    if (drRSat) r.minSaturated        = drAdd16(r.minSaturated, 1);
    if (drRDis) r.minDischargeBlocked = drAdd16(r.minDischargeBlocked, 1);
    if (drRChg) r.minChargeBlocked    = drAdd16(r.minChargeBlocked, 1);
    if (l.shellyOk) {                                               // Regelguete: Netz vom Shelly, unabhaengig vom Regler gemessen
      if (l.netzW < DR_TARGET_W - DR_BAND_BEZUG_W && !drRDis && !drRSat) r.minBezugBand = drAdd16(r.minBezugBand, 1);
      if (l.netzW > DR_BAND_EINSP_W && !drRChg && !drRSat)               r.minEinspBand = drAdd16(r.minEinspBand, 1);
      drA.abwSum += (uint32_t)abs(l.netzW - DR_TARGET_W); if (drA.abwN < 0xFFFF) drA.abwN++;
    }
  }
}

// Erster gueltiger Zeitpunkt nach dem Boot: fortsetzen, nachholen oder neu beginnen
static void drStartup(const DrLive& l, long day) {
  if (drCpOk && drCp.dayNo == day) {
    drResume(l);
  } else {
    if (drCpOk && drCp.dayNo < day) drClose(drCp, DRF_CATCHUP | DRF_CHECKPOINT, (DR_TEST_DAY_MIN > 0) ? l.today : drCp.dayNo);
    drStartNew(l, drCpOk ? &drCp : nullptr, true);
    if (drCpOk && drCp.dayNo > day) drA.rec.flags |= DRF_TIME;
  }
  drCpOk = false;
}

// ── Haken fuer den Sketch ────────────────────────────────────────────────────
// setup(): Speicher einbinden, Ring scannen, Checkpoint laden (vor dem Watchdog-Start aufrufen)
void drBegin(const char* fw, uint32_t bootCountTotal) {
  drFwStr = fw; drBoots = bootCountTotal;
  memset(&drA, 0, sizeof(drA));
  drStoreOk = drStoreBegin();
  if (drStoreOk) {
    drScan();
    if (drCurRead(&drCp, sizeof(drCp)) && drCp.magic == DR_MAGIC && drCp.ver == 1 && drCp.started &&
        drCp.crc32 == drCrc(&drCp, offsetof(DrAcc, crc32))) drCpOk = true;
  }
  drNextSeq = drNewest + 1;
}
// fetchSlow(): nach dem Setzen der Tagesbasis, VOR dem Mitternachts-Reset der Zaehler aufrufen
void drCheckDay(const DrLive& l) {
  long day = drDayOf(l);
  if (day < 0 || l.today < 0) return;
  if (!drA.started) { drStartup(l, day); return; }
  if (day > drA.dayNo) {
    drUpdate(l, false);                             // letzter Stand des alten Tages
    drClose(drA, 0, (DR_TEST_DAY_MIN > 0) ? l.today : drA.dayNo);
    drStartNew(l, &drA, false);
  } else if (day < drA.dayNo) {
    drA.rec.flags |= DRF_TIME;                      // Uhr lief rueckwaerts -> nicht schliessen, nur markieren
  }
}
// Mitternachts-Block im Sketch: nach dem Nullen der nicht-persistenten Zaehler aufrufen
void drCountersReset() { for (int i = 5; i < 12; i++) drA.lastW[i] = 0; }
// loop(): einmal pro Minute
void drMinute() {
  if (!drA.started) return;
  DrLive l; drLive(l);
  long day = drDayOf(l);
  if (day >= 0 && day != drA.dayNo) return;          // Tageswechsel steht an -> drCheckDay schliesst ab
  drUpdate(l, true);
  if (++drMinTicks >= 60) { drMinTicks = 0; drSaveCp(); }   // stuendlicher Checkpoint
}
// nach jeder erfolgreichen Shelly-Abfrage
void drOnShelly(int netzW) {
  if (!drA.started) return;
  DayRecV1& r = drA.rec;
  if (netzW < 0 && (uint32_t)(-netzW) > r.netzBezugMaxW) r.netzBezugMaxW = (uint16_t)((-netzW) > 0xFFFF ? 0xFFFF : -netzW);
  if (netzW > 0 && (uint32_t)netzW > r.netzEinspMaxW)    r.netzEinspMaxW = (uint16_t)(netzW > 0xFFFF ? 0xFFFF : netzW);
}
// nach jeder Zendure-Abfrage (ok = Antwort erhalten, ms = Dauer)
void drOnZendure(bool ok, unsigned long ms, int outW, int chgW, int cMax, int cMin, int tMax) {
  unsigned long now = millis();
  if (!ok) { drLastZenMs = 0; return; }
  if (!drA.started) { drLastZenMs = now; return; }
  DayRecV1& r = drA.rec;
  if (drLastZenMs && now - drLastZenMs <= 30000UL) {   // Energie-Integral nur ueber lueckenlose Abfragen
    double h = (double)(now - drLastZenMs) / 3600000.0;
    if (outW > 0) drA.zenDisWh += outW * h;
    if (chgW > 0) drA.zenChgWh += chgW * h;
  }
  drLastZenMs = now;
  if (outW > 0 && (uint32_t)outW > r.zenOutMaxW) r.zenOutMaxW = (uint16_t)(outW > 0xFFFF ? 0xFFFF : outW);
  if (chgW > 0 && (uint32_t)chgW > r.zenChgMaxW) r.zenChgMaxW = (uint16_t)(chgW > 0xFFFF ? 0xFFFF : chgW);
  if (ms > r.zenHttpMaxMs) r.zenHttpMaxMs = (uint16_t)(ms > 0xFFFF ? 0xFFFF : ms);
  if (cMax > 0 && cMax > r.cellMax) r.cellMax = (uint16_t)cMax;                      // packData im Standby leer -> keine Werte
  if (cMin > 0 && cMin < 9999 && cMin < r.cellMin) r.cellMin = (uint16_t)cMin;
  if (cMax > 0 && cMin > 0 && cMin < 9999 && cMax >= cMin) {
    uint16_t sp = (uint16_t)((cMax - cMin) * 10);                                      // mV (Einheit 0,01 V)
    if (sp > r.spreadMax) r.spreadMax = sp;
  }
  if (tMax > 0 && tMax > r.tempMax) r.tempMax = (uint16_t)tMax;
}
// voller Regler-/Broker-/status-Parse (1x pro Minute aus der Control-Abfrage)
void drOnRegler(JsonDocument& d) {
  drReglerMs = millis();
  drRSat = d["saturated"] | false;
  drRDis = d["discharge_blocked"] | false;
  drRChg = d["charge_blocked"] | false;
  if (!drA.started) return;
  DayRecV1& r = drA.rec;
  drParseVer(d["fw"].as<const char*>(), r.reglerFw);
  r.reglerResetReason = (uint8_t)(d["reset_reason"] | 255);
  uint32_t hk = d["heap_min"].as<uint32_t>() / 1024, st = d["stack_min"].as<uint32_t>();
  if (hk && hk < r.reglerHeapMinKb) r.reglerHeapMinKb = (uint16_t)hk;
  if (st && st < r.reglerStackMin)  r.reglerStackMin  = (uint16_t)(st > 0xFFFE ? 0xFFFE : st);
  uint32_t cur[6] = { d["boot_count"].as<uint32_t>(), d["shelly_fail"].as<uint32_t>(), d["shelly_retry"].as<uint32_t>(),
                      d["retreat_count"].as<uint32_t>(), d["mqtt_reconn"].as<uint32_t>(), d["mon_trip_count"].as<uint32_t>() };
  if (drA.seenR) {
    bool reboot = cur[0] != drA.lastR[0];             // Regler neu gestartet -> seine Zaehler begannen bei 0
    uint32_t dl[6];
    dl[0] = cur[0] > drA.lastR[0] ? cur[0] - drA.lastR[0] : (cur[0] < drA.lastR[0] ? 1 : 0);
    for (int i = 1; i < 6; i++) dl[i] = reboot ? cur[i] : drDelta(cur[i], drA.lastR[i]);
    r.reglerBoots       = drAdd8 (r.reglerBoots, dl[0]);
    r.reglerShellyFail  = drAdd16(r.reglerShellyFail, dl[1]);
    r.reglerShellyRetry = drAdd16(r.reglerShellyRetry, dl[2]);
    r.reglerRetreat     = drAdd16(r.reglerRetreat, dl[3]);
    r.reglerMqttReconn  = drAdd8 (r.reglerMqttReconn, dl[4]);
    r.reglerMonTrip     = drAdd8 (r.reglerMonTrip, dl[5]);
  }
  memcpy(drA.lastR, cur, sizeof(cur)); drA.seenR = 1;
}
void drOnBroker(JsonDocument& d) {
  if (!drA.started) return;
  DayRecV1& r = drA.rec;
  drParseVer(d["fw"].as<const char*>(), r.brokerFw);
  r.brokerResetReason = (uint8_t)(d["reset_reason"] | 255);
  uint32_t hk = d["heap_min"].as<uint32_t>() / 1024, st = d["stack_min"].as<uint32_t>();
  if (hk && hk < r.brokerHeapMinKb) r.brokerHeapMinKb = (uint16_t)hk;
  if (st && st < r.brokerStackMin)  r.brokerStackMin  = (uint16_t)(st > 0xFFFE ? 0xFFFE : st);
  int cl = d["clients_connected"] | -1;
  if (cl >= 0 && (r.brokerClientsMin == 255 || cl < r.brokerClientsMin)) r.brokerClientsMin = (uint8_t)(cl > 254 ? 254 : cl);
  r.brokerLastTripInv = (uint8_t)(d["last_trip_inv"] | 0);
  uint32_t cur[6] = { d["boot_count"].as<uint32_t>(), d["enforce_count"].as<uint32_t>(), d["monitor_trip_count"].as<uint32_t>(),
                      d["bypass_trip_count"].as<uint32_t>(), d["eth_down_count"].as<uint32_t>(), d["msg_count"].as<uint32_t>() };
  if (drA.seenB) {
    bool reboot = cur[0] != drA.lastB[0];
    uint32_t dl[6];
    dl[0] = cur[0] > drA.lastB[0] ? cur[0] - drA.lastB[0] : (cur[0] < drA.lastB[0] ? 1 : 0);
    for (int i = 1; i < 6; i++) dl[i] = reboot ? cur[i] : drDelta(cur[i], drA.lastB[i]);
    r.brokerBoots       = drAdd8 (r.brokerBoots, dl[0]);
    r.brokerEnforce     = drAdd16(r.brokerEnforce, dl[1]);
    r.brokerMonitorTrip = drAdd8 (r.brokerMonitorTrip, dl[2]);
    r.brokerBypassTrip  = drAdd8 (r.brokerBypassTrip, dl[3]);
    r.brokerEthDown     = drAdd8 (r.brokerEthDown, dl[4]);
    r.brokerMsg        += dl[5];
  }
  memcpy(drA.lastB, cur, sizeof(cur)); drA.seenB = 1;
}

// ── JSON-Ausgabe ─────────────────────────────────────────────────────────────
// Gepuffert: viele kleine print()-Aufrufe -> wenige TCP-Pakete
static char drOutBuf[1024];
class DrBufPrint : public Print {
public:
  explicit DrBufPrint(Print& o) : out(o), n(0) {}
  size_t write(uint8_t c) override { drOutBuf[n++] = (char)c; if (n == sizeof(drOutBuf)) drain(); return 1; }
  void drain() { if (n) { out.write((const uint8_t*)drOutBuf, n); n = 0; } }
private:
  Print& out; size_t n;
};
static void drK(Print& p, const char* k) { p.print(",\""); p.print(k); p.print("\":"); }
static void drKV (Print& p, const char* k, long v)                          { drK(p, k); p.print(v); }
static void drKVu(Print& p, const char* k, unsigned long v)                 { drK(p, k); p.print(v); }
static void drKVn(Print& p, const char* k, long v, long none)               { drK(p, k); if (v == none) p.print("null"); else p.print(v); }
static void drKVun(Print& p, const char* k, unsigned long v, unsigned long none) { drK(p, k); if (v == none) p.print("null"); else p.print(v); }
static void drKVs(Print& p, const char* k, const char* s)                   { drK(p, k); p.print("\""); p.print(s); p.print("\""); }
static void drKVhm(Print& p, const char* k, uint16_t m) {
  drK(p, k);
  if (m == 0xFFFF) { p.print("null"); return; }
  char b[8]; snprintf(b, sizeof(b), "\"%02u:%02u\"", (unsigned)(m / 60), (unsigned)(m % 60)); p.print(b);
}
static void drKVver(Print& p, const char* k, const uint8_t* v) {
  drK(p, k);
  if (!v[0] && !v[1] && !v[2]) { p.print("null"); return; }
  char b[16]; snprintf(b, sizeof(b), "\"%u.%u.%u\"", v[0], v[1], v[2]); p.print(b);
}
static const char* DR_CNT_NAMES[12] = { "c_regler_down", "c_broker_down", "c_control_down", "c_mqtt_down", "c_zen_stale",
                                        "c_shelly_out", "c_zendure_out", "c_bms", "c_tief", "c_netz", "c_bal", "c_warn" };

static void drPrintRec(Print& p, const DayRecV1& r) {
  char ds[12]; snprintf(ds, sizeof(ds), "%04u-%02u-%02u", (unsigned)r.year, (unsigned)r.month, (unsigned)r.day);
  char fw[13]; memcpy(fw, r.fw, 12); fw[12] = 0;
  p.print("{\"seq\":"); p.print((unsigned long)r.seq);
  drKVs(p, "date", ds);                 drKVu(p, "epoch_close", r.epochClose);
  drKVs(p, "fw", fw);                   drKV(p, "platform", r.platform);
  drKV(p, "flags", r.flags);            drKV(p, "minutes", r.minutesCovered);
  drKVu(p, "bezug_wh", r.bezugWh);      drKVu(p, "einsp_wh", r.einspWh);
  drKV(p, "saldo_wh", (long)r.einspWh - (long)r.bezugWh);
  drKVu(p, "zen_chg_wh", r.zenChgWh);   drKVu(p, "zen_dis_wh", r.zenDisWh);
  drKVn(p, "soc_min", r.socMin, 255);   drKVhm(p, "soc_min_at", r.socMinMinute);
  drKVn(p, "soc_max", r.socMax, 255);   drKVhm(p, "soc_max_at", r.socMaxMinute);
  drKVn(p, "soc_start", r.socStart, 255); drKVn(p, "soc_end", r.socEnd, 255);
  drKV(p, "netz_bezug_max_w", r.netzBezugMaxW); drKV(p, "netz_einsp_max_w", r.netzEinspMaxW);
  drKV(p, "zen_out_max_w", r.zenOutMaxW);       drKV(p, "zen_chg_max_w", r.zenChgMaxW);
  drKVn(p, "cell_max", r.cellMax, 0);   drKVn(p, "cell_min", r.cellMin, 0xFFFF);
  drKV(p, "spread_max_mv", r.spreadMax); drKVn(p, "temp_max", r.tempMax, 0);
  for (int i = 0; i < 12; i++) drKV(p, DR_CNT_NAMES[i], r.cnt[i]);
  drKV(p, "m_zendure_weg", r.minZendureWeg); drKV(p, "m_shelly_weg", r.minShellyWeg);
  drKV(p, "m_control_weg", r.minControlWeg); drKV(p, "m_soc_grenze", r.minSocGrenze);
  drKV(p, "w_boots", r.bootsToday);
  drKVn(p, "w_rssi_min", r.rssiMin, 127); drKVn(p, "w_rssi_max", r.rssiMax, -128); drKVn(p, "w_rssi_avg", r.rssiAvg, 127);
  drKVun(p, "w_min_free_heap", r.minFreeHeap, 0xFFFFFFFFUL);
  drKVu(p, "w_boot_count", r.bootCountTotal);
  drKVn(p, "w_drift_ppm_x10", r.driftPpmX10, 0x7FFF);
  drKV(p, "w_zen_http_max_ms", r.zenHttpMaxMs);
  drKVver(p, "r_fw", r.reglerFw);       drKV(p, "r_boots", r.reglerBoots);
  drKVn(p, "r_reset_reason", r.reglerResetReason, 255);
  drKVn(p, "r_heap_min_kb", r.reglerHeapMinKb, 0xFFFF); drKVn(p, "r_stack_min", r.reglerStackMin, 0xFFFF);
  drKV(p, "r_shelly_fail", r.reglerShellyFail); drKV(p, "r_shelly_retry", r.reglerShellyRetry);
  drKV(p, "r_retreat", r.reglerRetreat);        drKV(p, "r_mqtt_reconn", r.reglerMqttReconn);
  drKV(p, "r_mon_trip", r.reglerMonTrip);
  drKVver(p, "b_fw", r.brokerFw);       drKV(p, "b_boots", r.brokerBoots);
  drKVn(p, "b_reset_reason", r.brokerResetReason, 255);
  drKVn(p, "b_heap_min_kb", r.brokerHeapMinKb, 0xFFFF); drKVn(p, "b_stack_min", r.brokerStackMin, 0xFFFF);
  drKV(p, "b_enforce", r.brokerEnforce);        drKV(p, "b_monitor_trip", r.brokerMonitorTrip);
  drKV(p, "b_bypass_trip", r.brokerBypassTrip); drKV(p, "b_eth_down", r.brokerEthDown);
  drKV(p, "b_last_trip_inv", r.brokerLastTripInv);
  drKVn(p, "b_clients_min", r.brokerClientsMin, 255);
  drKVu(p, "b_msg", r.brokerMsg);
  drKV(p, "m_regler_weg", r.minReglerWeg);   drKV(p, "m_broker_weg", r.minBrokerWeg);
  drKV(p, "m_mqtt_down", r.minMqttDown);     drKV(p, "m_zen_stale", r.minZenStale);
  drKV(p, "m_saturated", r.minSaturated);    drKV(p, "m_discharge_blocked", r.minDischargeBlocked);
  drKV(p, "m_charge_blocked", r.minChargeBlocked);
  drKV(p, "m_bezug_band", r.minBezugBand);   drKV(p, "m_einsp_band", r.minEinspBand);
  drKVn(p, "netz_abw_mittel_w", r.netzAbwMittelW, 0xFFFF);
  p.print("}");
}
static const char* DR_HTTP_JSON = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\nAccess-Control-Allow-Origin: *\r\n\r\n";

// GET /daily[?from=<seq>&n=<1..31>]  -> abgeschlossene Tage, aelteste zuerst (ohne Parameter: letzte 7)
void drJsonDaily(Print& out, const String& req) {
  DrBufPrint p(out);
  long n = 7, from = -1;
  int i = req.indexOf("n=");    if (i >= 0 && (i == 0 || req[i - 1] == '?' || req[i - 1] == '&')) n = req.substring(i + 2).toInt();
  i = req.indexOf("from=");     if (i >= 0) from = req.substring(i + 5).toInt();
  if (n < 1) n = 1;
  if (n > 31) n = 31;
  long newest = (long)drNewest;
  long oldest = newest >= DR_CAP ? newest - DR_CAP + 1 : 1;
  if (from < 0) from = newest - n + 1;
  if (from < oldest) from = oldest;
  p.print(DR_HTTP_JSON);
  p.print("{\"fmt\":"); p.print(DR_FMT);
  drKV(p, "capacity", DR_CAP);   drKV(p, "stored", drStored); drKV(p, "count", drStored);
  drKV(p, "newest_seq", newest); drKV(p, "oldest_seq", newest ? oldest : 0); drKV(p, "from", from);
  p.print(",\"days\":[");
  int k = 0;
  for (long s = from; s <= newest && k < n && s - from < (long)DR_CAP; s++) {
    DayRecV1 r;
    if (!drReadSeq((uint32_t)s, r)) continue;      // Luecke / defekter Record -> ueberspringen, nie "reparieren"
    if (k) p.print(",");
    drPrintRec(p, r);
    k++;
  }
  p.print("]"); drKV(p, "returned", k); p.print("}");
  p.drain();
}
// GET /daily/cur -> laufender Tag (so wie er jetzt abgeschlossen wuerde, seq 0)
void drJsonCur(Print& out) {
  DrBufPrint p(out);
  p.print(DR_HTTP_JSON);
  if (!drA.started) { p.print("{\"started\":false}"); p.drain(); return; }
  DayRecV1 r;
  long ymd = drA.dayNo;
#if DR_TEST_DAY_MIN > 0
  { DrLive l; drLive(l); ymd = l.today; }
#endif
  drFinalize(drA, r, ymd, 0);
  p.print("{\"started\":true,\"day_no\":"); p.print((long)drA.dayNo);
  drKV(p, "minutes_since_checkpoint", drMinTicks);
  p.print(",\"rec\":"); drPrintRec(p, r); p.print("}");
  p.drain();
}
// GET /daily/info -> Speicher-/Ring-Zustand
void drJsonInfo(Print& out) {
  DrBufPrint p(out);
  p.print(DR_HTTP_JSON);
  p.print("{\"fmt\":"); p.print(DR_FMT);
  drKV(p, "rec_size", (long)sizeof(DayRecV1)); drKV(p, "capacity", DR_CAP);
  drKVs(p, "store", DR_STORE_NAME);            p.print(",\"store_ok\":"); p.print(drStoreOk ? "true" : "false");
  drStoreInfo(p);
  drKV(p, "stored", drStored);   drKVu(p, "newest_seq", drNewest); drKVu(p, "next_seq", drNextSeq);
  drKVu(p, "write_err", drWriteErr);
  drKV(p, "test_day_min", DR_TEST_DAY_MIN);
  p.print(",\"cur_started\":"); p.print(drA.started ? "true" : "false");
  drKV(p, "cur_day_no", drA.started ? (long)drA.dayNo : -1L);
  drKV(p, "cur_minutes", drA.started ? (long)drA.rec.minutesCovered : 0L);
  p.print("}");
  p.drain();
}
// Kurzfassung fuer /status (ohne fuehrende/abschliessende Klammer)
void drJsonStatusFields(Print& c) {
  c.print(",\"dr_ok\":");     c.print(drStoreOk ? "true" : "false");
  c.print(",\"dr_stored\":"); c.print(drStored);
  c.print(",\"dr_seq\":");    c.print((unsigned long)drNewest);
  c.print(",\"dr_err\":");    c.print((unsigned long)drWriteErr);
  c.print(",\"dr_min\":");    c.print(drA.started ? (long)drA.rec.minutesCovered : -1L);
}
