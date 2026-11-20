/**
 * XY-MD04 Temperatur/Luftfeuchte Sensor – RS485 Modbus RTU
 * Hardware: M5Stack CoreS3 + Module COMMU (RS485-Transceiver SP3485)
 *
 * Versionshistorie:
 *   v1.1.0 (2026-11-20) – Grafische Display-Ausgabe: Status-Leiste fuer alle IDs (gruen = OK,
 *                         orange = fehlerhafte Antwort, rot = ausgefallen), Kacheln je Sensor
 *                         mit Temperatur/Feuchte-Balken und Verlaufskurve, Fortschrittsbalken,
 *                         Diagnosehinweise ohne Sensor. Flackerfrei ueber Vollbild-Sprite.
 *                         Serielle Befehle: 'r' = neu scannen, 's' = Screenshot (RGB565-Dump).
 *   v1.0.0 (2026-11-20) – Portierung von xy_md04_atom_dtu2 v1.8.0 (ATOM DTU NB-IoT2):
 *                         RS485 ueber COMMU auf Serial2 (RX=G18, TX=G17), keine DE/RE-
 *                         Umschaltung noetig (COMMU schaltet automatisch), Messwerte auf
 *                         dem Display, Touch statt Button G41, Empfang endet sobald die
 *                         Antwort vollstaendig ist (schnellerer Scan)
 *
 *   Vorgaenger (ATOM DTU NB-IoT2): v1.0.0 – v1.8.0 (2026-04-20)
 *
 * Verdrahtung XY-MD04 → COMMU RS485-Anschluss:
 *   Rot     → VCC (5–28V)
 *   Schwarz → GND (gemeinsame Masse mit CoreS3/COMMU)
 *   Gelb    → RS485-A (A+)
 *   Weiß    → RS485-B (B-)
 *
 * RS485 (COMMU ueber M5-Bus):
 *   M5-Bus Pin 15 → GPIO 18 → RX   (COMMU-Bezeichnung "RS485_TX")
 *   M5-Bus Pin 16 → GPIO 17 → TX   (COMMU-Bezeichnung "RS485_RX")
 *   DE/RE         → auf dem COMMU automatisch, nicht am M5-Bus herausgefuehrt
 *   Hinweis: G17/G18 sind auch Port C des CoreS3 → Port C nicht zusaetzlich belegen.
 */

#include <M5Unified.h>
#include <stdarg.h>

// ── Version ───────────────────────────────────────────────────────────────────
#define FW_VERSION      "v1.1.0"
#define FW_DATE         "2026-11-20"

// ── Konfiguration ─────────────────────────────────────────────────────────────
#define RS485_RX         18      // M5-Bus Pin 15
#define RS485_TX         17      // M5-Bus Pin 16
#define RS485_DE_RE      -1      // -1 = automatische Richtungsumschaltung (COMMU)
#define COMMU_CAN_CS     6       // M5-Bus Pin 21: MCP2515 (CAN) abgewaehlt halten

#define BAUD_RATE        9600
#define SENSOR_ID_MIN    1       // Erste Sensor-ID
#define SENSOR_ID_MAX    20      // Letzte Sensor-ID
#define READ_INTERVAL_MS 5000    // Pause zwischen kompletten Durchläufen

static HardwareSerial& rs485 = Serial2;

// ── Adressänderung ────────────────────────────────────────────────────────────
// CHANGE_ADDRESS true  → Adresse ändern, dann per Touch in Scan-Modus
// CHANGE_ADDRESS false → direkt normaler Multi-Sensor-Betrieb
#define CHANGE_ADDRESS   false
#define CHANGE_FROM_ADDR 0x01   // Aktuelle Adresse des Sensors
#define CHANGE_TO_ADDR   0x03   // Neue Adresse (1–247)

// ── Messwerte ─────────────────────────────────────────────────────────────────

#define HIST 60   // Verlaufspunkte je Sensor (1 Punkt pro Scan-Durchlauf)

enum AddrState : uint8_t { ST_IDLE, ST_SCANNING, ST_OK, ST_NONE, ST_BAD, ST_LOST };

struct Sensor {
    bool    ok;      // letzte Abfrage erfolgreich
    bool    seen;    // jemals geantwortet (Kachel bleibt dann sichtbar)
    float   temp;
    float   hum;
    uint8_t st;      // AddrState fuer die Status-Leiste
    uint8_t n;       // Anzahl Verlaufspunkte
    uint8_t head;    // naechste Schreibposition im Ringpuffer
    int16_t tHist[HIST];   // Temperatur * 10
    int16_t hHist[HIST];   // Feuchte * 10
};
static Sensor sensors[SENSOR_ID_MAX + 1];

static void pushHist(Sensor& s) {
    s.tHist[s.head] = (int16_t)lroundf(s.temp * 10.0f);
    s.hHist[s.head] = (int16_t)lroundf(s.hum  * 10.0f);
    s.head = (s.head + 1) % HIST;
    if (s.n < HIST) s.n++;
}

// Zustand fuer Kopf-/Fusszeile
struct UiState {
    bool     scanning;
    bool     firstDone;
    uint8_t  cur;
    uint8_t  found;
    uint8_t  bad;
    uint16_t scanNo;
};
static UiState ui = {};

// Letzte Rohantwort von readTempHumidity (fuer Diagnose)
static uint8_t lastRx[16];
static uint8_t lastRxLen = 0;

// ── Ausgabe: Serial + (optional) Display-Konsole ──────────────────────────────

static bool lcdConsole = false;   // true: Display dient als Textkonsole (Adressaenderung)

void logf(const char* fmt, ...) {
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    Serial.print(buf);
    if (lcdConsole) M5.Display.print(buf);
}

void printHex(const char* label, const uint8_t* buf, uint8_t len) {
    logf("  %s [%d Bytes]: ", label, len);
    for (uint8_t i = 0; i < len; i++) logf("%02X ", buf[i]);
    logf("\n");
}

// ── Display (Scan-Ansicht) ────────────────────────────────────────────────────

// Alles wird in ein Vollbild-Sprite gezeichnet und dann am Stueck uebertragen
// (kein Flackern, und das Bild kann per 's' als Screenshot ausgelesen werden).
static M5Canvas screen(&M5.Display);

static const int HEADER_H = 24;
static const int STRIP_Y  = 27;
static const int STRIP_H  = 18;
static const int MAIN_Y   = 49;
static const int FOOTER_H = 22;

// Farben (RGB565, bewusst uint16_t: uint32_t wuerde als RGB888 interpretiert)
static constexpr uint16_t C_BG        = 0x0000;
static constexpr uint16_t C_HEADER    = 0x0010;  // Navy
static constexpr uint16_t C_FOOTER    = 0x0841;
static constexpr uint16_t C_TILE      = 0x1907;  // dunkles Blaugrau
static constexpr uint16_t C_CHART     = 0x0865;
static constexpr uint16_t C_GRID      = 0x2945;
static constexpr uint16_t C_BAR_BG    = 0x31A6;
static constexpr uint16_t C_TEXT      = 0xFFFF;
static constexpr uint16_t C_LABEL     = 0xAD55;
static constexpr uint16_t C_DIM       = 0x6B4D;
static constexpr uint16_t C_HUM       = 0x5D7F;  // Himmelblau
static constexpr uint16_t C_OK        = 0x0600;  // Gruen (Status-Leiste)
static constexpr uint16_t C_NONE      = 0x2104;
static constexpr uint16_t C_BAD       = 0xFC60;  // Orange
static constexpr uint16_t C_LOST      = 0xB000;  // Dunkelrot
static constexpr uint16_t C_SCAN      = 0xFFE0;  // Gelb
static constexpr uint16_t C_RED       = 0xF8C3;
static constexpr uint16_t C_GREEN     = 0x2FE5;

static uint16_t tempColor(float t) {
    if (t < 5.0f)  return 0x5C9F;   // blau
    if (t < 16.0f) return 0x07FF;   // cyan
    if (t < 24.0f) return C_GREEN;  // gruen (Komfortbereich)
    if (t < 28.0f) return 0xFD20;   // orange
    return C_RED;                   // rot
}

// Wertschrift mit Hoehe der Ziffern (cap) und Radius des Grad-Zeichens
struct ValFont { const lgfx::IFont* f; int8_t cap; int8_t r; };
static const ValFont VF24 = { &fonts::FreeSansBold24pt7b, 34, 4 };
static const ValFont VF18 = { &fonts::FreeSansBold18pt7b, 25, 3 };
static const ValFont VF12 = { &fonts::FreeSansBold12pt7b, 17, 2 };
static const ValFont VF9  = { &fonts::FreeSansBold9pt7b,  13, 2 };
static const ValFont VF2  = { &fonts::Font2,              10, 2 };

static int tempWidth(float t, const ValFont& vf, bool withC) {
    char buf[12];
    snprintf(buf, sizeof(buf), "%.1f", t);
    screen.setFont(vf.f);
    int w = screen.textWidth(buf) + 2 * vf.r + 3;
    if (withC) w += 1 + screen.textWidth("C");
    return w;
}

// Temperatur mit gezeichnetem Grad-Zeichen, (x, base) = linke Grundlinie
static void drawTemp(int x, int base, float t, const ValFont& vf, bool withC, uint16_t col) {
    char buf[12];
    snprintf(buf, sizeof(buf), "%.1f", t);
    screen.setFont(vf.f);
    screen.setTextDatum(baseline_left);
    screen.setTextColor(col);
    screen.drawString(buf, x, base);
    const int dx = x + screen.textWidth(buf) + 2 + vf.r;
    const int dy = base - vf.cap + vf.r;
    screen.drawCircle(dx, dy, vf.r, col);
    if (vf.r >= 3) screen.drawCircle(dx, dy, vf.r - 1, col);
    if (withC) screen.drawString("C", dx + vf.r + 3, base);
}

static void drawBar(int x, int y, int w, int h, float frac, uint16_t col) {
    if (frac < 0) frac = 0;
    if (frac > 1) frac = 1;
    screen.fillRoundRect(x, y, w, h, h / 2, C_BAR_BG);
    const int fw = (int)(w * frac + 0.5f);
    if (fw >= h)     screen.fillRoundRect(x, y, fw, h, h / 2, col);
    else if (fw > 0) screen.fillRect(x, y, fw, h, col);
}

static inline float tempFrac(float t) { return (t + 10.0f) / 60.0f; }   // Balken -10 … 50 °C
static inline float humFrac(float h)  { return h / 100.0f; }

// Verlaufskurve: Temperatur und Feuchte, jeweils eigene Auto-Skalierung
static void drawChart(int x, int y, int w, int h, const Sensor& s, uint16_t tc, uint16_t hc) {
    screen.fillRoundRect(x, y, w, h, 4, C_CHART);
    screen.setFont(&fonts::Font0);
    if (s.n < 2) {
        screen.setTextDatum(middle_center);
        screen.setTextColor(C_DIM);
        screen.drawString("Verlauf wird aufgezeichnet ...", x + w / 2, y + h / 2);
        return;
    }

    float tmin = 1e9f, tmax = -1e9f, hmin = 1e9f, hmax = -1e9f;
    for (uint8_t i = 0; i < s.n; i++) {
        const uint8_t k = (s.head + HIST - s.n + i) % HIST;
        const float t = s.tHist[k] / 10.0f, hu = s.hHist[k] / 10.0f;
        if (t < tmin) tmin = t;
        if (t > tmax) tmax = t;
        if (hu < hmin) hmin = hu;
        if (hu > hmax) hmax = hu;
    }

    char buf[28];
    screen.setTextDatum(top_left);
    screen.setTextColor(tc);
    snprintf(buf, sizeof(buf), "T %.1f..%.1f", tmin, tmax);
    screen.drawString(buf, x + 4, y + 3);
    screen.setTextDatum(top_right);
    screen.setTextColor(hc);
    snprintf(buf, sizeof(buf), "H %.0f..%.0f%%", hmin, hmax);
    screen.drawString(buf, x + w - 4, y + 3);

    if (tmax - tmin < 1.0f) { const float m = (tmax + tmin) / 2; tmin = m - 0.5f; tmax = m + 0.5f; }
    if (hmax - hmin < 4.0f) { const float m = (hmax + hmin) / 2; hmin = m - 2.0f; hmax = m + 2.0f; }

    const int px = x + 4, pw = w - 8, py = y + 13, ph = h - 17;
    for (int g = 1; g <= 3; g++) {
        const int gy = py + ph * g / 4;
        for (int gx = px; gx < px + pw; gx += 4) screen.drawPixel(gx, gy, C_GRID);
    }

    const float step = (float)pw / (HIST - 1);
    auto X = [&](uint8_t i) { return px + pw - (int)((s.n - 1 - i) * step + 0.5f); };

    // Feuchte zuerst (liegt hinten), dann Temperatur doppelt dick
    int lx = 0, ly = 0;
    for (uint8_t i = 0; i < s.n; i++) {
        const uint8_t k = (s.head + HIST - s.n + i) % HIST;
        const int xi = X(i);
        const int yi = py + ph - (int)((s.hHist[k] / 10.0f - hmin) / (hmax - hmin) * ph + 0.5f);
        if (i) screen.drawLine(lx, ly, xi, yi, hc);
        lx = xi; ly = yi;
    }
    for (uint8_t i = 0; i < s.n; i++) {
        const uint8_t k = (s.head + HIST - s.n + i) % HIST;
        const int xi = X(i);
        const int yi = py + ph - (int)((s.tHist[k] / 10.0f - tmin) / (tmax - tmin) * ph + 0.5f);
        if (i) { screen.drawLine(lx, ly, xi, yi, tc); screen.drawLine(lx, ly + 1, xi, yi + 1, tc); }
        lx = xi; ly = yi;
    }
    screen.fillCircle(lx, ly, 2, tc);   // aktueller Wert
}

// ── Kacheln ──

// Gross (1–2 Sensoren): Werte, Balken und Verlauf
static void drawTileLarge(int x, int y, int w, int h, uint8_t addr, const Sensor& s) {
    const uint16_t tc = s.ok ? tempColor(s.temp) : C_DIM;
    const uint16_t hc = s.ok ? C_HUM : C_DIM;
    char buf[24];

    screen.fillRoundRect(x, y, w, h, 6, C_TILE);
    screen.setFont(&fonts::Font2);
    screen.setTextDatum(top_left);
    screen.setTextColor(C_LABEL);
    snprintf(buf, sizeof(buf), "Sensor %u", addr);
    screen.drawString(buf, x + 8, y + 5);
    screen.setTextDatum(top_right);
    if (s.ok) { screen.setTextColor(C_DIM); snprintf(buf, sizeof(buf), "ID 0x%02X", addr); }
    else      { screen.setTextColor(C_RED); snprintf(buf, sizeof(buf), "OFFLINE"); }
    screen.drawString(buf, x + w - 8, y + 5);

    char hbuf[12];
    snprintf(hbuf, sizeof(hbuf), "%.1f%%", s.hum);

    if (w >= 250) {
        // Breit: Temperatur links, Feuchte rechts, darunter Balken und Verlauf
        const ValFont* vf = &VF24;
        screen.setFont(vf->f);
        if (tempWidth(s.temp, *vf, true) + screen.textWidth(hbuf) + 30 > w) vf = &VF18;
        const int base = y + 60;
        drawTemp(x + 10, base, s.temp, *vf, true, tc);
        screen.setFont(vf->f);
        screen.setTextDatum(baseline_right);
        screen.setTextColor(hc);
        screen.drawString(hbuf, x + w - 10, base);
        const int half = (w - 30) / 2;
        drawBar(x + 10,        y + 68, half, 6, tempFrac(s.temp), tc);
        drawBar(x + 20 + half, y + 68, half, 6, humFrac(s.hum),  hc);
        drawChart(x + 6, y + 82, w - 12, h - 88, s, tc, hc);
    } else {
        // Schmal: untereinander
        drawTemp(x + 8, y + 50, s.temp, VF18, tempWidth(s.temp, VF18, true) <= w - 16, tc);
        drawBar(x + 8, y + 57, w - 16, 5, tempFrac(s.temp), tc);
        screen.setFont(VF12.f);
        screen.setTextDatum(baseline_left);
        screen.setTextColor(hc);
        screen.drawString(hbuf, x + 8, y + 84);
        drawBar(x + 8, y + 90, w - 16, 5, humFrac(s.hum), hc);
        drawChart(x + 4, y + 100, w - 8, h - 104, s, tc, hc);
    }
}

// Farbiger Streifen links mit der Sensor-ID
static void drawIdStrip(int x, int y, int h, int sw, int r, uint8_t addr, uint16_t col, bool ok,
                        const lgfx::IFont* f) {
    screen.fillRoundRect(x, y, sw + r, h, r, col);
    screen.fillRect(x + sw, y, r, h, C_TILE);
    screen.setFont(f);
    screen.setTextDatum(middle_center);
    screen.setTextColor(ok ? C_BG : C_TEXT);
    screen.drawNumber(addr, x + sw / 2, y + h / 2);
}

// Mittel (3–6 Sensoren): Werte + zwei Balken
static void drawTileMedium(int x, int y, int w, int h, uint8_t addr, const Sensor& s) {
    const uint16_t tc = s.ok ? tempColor(s.temp) : C_DIM;
    const uint16_t hc = s.ok ? C_HUM : C_DIM;
    screen.fillRoundRect(x, y, w, h, 5, C_TILE);
    drawIdStrip(x, y, h, 18, 5, addr, tc, s.ok, &fonts::Font2);

    const int cx = x + 18 + 6;
    const int cw = x + w - 6 - cx;
    const bool big = cw >= 110;
    const ValFont& tf = big ? VF18 : VF12;
    const ValFont& hf = big ? VF12 : VF9;

    const int base1 = y + 8 + tf.cap;
    drawTemp(cx, base1, s.temp, tf, tempWidth(s.temp, tf, true) <= cw, tc);

    char buf[12];
    snprintf(buf, sizeof(buf), "%.1f%%", s.hum);
    screen.setFont(hf.f);
    screen.setTextDatum(baseline_left);
    screen.setTextColor(hc);
    screen.drawString(buf, cx, base1 + 8 + hf.cap);

    drawBar(cx, y + h - 15, cw, 4, tempFrac(s.temp), tc);
    drawBar(cx, y + h - 8,  cw, 4, humFrac(s.hum),  hc);

    if (!s.ok) {
        screen.setFont(&fonts::Font0);
        screen.setTextDatum(top_right);
        screen.setTextColor(C_RED);
        screen.drawString("OFFLINE", x + w - 5, y + 4);
    }
}

// Kompakt (7–20 Sensoren): nur Werte
static void drawTileCompact(int x, int y, int w, int h, uint8_t addr, const Sensor& s) {
    const uint16_t tc = s.ok ? tempColor(s.temp) : C_DIM;
    const uint16_t hc = s.ok ? C_HUM : C_DIM;
    screen.fillRoundRect(x, y, w, h, 4, C_TILE);
    drawIdStrip(x, y, h, 14, 4, addr, tc, s.ok, &fonts::Font0);

    const int rx    = x + w - 4;
    const int avail = rx - (x + 14 + 3);
    const ValFont& vf = (w >= 70) ? VF9 : VF2;
    const int base1 = y + (h - (vf.cap + 5 + 12)) / 2 + vf.cap;
    drawTemp(rx - tempWidth(s.temp, vf, false), base1, s.temp, vf, false, tc);

    char buf[12];
    snprintf(buf, sizeof(buf), "%.1f%%", s.hum);
    screen.setFont(&fonts::Font2);
    if (screen.textWidth(buf) > avail) snprintf(buf, sizeof(buf), "%.0f%%", s.hum);
    screen.setTextDatum(top_right);
    screen.setTextColor(hc);
    screen.drawString(buf, rx, base1 + 4);
}

static void gridFor(uint8_t n, uint8_t& cols, uint8_t& rows) {
    if      (n <= 1)  { cols = 1; rows = 1; }
    else if (n == 2)  { cols = 2; rows = 1; }
    else if (n <= 4)  { cols = 2; rows = 2; }
    else if (n <= 6)  { cols = 3; rows = 2; }
    else if (n <= 9)  { cols = 3; rows = 3; }
    else if (n <= 12) { cols = 4; rows = 3; }
    else if (n <= 16) { cols = 4; rows = 4; }
    else              { cols = 5; rows = (n + 4) / 5; }
}

// ── Bildbereiche ──

static void drawHeader() {
    const int W = screen.width();
    screen.fillRect(0, 0, W, HEADER_H, C_HEADER);
    screen.setFont(&fonts::Font2);
    screen.setTextDatum(middle_left);
    screen.setTextColor(C_TEXT);
    screen.drawString("XY-MD04  Klima-Monitor", 6, HEADER_H / 2);
    char buf[32];
    snprintf(buf, sizeof(buf), "%d Bd  |  %s", BAUD_RATE, FW_VERSION);
    screen.setTextDatum(middle_right);
    screen.setTextColor(C_LABEL);
    screen.drawString(buf, W - 6, HEADER_H / 2);
}

// Eine Box je Adresse: grau = noch nicht abgefragt, gelb = wird abgefragt,
// gruen = OK, dunkel = keine Antwort, orange = fehlerhafte Antwort, rot = ausgefallen
static void drawStrip() {
    const int W = screen.width();
    const int n = SENSOR_ID_MAX - SENSOR_ID_MIN + 1;
    const int cell = (W - 8) / n;
    const int x0 = (W - cell * n) / 2;
    const int bw = cell - 2;
    screen.setFont(&fonts::Font0);
    screen.setTextDatum(middle_center);
    for (int i = 0; i < n; i++) {
        const uint8_t addr = SENSOR_ID_MIN + i;
        const int x = x0 + i * cell + 1;
        uint16_t fill, txt;
        switch (sensors[addr].st) {
            case ST_SCANNING: fill = C_SCAN; txt = C_BG;    break;
            case ST_OK:       fill = C_OK;   txt = C_TEXT;  break;
            case ST_BAD:      fill = C_BAD;  txt = C_BG;    break;
            case ST_LOST:     fill = C_LOST; txt = C_TEXT;  break;
            case ST_NONE:     fill = C_NONE; txt = C_DIM;   break;
            default:          fill = C_BG;   txt = C_DIM;   break;
        }
        if (sensors[addr].st == ST_IDLE) screen.drawRoundRect(x, STRIP_Y, bw, STRIP_H, 3, C_NONE);
        else                             screen.fillRoundRect(x, STRIP_Y, bw, STRIP_H, 3, fill);
        if (bw >= 12 || addr < 10) {
            screen.setTextColor(txt);
            screen.drawNumber(addr, x + bw / 2, STRIP_Y + STRIP_H / 2 + 1);
        }
    }
}

static void drawEmpty(int x, int y, int w, int h) {
    const int cx = x + w / 2;
    if (!ui.firstDone) {
        screen.setFont(&fonts::FreeSans9pt7b);
        screen.setTextDatum(middle_center);
        screen.setTextColor(C_LABEL);
        screen.drawString("Erster Scan laeuft ...", cx, y + h / 2);
        return;
    }

    // Titel mit Warndreieck
    const char* title = "Keine Sensoren gefunden";
    const lgfx::IFont* tf = &fonts::FreeSansBold12pt7b;
    screen.setFont(tf);
    if (28 + screen.textWidth(title) > w - 8) { tf = &fonts::FreeSansBold9pt7b; screen.setFont(tf); }
    const int tw = screen.textWidth(title);
    const int total = 28 + tw;
    const int sx = cx - total / 2;
    const int ty = y + 18;
    screen.fillTriangle(sx + 11, ty - 11, sx, ty + 9, sx + 22, ty + 9, C_SCAN);
    screen.setFont(&fonts::Font2);
    screen.setTextDatum(middle_center);
    screen.setTextColor(C_BG);
    screen.drawString("!", sx + 11, ty + 2);
    screen.setFont(tf);
    screen.setTextDatum(middle_left);
    screen.setTextColor(C_RED);
    screen.drawString(title, sx + 28, ty);

    // Hinweise
    screen.setFont(&fonts::Font2);
    screen.setTextDatum(top_left);
    int ly = y + 44;
    const int lx = x + 18;
    if (ui.bad) {
        char buf[48];
        snprintf(buf, sizeof(buf), "%u ID(s) antworten fehlerhaft (orange):", ui.bad);
        screen.setTextColor(C_BAD);
        screen.drawString(buf, lx, ly);              ly += 18;
        screen.drawString("  -> Baudrate / A-B-Polaritaet pruefen", lx, ly); ly += 24;
    }
    screen.setTextColor(C_LABEL);
    static const char* const hints[] = {
        "- A/B tauschen (Gelb = A+, Weiss = B-)",
        "- Sensor-Versorgung 5-28 V DC pruefen",
        "- GND Sensor <-> COMMU verbinden",
        "- Sensor auf 9600 Bd und ID 1-20?",
    };
    for (const char* hnt : hints) {
        if (ly > y + h - 16) break;
        screen.drawString(hnt, lx, ly);
        ly += 20;
    }
}

static void drawTiles(int X, int Y, int W, int H) {
    uint8_t list[SENSOR_ID_MAX + 1];
    uint8_t n = 0;
    for (uint8_t addr = SENSOR_ID_MIN; addr <= SENSOR_ID_MAX; addr++)
        if (sensors[addr].seen) list[n++] = addr;

    if (n == 0) { drawEmpty(X, Y, W, H); return; }

    uint8_t cols, rows;
    gridFor(n, cols, rows);
    const int gap = 4;
    const int tw = (W - (cols - 1) * gap) / cols;
    const int th = (H - (rows - 1) * gap) / rows;
    for (uint8_t i = 0; i < n; i++) {
        const int x = X + (i % cols) * (tw + gap);
        const int y = Y + (i / cols) * (th + gap);
        const Sensor& s = sensors[list[i]];
        if (th >= 120 && tw >= 140) drawTileLarge  (x, y, tw, th, list[i], s);
        else if (th >= 70)          drawTileMedium (x, y, tw, th, list[i], s);
        else                        drawTileCompact(x, y, tw, th, list[i], s);
    }
}

static void drawFooter() {
    const int W = screen.width();
    const int y = screen.height() - FOOTER_H;
    const uint8_t total = SENSOR_ID_MAX - SENSOR_ID_MIN + 1;
    char buf[48];

    screen.fillRect(0, y, W, FOOTER_H, C_FOOTER);
    screen.drawFastHLine(0, y, W, C_GRID);
    screen.setFont(&fonts::Font2);
    screen.setTextDatum(middle_left);

    if (ui.scanning) {
        const float p = (float)(ui.cur - SENSOR_ID_MIN) / total;
        screen.fillRect(0, y, (int)(W * p), 3, C_SCAN);
        snprintf(buf, sizeof(buf), "Scan #%u   ID %u/%u ...   %u gefunden",
                 ui.scanNo, ui.cur, SENSOR_ID_MAX, ui.found);
        screen.setTextColor(C_SCAN);
        screen.drawString(buf, 6, y + FOOTER_H / 2 + 1);
    } else {
        snprintf(buf, sizeof(buf), "Scan #%u:  %u/%u gefunden", ui.scanNo, ui.found, total);
        screen.setTextColor(ui.found ? C_GREEN : C_RED);
        screen.drawString(buf, 6, y + FOOTER_H / 2 + 1);
        screen.setTextDatum(middle_right);
        screen.setTextColor(C_DIM);
        screen.drawString("Touch = Scan", W - 6, y + FOOTER_H / 2 + 1);
    }
}

static void render() {
    screen.fillScreen(C_BG);
    drawHeader();
    drawStrip();
    drawTiles(4, MAIN_Y, screen.width() - 8, screen.height() - FOOTER_H - MAIN_Y - 3);
    drawFooter();
    screen.pushSprite(0, 0);
}

// Screenshot: "SCREENSHOT <w> <h> <bytes>\n" + Rohdaten RGB565 (Big-Endian, wie im Sprite)
static void dumpScreen() {
    const uint8_t* buf = (const uint8_t*)screen.getBuffer();
    if (!buf) { Serial.println("SCREENSHOT FEHLER: kein Sprite"); return; }
    const size_t len = (size_t)screen.width() * screen.height() * 2;
    Serial.printf("\nSCREENSHOT %d %d %u\n", screen.width(), screen.height(), (unsigned)len);
    for (size_t off = 0; off < len; off += 512) {
        const size_t n = (len - off < 512) ? (len - off) : 512;
        Serial.write(buf + off, n);
    }
    Serial.flush();
    Serial.println("\nSCREENSHOT ENDE");
}

// Serielle Befehle: 's' = Screenshot, 'r' = neu scannen. Rueckgabe: Neuscan angefordert
static bool handleSerial() {
    bool rescan = false;
    while (Serial.available()) {
        const int ch = Serial.read();
        if (ch == 's' || ch == 'S') dumpScreen();
        else if (ch == 'r' || ch == 'R') rescan = true;
    }
    return rescan;
}

// ── CRC ───────────────────────────────────────────────────────────────────────

uint16_t crc16(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
        }
    }
    return crc;
}

// ── RS485 Kommunikation ───────────────────────────────────────────────────────

void rs485Send(const uint8_t* buf, uint8_t len) {
#if RS485_DE_RE >= 0
    digitalWrite(RS485_DE_RE, HIGH);
    delayMicroseconds(1000);
#endif
    rs485.write(buf, len);
    rs485.flush();
#if RS485_DE_RE >= 0
    delayMicroseconds(1000);
    digitalWrite(RS485_DE_RE, LOW);
#endif
}

// Empfaengt bis timeoutMs ohne neues Byte vergangen sind. Bricht frueher ab, wenn
// expectLen Bytes da sind (0 = nur Timeout) oder eine Modbus-Exception (5 Bytes) komplett ist.
uint8_t rs485Receive(uint8_t* buf, uint8_t maxLen, uint32_t timeoutMs = 500, uint8_t expectLen = 0) {
    uint32_t start = millis();
    uint8_t idx = 0;
    while (millis() - start < timeoutMs) {
        if (rs485.available()) {
            buf[idx++] = rs485.read();
            if (idx >= maxLen) break;
            if (expectLen && idx >= expectLen) break;
            if (idx == 5 && (buf[1] & 0x80)) break;
            start = millis();
        } else {
            delay(1);
        }
    }
    return idx;
}

// ── Adresse lesen (FC 0x03, Register 0x0101) ─────────────────────────────────

uint8_t readAddress(uint8_t queryAddr) {
    uint8_t req[8];
    req[0] = queryAddr;
    req[1] = 0x03;
    req[2] = 0x01;
    req[3] = 0x01;
    req[4] = 0x00;
    req[5] = 0x01;
    uint16_t crc = crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = crc >> 8;

    while (rs485.available()) rs485.read();
    rs485Send(req, 8);

    uint8_t resp[16];
    uint8_t n = rs485Receive(resp, 16, 500, 7);

    if (n < 7) return 0xFF;
    uint16_t recvCrc = (uint16_t)resp[n-1] << 8 | resp[n-2];
    if (recvCrc != crc16(resp, n - 2)) return 0xFF;
    return resp[4];
}

// ── Einzelnen Sensor auslesen (FC 0x04, Register 0x0001–0x0002) ──────────────

bool readTempHumidity(uint8_t addr, float& temp, float& hum) {
    uint8_t req[8];
    req[0] = addr;
    req[1] = 0x04;
    req[2] = 0x00;
    req[3] = 0x01;
    req[4] = 0x00;
    req[5] = 0x02;
    uint16_t crc = crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = crc >> 8;

    while (rs485.available()) rs485.read();
    rs485Send(req, 8);

    uint8_t* resp = lastRx;
    uint8_t n = rs485Receive(resp, sizeof(lastRx), 500, 9);
    lastRxLen = n;

    if (n < 9) return false;

    uint16_t recvCrc = (uint16_t)resp[n-1] << 8 | resp[n-2];
    if (recvCrc != crc16(resp, n - 2)) return false;

    temp = (int16_t)((uint16_t)resp[3] << 8 | resp[4]) / 10.0f;
    hum  = ((uint16_t)resp[5] << 8 | resp[6]) / 10.0f;
    return true;
}

// ── Baudrate-Codierung für FC10 (laut Datenblatt) ────────────────────────────
// 9600=0x2580, 14400=0x3840, 19200=0x4B00, 38400=0x9600, 115200=0x2D00(11520)
uint16_t baudCode(uint32_t baud) {
    switch (baud) {
        case 14400:  return 0x3840;
        case 19200:  return 0x4B00;
        case 38400:  return 0x9600;
        case 115200: return 0x2D00;  // Sensor intern: 11520
        default:     return 0x2580;  // 9600
    }
}

// ── Write-Versuche ────────────────────────────────────────────────────────────

// FC 0x06: Write Single Register (Reg 0x0101, nur Adresse)
bool tryFC06(uint8_t currentAddr, uint8_t newAddr) {
    uint8_t req[8];
    req[0] = currentAddr; req[1] = 0x06;
    req[2] = 0x01; req[3] = 0x01;   // Register 0x0101 = Device Address
    req[4] = 0x00; req[5] = newAddr;
    uint16_t c = crc16(req, 6);
    req[6] = c & 0xFF; req[7] = c >> 8;
    printHex("  TX", req, 8);
    while (rs485.available()) rs485.read();
    rs485Send(req, 8);
    uint8_t resp[16];
    uint8_t n = rs485Receive(resp, 16, 600, 8);
    if (n == 0) { logf("  -> Keine Antwort\n"); return false; }
    printHex("  RX", resp, n);
    if (resp[0] == currentAddr && resp[1] == 0x06) { logf("  -> Antwort OK\n"); return true; }
    if (resp[1] == 0x86) logf("  -> Exception: 0x%02X\n", resp[2]);
    return false;
}

// FC 0x10: Write 2 Registers gleichzeitig: Adresse (0x0101) + Baudrate (0x0102)
// Exakt wie Datenblatt Seite 13 Beispiel (dort: Adresse 0x20 + Baud 9600)
bool tryFC10(uint8_t currentAddr, uint8_t newAddr) {
    uint16_t baud = baudCode(BAUD_RATE);
    uint8_t req[13];
    req[0]  = currentAddr; req[1]  = 0x10;
    req[2]  = 0x01; req[3]  = 0x01;   // Start-Register 0x0101
    req[4]  = 0x00; req[5]  = 0x02;   // 2 Register schreiben
    req[6]  = 0x04;                    // 4 Bytes Nutzdaten
    req[7]  = 0x00; req[8]  = newAddr; // Register 0x0101: neue Adresse
    req[9]  = (baud >> 8); req[10] = (baud & 0xFF); // Register 0x0102: Baudrate
    uint16_t c = crc16(req, 11);
    req[11] = c & 0xFF; req[12] = c >> 8;
    printHex("  TX", req, 13);
    while (rs485.available()) rs485.read();
    rs485Send(req, 13);
    uint8_t resp[16];
    uint8_t n = rs485Receive(resp, 16, 600, 8);
    if (n == 0) { logf("  -> Keine Antwort\n"); return false; }
    printHex("  RX", resp, n);
    if (resp[0] == currentAddr && resp[1] == 0x10) { logf("  -> Antwort OK\n"); return true; }
    if (resp[1] == 0x90) logf("  -> Exception: 0x%02X\n", resp[2]);
    return false;
}

// ── Adresse ändern – mehrere Methoden, mit Verifikation ──────────────────────

bool changeAddress(uint8_t currentAddr, uint8_t newAddr) {
    logf("========================================\n");
    logf("Adressaenderung: 0x%02X -> 0x%02X\n", currentAddr, newAddr);
    logf("========================================\n");

    // Schritt 1: Sensor auf aktueller Adresse prüfen
    logf("[1] Pruefe Sensor auf Adresse 0x%02X ...\n", currentAddr);
    uint8_t curVerify = readAddress(currentAddr);
    if (curVerify == 0xFF) {
        logf("    WARNUNG: Sensor antwortet nicht auf 0x%02X!\n", currentAddr);
        logf("    Verkabelung pruefen oder Adresse bereits anders.\n");
    } else {
        logf("    OK Sensor gefunden, meldet Adresse: 0x%02X\n", curVerify);
    }

    // Schritt 2: Methoden nacheinander probieren
    struct {
        const char* desc;
        bool (*fn)(uint8_t, uint8_t);
    } methods[] = {
        { "FC06, Reg 0x0101 (Datenblatt Standard)",          tryFC06 },
        { "FC10, Reg 0x0101+0x0102 (Adresse+Baud, lt. DB)", tryFC10 },
    };

    for (uint8_t m = 0; m < 2; m++) {
        logf("[2.%d] Methode: %s\n", m+1, methods[m].desc);
        bool sent = methods[m].fn(currentAddr, newAddr);
        if (!sent) { logf("  -> Ueberspringe.\n"); continue; }

        logf("[3] Warte 1s auf EEPROM-Speicherung ...\n");
        delay(1000);

        // Verifikation direkt (ohne Power-Cycle)
        logf("[4a] Sofort-Verifikation auf 0x%02X ...\n", newAddr);
        float temp, hum;
        if (readTempHumidity(newAddr, temp, hum)) {
            logf("    OK Sensor antwortet sofort auf 0x%02X!\n", newAddr);
            logf("    Temp: %.1f C  |  Hum: %.1f %%RH\n", temp, hum);
            logf("========================================\n");
            logf("Adressaenderung mit Methode %d erfolgreich!\n", m+1);
            logf("-> CHANGE_ADDRESS auf false setzen, neu flashen.\n");
            logf("========================================\n");
            return true;
        }

        // Sensor braucht Power-Cycle -> Nutzer auffordern
        logf("    Sensor antwortet noch nicht auf 0x%02X\n", newAddr);
        logf("========================================\n");
        logf(">>> AKTION ERFORDERLICH: Sensor Strom trennen!\n");
        logf(">>> VCC-Kabel des Sensors abziehen und\n");
        logf(">>> wieder anschliessen (ca. 3 Sekunden warten).\n");
        logf(">>> Danach automatische Verifikation...\n");
        logf("========================================\n");

        // Warten bis Sensor nach Power-Cycle wieder antwortet (max 30s)
        logf("[4b] Warte auf Sensor nach Power-Cycle");
        uint32_t waitStart = millis();
        while (millis() - waitStart < 30000) {
            float t3, h3;
            if (readTempHumidity(newAddr, t3, h3)) {
                logf("\n");
                logf("    OK Sensor antwortet auf 0x%02X nach Power-Cycle!\n", newAddr);
                logf("    Temp: %.1f C  |  Hum: %.1f %%RH\n", t3, h3);
                logf("========================================\n");
                logf("Adressaenderung mit Methode %d + Power-Cycle erfolgreich!\n", m+1);
                logf("-> CHANGE_ADDRESS auf false setzen, neu flashen.\n");
                logf("========================================\n");
                return true;
            }
            logf(".");
            delay(1000);
        }
        logf("\n    Timeout. Sensor nicht gefunden nach Power-Cycle.\n");

        // Fallback: antwortet er noch auf alte Adresse?
        float t2, h2;
        if (readTempHumidity(currentAddr, t2, h2)) {
            logf("    -> Sensor noch auf 0x%02X (%.1f C). Naechste Methode ...\n",
                 currentAddr, t2);
        } else {
            logf("    -> Sensor antwortet auf keine Adresse. Bus-Problem?\n");
        }
    }

    logf("========================================\n");
    logf("FEHLER: Alle Methoden fehlgeschlagen!\n");
    logf("-> Stromversorgung, A/B-Polaritaet und\n");
    logf("   CHANGE_FROM_ADDR pruefen.\n");
    logf("========================================\n");
    return false;
}

// ── Alle Sensoren abfragen ────────────────────────────────────────────────────

void scanAllSensors() {
    const uint8_t total = SENSOR_ID_MAX - SENSOR_ID_MIN + 1;

    ui.scanning = true;
    ui.scanNo++;
    ui.found = 0;
    ui.bad   = 0;

    Serial.println("----------------------------------------");
    Serial.printf("Scan #%u  Adressen %d-%d:\n", ui.scanNo, SENSOR_ID_MIN, SENSOR_ID_MAX);

    for (uint8_t addr = SENSOR_ID_MIN; addr <= SENSOR_ID_MAX; addr++) {
        Sensor& s = sensors[addr];
        ui.cur = addr;
        s.st = ST_SCANNING;
        render();

        float t, h;
        if (readTempHumidity(addr, t, h)) {
            s.ok = true;
            s.seen = true;
            s.temp = t;
            s.hum  = h;
            s.st   = ST_OK;
            pushHist(s);
            ui.found++;
            Serial.printf("  [0x%02X] Temp: %.1f C  |  Hum: %.1f %%RH\n", addr, t, h);
        } else if (lastRxLen > 0) {
            s.ok = false;
            s.st = ST_BAD;
            ui.bad++;
            Serial.printf("  [0x%02X] fehlerhafte Antwort:", addr);
            for (uint8_t i = 0; i < lastRxLen; i++) Serial.printf(" %02X", lastRx[i]);
            Serial.println();
        } else {
            s.ok = false;
            s.st = s.seen ? ST_LOST : ST_NONE;
            Serial.printf("  [0x%02X] nicht erreichbar%s\n", addr, s.seen ? " (AUSGEFALLEN)" : "");
        }
        handleSerial();   // Screenshot auch waehrend des Scans moeglich
        delay(200);       // Pause zwischen Sensor-Anfragen
    }

    ui.scanning  = false;
    ui.firstDone = true;
    Serial.printf("Gefunden: %d/%d Sensoren", ui.found, total);
    if (ui.bad) Serial.printf("  |  %d fehlerhafte Antwort(en)", ui.bad);
    Serial.println("\n----------------------------------------\n");
    render();
}

// ── Touch statt Button ────────────────────────────────────────────────────────

void waitForTouch(const char* msg) {
    logf("%s\n", msg);
    logf(">>> Display beruehren (oder Taste im Monitor) zum Fortfahren...\n");
    // Falls noch beruehrt: erst loslassen
    do { M5.update(); delay(10); } while (M5.Touch.getCount() > 0);
    while (Serial.available()) Serial.read();
    while (true) {
        M5.update();
        if (M5.Touch.getDetail().wasReleased()) break;
        if (Serial.available()) { while (Serial.available()) Serial.read(); break; }
        delay(10);
    }
    logf(">>> Fahre fort.\n");
}

// ── Setup & Loop ──────────────────────────────────────────────────────────────

void setup() {
    auto cfg = M5.config();
    cfg.serial_baudrate = 115200;
    cfg.output_power    = true;    // 5V auf M5-Bus fuer das COMMU-Modul
    cfg.internal_spk    = false;   // G13 (I2S) liegt am M5-Bus Pin 23 (COMMU CAN_INT)
    cfg.internal_mic    = false;
    M5.begin(cfg);
    delay(500);

    // MCP2515 auf dem COMMU nicht benutzt -> CS high, damit es den SPI-Bus nicht stoert
    pinMode(COMMU_CAN_CS, OUTPUT);
    digitalWrite(COMMU_CAN_CS, HIGH);

    M5.Display.fillScreen(TFT_BLACK);
    Serial.printf("\n=== XY-MD04 Multi-Sensor - M5Stack CoreS3 + COMMU | %s (%s) ===\n",
                  FW_VERSION, FW_DATE);

    // Vollbild-Sprite (150 KB) bevorzugt im PSRAM
    screen.setColorDepth(16);
    screen.setPsram(true);
    if (!screen.createSprite(M5.Display.width(), M5.Display.height())) {
        screen.setPsram(false);
        if (!screen.createSprite(M5.Display.width(), M5.Display.height()))
            Serial.println("FEHLER: Display-Sprite konnte nicht angelegt werden!");
    }

#if RS485_DE_RE >= 0
    pinMode(RS485_DE_RE, OUTPUT);
    digitalWrite(RS485_DE_RE, LOW);
#endif
    rs485.begin(BAUD_RATE, SERIAL_8N1, RS485_RX, RS485_TX);

#if CHANGE_ADDRESS
    // Display als scrollende Textkonsole fuer die Diagnose-Ausgaben
    lcdConsole = true;
    M5.Display.setFont(&fonts::Font2);
    M5.Display.setTextDatum(top_left);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setTextScroll(true);
    M5.Display.setCursor(0, 0);

    delay(1000);
    bool ok = changeAddress(CHANGE_FROM_ADDR, CHANGE_TO_ADDR);
    if (ok) {
        waitForTouch("\n========================================\n"
                     "Adressaenderung erfolgreich!\n"
                     "Starte jetzt Sensor-Scan...\n"
                     "========================================");
    } else {
        waitForTouch("\n========================================\n"
                     "Adressaenderung fehlgeschlagen.\n"
                     "Scan wird trotzdem gestartet.\n"
                     "========================================");
    }

    lcdConsole = false;
    M5.Display.setTextScroll(false);
    M5.Display.fillScreen(TFT_BLACK);
#endif

    Serial.printf("Baud: %d | Scan-Bereich: ID %d-%d\n", BAUD_RATE, SENSOR_ID_MIN, SENSOR_ID_MAX);
    Serial.println("Befehle: 'r' = neu scannen, 's' = Screenshot\n");
    render();
}

void loop() {
    scanAllSensors();

    // Pause bis zum naechsten Durchlauf – Touch oder 'r' startet sofort einen neuen Scan
    uint32_t waitStart = millis();
    while (millis() - waitStart < READ_INTERVAL_MS) {
        M5.update();
        if (M5.Touch.getDetail().wasPressed()) break;
        if (handleSerial()) break;
        delay(20);
    }
}
