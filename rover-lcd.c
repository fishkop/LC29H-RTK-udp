/*
 * rover-lcd.c
 * ===============================================================
 * RTK-ROVER MIT 20x4-LCD  -  wie rover.c, zusaetzlich Anzeige
 * auf einem HD44780-Display mit PCF8574-I2C-Backpack
 * ===============================================================
 *
 * Identisch zu rover.c (Konsolen-Dashboard bleibt erhalten),
 * zusaetzlich wird 1x pro Sekunde ein 4-zeiliges Kurz-Dashboard
 * auf das LCD geschrieben:
 *
 *   +--------------------+
 *   |Q4 FIXED S38 H0.4   |   Fix-Status, Satelliten, HDOP
 *   |GST H  0.4 V  0.5 cm|   Genauigkeit (GST, sonst EPE)
 *   |Base  6.6cm N +6 O -2|  Abstand zur Basisantenne
 *   |20:24:23  Link  0.4s|   UTC + Alter des letzten RTCM-Pakets
 *   +--------------------+
 *
 * HARDWARE / VERKABELUNG (Pi Zero 2 W)
 *   Backpack VCC -> Pin 2 (5V)
 *   Backpack GND -> Pin 6 (GND)
 *   Backpack SDA -> Pin 3 (GPIO 2, SDA1)
 *   Backpack SCL -> Pin 5 (GPIO 3, SCL1)
 *   Kein Konflikt mit dem GPS-HAT (der nutzt UART GPIO 14/15).
 *   Kontrast-Poti auf dem Backpack ggf. nachstellen!
 *
 * INBETRIEBNAHME
 *   sudo raspi-config     -> Interface Options -> I2C -> Enable
 *   sudo i2cdetect -y 1   -> Adresse 0x27 oder 0x3F muss erscheinen
 *                            (0x50 = EEPROM des GPS-HATs, ignorieren)
 *
 * PCF8574-BITBELEGUNG (uebliche Backpacks)
 *   P0=RS  P1=RW  P2=E  P3=Backlight  P4..P7=D4..D7
 *   Das Display wird im 4-Bit-Modus initialisiert.
 *   Zeilenadressen 20x4: 0x00, 0x40, 0x14, 0x54.
 *
 * ZUSATZ-OPTIONEN (gegenueber rover.c)
 *   -a <addr>   I2C-Adresse erzwingen, z.B. -a 0x3F
 *               (Default: 0x27 und 0x3F automatisch probieren;
 *                PCF8574A-Backpacks nutzen teils 0x38..0x3E)
 *   -D          LCD deaktivieren, nur Konsole
 *   Ohne gefundenes LCD laeuft das Programm normal weiter.
 *
 * Build:  make rover-lcd  (siehe Makefile)
 * ===============================================================
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <math.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define DEF_PORT    9250
#define DEF_DEVICE  "/dev/serial0"
#define DEF_BAUD    115200
#define LINK_TIMEOUT 5.0            /* s ohne UDP-Paket -> "getrennt"   */

static volatile sig_atomic_t g_run = 1;
static void on_sigint(int sig) { (void)sig; g_run = 0; }

/* ---------------------------------------------------------------
 * Monotone Zeit in Sekunden
 * --------------------------------------------------------------- */
static double mono_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

/* ---------------------------------------------------------------
 * UART (identisch zur Basisstation)
 * --------------------------------------------------------------- */
static speed_t baud_to_speed(int baud)
{
    switch (baud) {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 921600:  return B921600;
    default:      return 0;
    }
}

static int uart_open(const char *dev, int baud)
{
    speed_t sp = baud_to_speed(baud);
    if (sp == 0) {
        fprintf(stderr, "Baudrate %d wird nicht unterstuetzt\n", baud);
        return -1;
    }
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "Kann %s nicht oeffnen: %s\n", dev, strerror(errno));
        return -1;
    }
    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) { close(fd); return -1; }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);

    if (tcsetattr(fd, TCSANOW, &tio) != 0) { close(fd); return -1; }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

/* ---------------------------------------------------------------
 * Bit-Extraktion aus RTCM-Payload (MSB zuerst) und
 * ECEF (m) -> WGS84 - fuer die Dekodierung der Basisposition (1005)
 * --------------------------------------------------------------- */
static uint64_t getbits(const uint8_t *buf, unsigned pos, unsigned len)
{
    uint64_t v = 0;
    for (unsigned i = 0; i < len; i++)
        v = (v << 1) | ((buf[(pos + i) / 8] >> (7 - (pos + i) % 8)) & 1);
    return v;
}

static int64_t getbits_signed(const uint8_t *buf, unsigned pos, unsigned len)
{
    uint64_t v = getbits(buf, pos, len);
    if (v & (1ULL << (len - 1)))
        v |= ~((1ULL << len) - 1);
    return (int64_t)v;
}

static void ecef2lla(double x, double y, double z,
                     double *lat, double *lon, double *h)
{
    const double a = 6378137.0, f = 1.0 / 298.257223563;
    const double e2 = f * (2.0 - f);
    double p = sqrt(x * x + y * y);
    double phi = atan2(z, p * (1.0 - e2));
    for (int i = 0; i < 6; i++) {
        double s  = sin(phi);
        double N  = a / sqrt(1.0 - e2 * s * s);
        double hh = p / cos(phi) - N;
        phi = atan2(z, p * (1.0 - e2 * N / (N + hh)));
    }
    double s = sin(phi);
    double N = a / sqrt(1.0 - e2 * s * s);
    *lat = phi * 180.0 / M_PI;
    *lon = atan2(y, x) * 180.0 / M_PI;
    *h   = p / cos(phi) - N;
}

/* ===============================================================
 * 20x4-LCD (HD44780) am PCF8574-I2C-Backpack
 * Uebliche Verdrahtung: P0=RS P1=RW P2=E P3=Backlight P4..P7=D4..D7
 * =============================================================== */
#define LCD_COLS 20
#define LCD_ROWS 4

static int g_lcd_fd = -1;      /* -1 = kein LCD */

static void lcd_i2c(uint8_t b)
{
    if (g_lcd_fd >= 0 && write(g_lcd_fd, &b, 1) < 0) { /* still */ }
}

static void lcd_nibble(uint8_t nib, uint8_t flags)
{
    uint8_t b = (uint8_t)((nib << 4) | flags | 0x08);  /* 0x08=Backlight */
    lcd_i2c(b | 0x04);          /* E high  */
    usleep(1);
    lcd_i2c(b & ~0x04);         /* E low   */
    usleep(50);
}

static void lcd_cmd(uint8_t c)
{
    lcd_nibble(c >> 4, 0x00);
    lcd_nibble(c & 0x0F, 0x00);
    if (c == 0x01 || c == 0x02) usleep(2000);   /* clear/home brauchen laenger */
}

static void lcd_putc(char c)
{
    lcd_nibble((uint8_t)c >> 4, 0x01);          /* RS=1: Daten */
    lcd_nibble((uint8_t)c & 0x0F, 0x01);
}

/* Zeile z (0..3) komplett schreiben, auf 20 Zeichen aufgefuellt */
static void lcd_line(int z, const char *txt)
{
    static const uint8_t rowaddr[LCD_ROWS] = { 0x00, 0x40, 0x14, 0x54 };
    if (g_lcd_fd < 0 || z < 0 || z >= LCD_ROWS) return;
    lcd_cmd((uint8_t)(0x80 | rowaddr[z]));
    for (int i = 0; i < LCD_COLS; i++)
        lcd_putc(i < (int)strlen(txt) ? txt[i] : ' ');
}

/* LCD initialisieren. addr=0 -> 0x27 und 0x3F automatisch probieren.
 * Rueckgabe: verwendete Adresse oder 0 bei Misserfolg. */
static int lcd_init(const char *dev, int addr)
{
    int tries[3] = { addr, 0x27, 0x3F };
    for (int t = (addr ? 0 : 1); t < (addr ? 1 : 3); t++) {
        int fd = open(dev, O_RDWR);
        if (fd < 0) return 0;
        if (ioctl(fd, I2C_SLAVE, tries[t]) < 0) { close(fd); continue; }
        uint8_t probe = 0x08;
        if (write(fd, &probe, 1) != 1) { close(fd); continue; }  /* ACK? */
        g_lcd_fd = fd;
        /* 4-Bit-Initialisierung nach Datenblatt */
        usleep(50000);
        lcd_nibble(0x03, 0); usleep(4500);
        lcd_nibble(0x03, 0); usleep(4500);
        lcd_nibble(0x03, 0); usleep(150);
        lcd_nibble(0x02, 0);                    /* 4-Bit-Modus        */
        lcd_cmd(0x28);                          /* 2 Zeilen, 5x8      */
        lcd_cmd(0x0C);                          /* Display an, Cursor aus */
        lcd_cmd(0x06);                          /* Autoinkrement      */
        lcd_cmd(0x01);                          /* Clear              */
        return tries[t];
    }
    return 0;
}

/* NMEA-Kommando mit Checksumme senden */
static int nmea_send(int fd, const char *body)
{
    uint8_t cs = 0;
    for (const char *p = body; *p; p++) cs ^= (uint8_t)*p;

    char line[256];
    int n = snprintf(line, sizeof(line), "$%s*%02X\r\n", body, cs);
    if (n <= 0 || n >= (int)sizeof(line)) return -1;

    ssize_t w = write(fd, line, (size_t)n);
    tcdrain(fd);
    usleep(150 * 1000);
    return (w == n) ? 0 : -1;
}

/* NMEA-Checksumme einer empfangenen Zeile pruefen ("$...*hh") */
static bool nmea_check(const char *line)
{
    const char *star = strrchr(line, '*');
    if (line[0] != '$' || !star || strlen(star) < 3) return false;
    uint8_t cs = 0;
    for (const char *p = line + 1; p < star; p++) cs ^= (uint8_t)*p;
    unsigned want;
    if (sscanf(star + 1, "%2x", &want) != 1) return false;
    return cs == (uint8_t)want;
}

/* ---------------------------------------------------------------
 * Zustand der Anzeige
 * --------------------------------------------------------------- */
typedef struct {
    /* aus GGA */
    char    utc[16];
    double  lat, lon;          /* Dezimalgrad, +N/+E */
    bool    has_pos;
    int     quality;           /* 0..6 */
    int     sats;
    double  hdop;
    double  alt;               /* m ueber MSL */
    double  geoid_sep;         /* Geoid-Separation aus GGA (m) */
    double  t_gga;             /* Empfangszeit (monoton) */

    /* Basisposition aus RTCM 1005 (aus dem UDP-Strom dekodiert) */
    double  base_lat, base_lon, base_h;   /* WGS84, h ellipsoidisch */
    bool    has_base;
    double  t_base;

    /* aus GST */
    double  sig_lat, sig_lon, sig_alt;   /* Std.-Abw. in m */
    bool    has_gst;
    double  t_gst;

    /* aus PQTMEPE (Fallback, falls Firmware kein GST ausgibt) */
    double  epe_h, epe_v;                /* gesch. Fehler in m */
    bool    has_epe;
    double  t_epe;

    /* Firmware-Version aus $PQTMVERNO */
    char    fw[64];

    /* Ringpuffer der letzten 5 Minuten fuer empirische Streuung */
    #define POSBUF_N 300
    double  pb_lat[POSBUF_N], pb_lon[POSBUF_N], pb_alt[POSBUF_N];
    int     pb_count, pb_idx;

    /* RTCM-Link */
    double    t_udp;           /* letztes UDP-Paket */
    uint64_t  udp_pkts;
    uint64_t  udp_bytes;
    char      peer[96];
} state_t;

/* Dashboard-Kurzfassung auf das LCD schreiben (1 Hz) */
static void lcd_update(const state_t *st)
{
    if (g_lcd_fd < 0) return;
    char l[4][32];
    double now = mono_s();

    /* Zeile 1: Fix-Status + Satelliten + HDOP */
    const char *q;
    switch (st->quality) {
    case 4:  q = "FIXED"; break;
    case 5:  q = "FLOAT"; break;
    case 2:  q = "DGPS "; break;
    case 1:  q = "GPS  "; break;
    default: q = "-----"; break;
    }
    snprintf(l[0], sizeof(l[0]), "Q%d %s S%02d H%.1f",
             st->quality, q, st->sats, st->hdop);

    /* Zeile 2: Genauigkeit (GST bevorzugt, sonst EPE) in cm */
    if (st->has_gst && now - st->t_gst < 5.0) {
        double h = sqrt(st->sig_lat * st->sig_lat +
                        st->sig_lon * st->sig_lon) * 100.0;
        snprintf(l[1], sizeof(l[1]), "GST H%5.1f V%5.1f cm",
                 h, st->sig_alt * 100.0);
    } else if (st->has_epe && now - st->t_epe < 5.0) {
        snprintf(l[1], sizeof(l[1]), "EPE H%5.1f V%5.1f cm",
                 st->epe_h * 100.0, st->epe_v * 100.0);
    } else {
        snprintf(l[1], sizeof(l[1]), "Genauigkeit: ---");
    }

    /* Zeile 3: Abstand zur Base */
    if (st->has_base && st->has_pos && now - st->t_gga < 5.0) {
        double m_lon = 111320.0 * cos(st->base_lat * M_PI / 180.0);
        double dn = (st->lat - st->base_lat) * 111320.0;
        double de = (st->lon - st->base_lon) * m_lon;
        double d2 = sqrt(dn * dn + de * de);
        if (d2 < 1.0)
            snprintf(l[2], sizeof(l[2]), "Base %4.1fcm N%+3.0f O%+3.0f",
                     d2 * 100.0, dn * 100.0, de * 100.0);
        else
            snprintf(l[2], sizeof(l[2]), "Base %7.2f m", d2);
    } else {
        snprintf(l[2], sizeof(l[2]), "Base: keine 1005");
    }

    /* Zeile 4: UTC + Link-Alter */
    if (st->udp_pkts > 0) {
        double age = now - st->t_udp;
        if (age < 99.0)
            snprintf(l[3], sizeof(l[3]), "%.8s  Link %4.1fs",
                     st->utc[0] ? st->utc : "--:--:--", age);
        else
            snprintf(l[3], sizeof(l[3]), "%.8s  Link TOT",
                     st->utc[0] ? st->utc : "--:--:--");
    } else {
        snprintf(l[3], sizeof(l[3]), "%.8s  warte UDP",
                 st->utc[0] ? st->utc : "--:--:--");
    }

    for (int z = 0; z < LCD_ROWS; z++)
        lcd_line(z, l[z]);
}


static const char *quality_name(int q)
{
    switch (q) {
    case 0:  return "KEIN FIX";
    case 1:  return "GPS (autonom)";
    case 2:  return "DGPS";
    case 4:  return "RTK FIXED";
    case 5:  return "RTK FLOAT";
    case 6:  return "Koppelnavigation";
    default: return "unbekannt";
    }
}

/* "ddmm.mmmmm" -> Dezimalgrad */
static double dm_to_deg(const char *s, char hemi)
{
    if (!s || !*s) return 0.0;
    double v = atof(s);
    double deg = floor(v / 100.0);
    double min = v - deg * 100.0;
    double d = deg + min / 60.0;
    if (hemi == 'S' || hemi == 'W') d = -d;
    return d;
}

/* Zeile in Felder zerlegen (zerstoerend), Anzahl zurueckgeben */
static int split_fields(char *line, char *fields[], int maxf)
{
    char *star = strrchr(line, '*');
    if (star) *star = '\0';
    int n = 0;
    char *p = line;
    while (n < maxf) {
        fields[n++] = p;
        char *c = strchr(p, ',');
        if (!c) break;
        *c = '\0';
        p = c + 1;
    }
    return n;
}

static void parse_nmea_line(state_t *st, const char *orig)
{
    if (!nmea_check(orig)) return;

    char line[256];
    strncpy(line, orig, sizeof(line) - 1);
    line[sizeof(line) - 1] = '\0';

    char *f[24];
    int nf = split_fields(line, f, 24);
    if (nf < 1) return;

    /* --- proprietaere Quectel-Meldungen --- */
    if (strcmp(f[0], "$PQTMVERNO") == 0 && nf >= 2) {
        strncpy(st->fw, f[1], sizeof(st->fw) - 1);
        st->fw[sizeof(st->fw) - 1] = '\0';
        return;
    }
    if (strcmp(f[0], "$PQTMEPE") == 0 && nf >= 7) {
        /* $PQTMEPE,2,<North>,<East>,<Down>,<2D>,<3D>  (alles Meter) */
        st->epe_h  = atof(f[5]);            /* horizontaler Fehler   */
        st->epe_v  = fabs(atof(f[4]));      /* vertikaler Fehler     */
        st->has_epe = (f[5][0] != '\0');
        st->t_epe  = mono_s();
        return;
    }

    const char *typ = f[0] + 3;                 /* "$GNGGA" -> "GGA" */
    if (strlen(f[0]) < 6) return;

    if (strcmp(typ, "GGA") == 0 && nf >= 10) {
        if (f[1][0]) {
            snprintf(st->utc, sizeof(st->utc), "%.2s:%.2s:%s",
                     f[1], f[1] + 2, f[1] + 4);
        }
        st->quality = atoi(f[6]);
        st->sats    = atoi(f[7]);
        st->hdop    = atof(f[8]);
        if (f[2][0] && f[4][0]) {
            st->lat = dm_to_deg(f[2], f[3][0]);
            st->lon = dm_to_deg(f[4], f[5][0]);
            st->alt = atof(f[9]);
            if (nf >= 12 && f[11][0]) st->geoid_sep = atof(f[11]);
            st->has_pos = true;
        } else {
            st->has_pos = false;
        }
        st->t_gga = mono_s();

        /* Position fuer Streuungsstatistik puffern (nur RTK Float/Fixed,
         * damit ein Fix-Wechsel die Statistik nicht verfaelscht) */
        if (st->has_pos && st->quality == 4) {   /* nur RTK FIXED */
            st->pb_lat[st->pb_idx] = st->lat;
            st->pb_lon[st->pb_idx] = st->lon;
            st->pb_alt[st->pb_idx] = st->alt;
            st->pb_idx = (st->pb_idx + 1) % POSBUF_N;
            if (st->pb_count < POSBUF_N) st->pb_count++;
        } else {
            st->pb_count = 0;          /* Fix verloren -> Statistik neu */
            st->pb_idx   = 0;
        }

        /* Basisposition verschoben (z.B. neuer Survey-In)?
         * -> Statistik neu, sonst misst sie den Sprung der Referenz. */
        if (st->has_base) {
            static double bl0, bo0, bh0; static bool binit;
            if (binit &&
                (fabs(st->base_lat - bl0) * 111320.0 > 0.05 ||
                 fabs(st->base_lon - bo0) * 111320.0 > 0.05 ||
                 fabs(st->base_h   - bh0)            > 0.05)) {
                st->pb_count = 0;
                st->pb_idx   = 0;
            }
            bl0 = st->base_lat; bo0 = st->base_lon; bh0 = st->base_h;
            binit = true;
        }
    }
    else if (strcmp(typ, "GST") == 0 && nf >= 9) {
        st->sig_lat = atof(f[6]);
        st->sig_lon = atof(f[7]);
        st->sig_alt = atof(f[8]);
        st->has_gst = (f[6][0] || f[7][0]);
        st->t_gst   = mono_s();
    }
}

/* NMEA-Bytestrom in Zeilen zerlegen */
static void nmea_feed(state_t *st, const uint8_t *data, size_t len)
{
    static char line[256];
    static size_t pos = 0;
    static bool in_line = false;

    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '$') { in_line = true; pos = 0; }
        if (!in_line) continue;
        if (c == '\r' || c == '\n') {
            if (pos > 6) {
                line[pos] = '\0';
                parse_nmea_line(st, line);
            }
            in_line = false;
            continue;
        }
        if (pos < sizeof(line) - 1 && c >= 0x20 && c < 0x7F)
            line[pos++] = c;
        else if (pos >= sizeof(line) - 1)
            in_line = false;
    }
}

/* ---------------------------------------------------------------
 * Genauigkeit huebsch formatieren: "±1.4 cm (14 mm)"
 * --------------------------------------------------------------- */
static void fmt_acc(char *out, size_t n, double meters)
{
    double cm = meters * 100.0;
    double mm = meters * 1000.0;
    if (mm < 100.0)
        snprintf(out, n, "\u00B1%.1f cm (%.0f mm)", cm, mm);
    else if (cm < 1000.0)
        snprintf(out, n, "\u00B1%.1f cm", cm);
    else
        snprintf(out, n, "\u00B1%.2f m", meters);
}

/* ---------------------------------------------------------------
 * Statusanzeige (ganzer Bildschirm, 1 Hz)
 * --------------------------------------------------------------- */
static void draw(const state_t *st, int port)
{
    double now = mono_s();

    printf("\033[H\033[J");   /* Cursor home + Bildschirm loeschen */
    printf("================ LC29H(DA)  RTK-ROVER ================\n");
    printf("  Firmware:     %s\n\n", st->fw[0] ? st->fw : "unbekannt");

    /* Position */
    if (st->has_pos && now - st->t_gga < 5.0) {
        printf("  Zeit (UTC):   %s\n", st->utc[0] ? st->utc : "-");
        printf("  Breite:       %+.8f\u00B0\n", st->lat);
        printf("  Laenge:       %+.8f\u00B0\n", st->lon);
        printf("  Hoehe (MSL):  %.3f m\n", st->alt);
    } else {
        printf("  Position:     --- (keine gueltige GGA-Meldung)\n");
    }

    /* Fix-Status */
    printf("\n  Fix-Status:   %s (Q=%d)   Satelliten: %d   HDOP: %.1f\n",
           quality_name(st->quality), st->quality, st->sats, st->hdop);

    /* Genauigkeit: GST bevorzugen, sonst PQTMEPE */
    if (st->has_gst && now - st->t_gst < 5.0) {
        double h = sqrt(st->sig_lat * st->sig_lat +
                        st->sig_lon * st->sig_lon);
        char hb[48], vb[48];
        fmt_acc(hb, sizeof(hb), h);
        fmt_acc(vb, sizeof(vb), st->sig_alt);
        printf("  Genauigkeit:  horizontal %s   vertikal %s  [GST]\n",
               hb, vb);
    } else if (st->has_epe && now - st->t_epe < 5.0) {
        char hb[48], vb[48];
        fmt_acc(hb, sizeof(hb), st->epe_h);
        fmt_acc(vb, sizeof(vb), st->epe_v);
        printf("  Genauigkeit:  horizontal %s   vertikal %s  [EPE]\n",
               hb, vb);
    } else {
        printf("  Genauigkeit:  --- (keine GST-/PQTMEPE-Meldung)\n");
    }

    /* Empirische Streuung (1 Sigma) der letzten <=5 min bei RTK-Fix.
     * Aussagekraeftig nur bei STILLSTEHENDEM Rover. */
    if (st->pb_count >= 30) {
        double mlat = 0, mlon = 0, malt = 0;
        for (int i = 0; i < st->pb_count; i++) {
            mlat += st->pb_lat[i]; mlon += st->pb_lon[i];
            malt += st->pb_alt[i];
        }
        mlat /= st->pb_count; mlon /= st->pb_count; malt /= st->pb_count;

        double m_per_deg_lat = 111320.0;
        double m_per_deg_lon = 111320.0 * cos(mlat * M_PI / 180.0);
        double vn = 0, ve = 0, vu = 0;
        for (int i = 0; i < st->pb_count; i++) {
            double dn = (st->pb_lat[i] - mlat) * m_per_deg_lat;
            double de = (st->pb_lon[i] - mlon) * m_per_deg_lon;
            double du =  st->pb_alt[i] - malt;
            vn += dn * dn; ve += de * de; vu += du * du;
        }
        double sn = sqrt(vn / st->pb_count) * 100.0;   /* cm */
        double se = sqrt(ve / st->pb_count) * 100.0;
        double sh = sqrt(sn * sn + se * se);
        double su = sqrt(vu / st->pb_count) * 100.0;
        printf("  Streuung:     horizontal \u00B1%.1f cm  vertikal \u00B1%.1f cm"
               "  (empirisch, %d s, nur bei Stillstand gueltig)\n",
               sh, su, st->pb_count);
    }

    /* Abstand zur Basisantenne (aus RTCM 1005 + eigener Position) */
    if (st->has_base && st->has_pos && now - st->t_gga < 5.0) {
        double m_lat = 111320.0;
        double m_lon = 111320.0 * cos(st->base_lat * M_PI / 180.0);
        double dn = (st->lat - st->base_lat) * m_lat;      /* Nord  (m) */
        double de = (st->lon - st->base_lon) * m_lon;      /* Ost   (m) */
        double rover_h_ell = st->alt + st->geoid_sep;      /* MSL->Ell. */
        double du = rover_h_ell - st->base_h;              /* Hoehe (m) */
        double d2 = sqrt(dn * dn + de * de);
        double d3 = sqrt(d2 * d2 + du * du);

        char b2[48], b3[48];
        fmt_acc(b2, sizeof(b2), d2);
        fmt_acc(b3, sizeof(b3), d3);
        /* fmt_acc liefert "±..." - Vorzeichen hier nicht sinnvoll,
         * daher ab Zeichen nach dem ±-Symbol (UTF-8: 2 Bytes) ausgeben */
        printf("\n  Abstand Base: horizontal %s   3D %s\n", b2 + 2, b3 + 2);
        printf("                (N %+.1f cm, O %+.1f cm, H %+.1f cm)\n",
               dn * 100.0, de * 100.0, du * 100.0);
    }

    /* RTCM-Verbindungsstatus */
    printf("\n  ---------------- RTCM-Korrekturlink ----------------\n");
    if (st->udp_pkts == 0) {
        printf("  Status:       WARTE AUF DATEN (UDP-Port %d)\n", port);
    } else {
        double age = now - st->t_udp;
        if (age <= LINK_TIMEOUT)
            printf("  Status:       VERBUNDEN mit %s\n", st->peer);
        else
            printf("  Status:       GETRENNT (seit %.0f s keine Daten)\n", age);
        printf("  Letztes Paket: vor %.1f s\n", age);
        printf("  Empfangen:    %llu Pakete / %.1f KB",
               (unsigned long long)st->udp_pkts,
               (double)st->udp_bytes / 1024.0);
        printf("\n");
    }
    printf("\n  (Beenden mit Strg+C)\n");
    fflush(stdout);
}

/* --------------------------------------------------------------- */
static void usage(const char *prog)
{
    fprintf(stderr,
        "Aufruf: %s [-p port] [-d device] [-b baud] [-q]\n"
        "           [-a 0x27] [-D]\n"
        "  -a <addr>   I2C-Adresse des LCD (Default: 0x27/0x3F suchen)\n"
        "  -D          LCD deaktivieren (nur Konsole)\n", prog);
}

int main(int argc, char **argv)
{
    const char *dev  = DEF_DEVICE;
    int         baud = DEF_BAUD;
    int         port = DEF_PORT;
    bool        quiet_cfg = false;
    int         lcd_addr  = 0;          /* 0 = Autosuche */
    bool        lcd_off   = false;

    int opt;
    while ((opt = getopt(argc, argv, "p:d:b:qa:D")) != -1) {
        switch (opt) {
        case 'p': port = atoi(optarg); break;
        case 'd': dev  = optarg;       break;
        case 'b': baud = atoi(optarg); break;
        case 'q': quiet_cfg = true;    break;
        case 'a': lcd_addr = (int)strtol(optarg, NULL, 0); break;
        case 'D': lcd_off  = true;     break;
        default: usage(argv[0]); return 1;
        }
    }

    signal(SIGINT,  on_sigint);
    signal(SIGTERM, on_sigint);

    /* --- UART --- */
    int ufd = uart_open(dev, baud);
    if (ufd < 0) return 1;

    /* --- UDP-Empfangssocket (IPv4) --- */
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return 1; }
    int one = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons((uint16_t)port);
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return 1;
    }

    /* --- LCD initialisieren --- */
    if (!lcd_off) {
        int a = lcd_init("/dev/i2c-1", lcd_addr);
        if (a)
            printf("[INFO] LCD 20x4 auf /dev/i2c-1 Adresse 0x%02X\n", a);
        else
            fprintf(stderr, "[WARN] Kein LCD gefunden (0x27/0x3F) - "
                            "weiter ohne Display\n");
    }

    /* --- Rover-Modul konfigurieren --- */
    if (!quiet_cfg) {
        nmea_send(ufd, "PQTMVERNO");              /* Firmware-Version    */
        nmea_send(ufd, "PAIR062,0,1");            /* GGA  ein, 1 Hz      */
        nmea_send(ufd, "PAIR062,8,1");            /* GST (aeltere FW)    */
        nmea_send(ufd, "PQTMCFGMSGRATE,W,GST,1");     /* GST ab A04S     */
        nmea_send(ufd, "PAIR062,3,0");            /* GSV  aus (Bandbr.)  */
        nmea_send(ufd, "PQTMCFGMSGRATE,W,PQTMEPE,1,2"); /* EPE ein, 1 Hz */
        nmea_send(ufd, "PQTMCFGNMEADP,W,3,8,3,2,3,2"); /* mehr Dezimalen */
        nmea_send(ufd, "PQTMSAVEPAR");            /* speichern           */
    }

    state_t st = {0};
    uint8_t buf[2048];
    double  last_draw = 0.0;

    int maxfd = (ufd > sock ? ufd : sock) + 1;

    while (g_run) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(ufd,  &rfds);
        FD_SET(sock, &rfds);
        struct timeval tv = { .tv_sec = 0, .tv_usec = 200 * 1000 };

        int r = select(maxfd, &rfds, NULL, NULL, &tv);
        if (r < 0) {
            if (errno == EINTR) continue;
            perror("select");
            break;
        }

        /* UDP -> UART (RTCM-Korrekturen ins Modul schreiben) */
        if (r > 0 && FD_ISSET(sock, &rfds)) {
            struct sockaddr_in src;
            socklen_t slen = sizeof(src);
            ssize_t n = recvfrom(sock, buf, sizeof(buf), 0,
                                 (struct sockaddr *)&src, &slen);
            if (n > 0) {
                ssize_t off = 0;
                while (off < n) {
                    ssize_t w = write(ufd, buf + off, (size_t)(n - off));
                    if (w < 0) {
                        if (errno == EAGAIN) { usleep(2000); continue; }
                        break;
                    }
                    off += w;
                }
                st.udp_pkts++;
                st.udp_bytes += (uint64_t)n;
                st.t_udp = mono_s();
                snprintf(st.peer, sizeof(st.peer), "%s:%u",
                         inet_ntoa(src.sin_addr), ntohs(src.sin_port));

                /* Basisposition mitlesen: jedes UDP-Paket ist genau ein
                 * RTCM-Rahmen. Typ 1005 = Antennen-Referenzposition. */
                if (n >= 25 && buf[0] == 0xD3) {
                    unsigned type = ((unsigned)buf[3] << 4) | (buf[4] >> 4);
                    if (type == 1005) {
                        const uint8_t *pl = buf + 3;
                        double x = (double)getbits_signed(pl,  34, 38) * 1e-4;
                        double y = (double)getbits_signed(pl,  74, 38) * 1e-4;
                        double z = (double)getbits_signed(pl, 114, 38) * 1e-4;
                        if (x != 0.0 || y != 0.0 || z != 0.0) {
                            ecef2lla(x, y, z, &st.base_lat, &st.base_lon,
                                     &st.base_h);
                            st.has_base = true;
                            st.t_base   = mono_s();
                        }
                    }
                }
            }
        }

        /* UART -> NMEA-Parser (Position + Genauigkeit) */
        if (r > 0 && FD_ISSET(ufd, &rfds)) {
            ssize_t n = read(ufd, buf, sizeof(buf));
            if (n > 0) nmea_feed(&st, buf, (size_t)n);
        }

        /* Anzeige mit 1 Hz aktualisieren */
        double now = mono_s();
        if (now - last_draw >= 1.0) {
            draw(&st, port);
            lcd_update(&st);
            last_draw = now;
        }
    }

    if (g_lcd_fd >= 0) {
        lcd_cmd(0x01);
        lcd_line(0, "Rover beendet.");
        close(g_lcd_fd);
    }
    printf("\n[INFO] Beende Rover.\n");
    close(sock);
    close(ufd);
    return 0;
}
