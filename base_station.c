/*
 * base_station.c
 * ===============================================================
 * RTK-BASISSTATION  -  Quectel LC29H(BS) HAT am Raspberry Pi
 * ===============================================================
 *
 * AUFGABE
 *   Liest den RTCM3-Korrekturdatenstrom des LC29H(BS)-Moduls von
 *   der seriellen Schnittstelle (/dev/serial0, 115200 Baud),
 *   validiert jeden RTCM-Rahmen (Preamble 0xD3, Laenge, CRC-24Q)
 *   und sendet vollstaendige Rahmen als einzelne UDP-Datagramme
 *   an 1..8 Rover (IPv4, komma-separierte Liste).
 *   "1 RTCM-Rahmen = 1 UDP-Datagramm" - der Rover kann dadurch
 *   jedes Paket direkt und ohne Re-Framing ins Modul schreiben.
 *
 * FUNKTIONEN
 *   - Survey-In starten (-s) oder feste ECEF-Position setzen (-f)
 *   - $PQTMSVINSTATUS aktivieren + parsen  -> [SVIN]-Fortschritt
 *     (Achtung: ab Werk DEAKTIVIERT, auch auf alter Firmware
 *      A01S vorhanden, muss nur eingeschaltet werden!)
 *   - Automatische $PQTMCFGSVIN,R-Abfrage alle 30 s
 *     (liefert erst nach Abschluss des Survey-In echte Werte,
 *      vorher 0.0000)
 *   - Live-Dekodierung der gesendeten RTCM-1005-Nachricht
 *     -> [BASE-POS] mit ECEF + WGS84 (wandert waehrend Survey-In,
 *        friert nach Abschluss bzw. mit -f ein)
 *   - [STAT]-Zeile mit Rahmen-/Byte-Zaehler
 *
 * AUFRUF
 *   ./base_station <ip[,ip2,ip3...]> [optionen]
 *
 *   -p <port>        UDP-Zielport                (Default: 9250)
 *   -d <device>      UART-Device                 (Default: /dev/serial0)
 *   -b <baud>        Baudrate                    (Default: 115200)
 *   -s <dauer>,<acc> Survey-In: Dauer s, 3D-Acc-Limit m
 *                    Achtung: BEIDE Bedingungen muessen erfuellt
 *                    sein (Mindestdauer UND Genauigkeit <= Limit)
 *   -f <x>,<y>,<z>   Feste ECEF-Basisposition (m) - Modus 2
 *   -q               nur weiterleiten, KEINE Konfiguration senden
 *                    (kein PQTMSAVEPAR -> schont das Modul-Flash)
 *
 * TYPISCHER ABLAUF (siehe README.md)
 *   1) Einmalig:  ./base_station <ip> -s 3600,1.5
 *      -> warten bis [SVIN] Status=GUELTIG, ECEF X/Y/Z notieren
 *   2) Einmalig:  ./base_station <ip> -f X,Y,Z
 *      -> Modus 2 (Festposition) wird im NVM gespeichert
 *   3) Ab dann:   ./base_station <ip> -q
 *      WICHTIG: Solange Modus 1 (Survey-In) im NVM steht, laeuft
 *      der Survey-In bei JEDEM Neustart neu und die Referenz
 *      verschiebt sich um Meter!
 *
 * Build:  make            (siehe Makefile)
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
#include <termios.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define DEF_PORT     9250
#define DEF_DEVICE   "/dev/serial0"
#define DEF_BAUD     115200
#define RTCM_MAXLEN  1200          /* max. RTCM3 Payload = 1023 + Rahmen  */
#define STAT_PERIOD  5             /* Statistikausgabe alle n Sekunden    */

static volatile sig_atomic_t g_run = 1;
static void on_sigint(int sig) { (void)sig; g_run = 0; }

/* ---------------------------------------------------------------
 * CRC-24Q  (Qualcomm, Polynom 0x1864CFB) - Pruefsumme fuer RTCM3
 * --------------------------------------------------------------- */
static uint32_t crc24q(const uint8_t *buf, size_t len)
{
    uint32_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint32_t)buf[i]) << 16;
        for (int b = 0; b < 8; b++) {
            crc <<= 1;
            if (crc & 0x1000000) crc ^= 0x1864CFB;
        }
    }
    return crc & 0xFFFFFF;
}

/* ---------------------------------------------------------------
 * NMEA-Checksumme einer empfangenen Zeile pruefen ("$...*hh")
 * -> filtert binaeren RTCM-Muell aus der Konsolenausgabe
 * --------------------------------------------------------------- */
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
 * Bit-Extraktion aus RTCM-Payload (MSB zuerst)
 * --------------------------------------------------------------- */
static uint64_t getbits(const uint8_t *buf, unsigned pos, unsigned len)
{
    uint64_t v = 0;
    for (unsigned i = 0; i < len; i++) {
        v = (v << 1) | ((buf[(pos + i) / 8] >> (7 - (pos + i) % 8)) & 1);
    }
    return v;
}

static int64_t getbits_signed(const uint8_t *buf, unsigned pos, unsigned len)
{
    uint64_t v = getbits(buf, pos, len);
    if (v & (1ULL << (len - 1)))                  /* Vorzeichen erweitern */
        v |= ~((1ULL << len) - 1);
    return (int64_t)v;
}

/* ---------------------------------------------------------------
 * ECEF (m) -> WGS84 Lat/Lon/Hoehe  (iterativ)
 * --------------------------------------------------------------- */
#include <math.h>
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

/* ---------------------------------------------------------------
 * UART oeffnen und konfigurieren (8N1, raw)
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
 * NMEA/PQTM-Kommando mit berechneter Checksumme senden
 * z.B.  nmea_send(fd, "PQTMSAVEPAR")  ->  "$PQTMSAVEPAR*5A\r\n"
 * --------------------------------------------------------------- */
static int nmea_send(int fd, const char *body)
{
    uint8_t cs = 0;
    for (const char *p = body; *p; p++) cs ^= (uint8_t)*p;

    char line[256];
    int n = snprintf(line, sizeof(line), "$%s*%02X\r\n", body, cs);
    if (n <= 0 || n >= (int)sizeof(line)) return -1;

    printf("[TX-UART] %.*s\n", n - 2, line);
    ssize_t w = write(fd, line, (size_t)n);
    tcdrain(fd);
    usleep(150 * 1000);               /* Modul Zeit zum Antworten geben */
    return (w == n) ? 0 : -1;
}

/* ---------------------------------------------------------------
 * RTCM3-Parser (Zustandsautomat ueber Bytestrom)
 * Rahmen:  0xD3 | 6 Bit reserviert + 10 Bit Laenge | Payload | CRC24Q
 * --------------------------------------------------------------- */
typedef struct {
    uint8_t  buf[RTCM_MAXLEN];
    size_t   have;                 /* bereits gesammelte Bytes            */
    size_t   need;                 /* Gesamtlaenge des aktuellen Rahmens  */
} rtcm_parser_t;

typedef void (*rtcm_cb_t)(const uint8_t *frame, size_t len, void *ctx);

static void rtcm_feed(rtcm_parser_t *p, const uint8_t *data, size_t len,
                      rtcm_cb_t cb, void *ctx)
{
    for (size_t i = 0; i < len; i++) {
        uint8_t c = data[i];

        if (p->have == 0) {
            if (c != 0xD3) continue;          /* auf Preamble warten */
            p->buf[p->have++] = c;
            p->need = 0;
            continue;
        }
        p->buf[p->have++] = c;

        if (p->have == 3) {
            size_t plen = ((size_t)(p->buf[1] & 0x03) << 8) | p->buf[2];
            p->need = 3 + plen + 3;           /* Header + Payload + CRC */
            if (p->need > RTCM_MAXLEN) {      /* unplausibel -> reset   */
                p->have = 0;
                continue;
            }
        }

        if (p->need && p->have == p->need) {
            uint32_t want = ((uint32_t)p->buf[p->need-3] << 16) |
                            ((uint32_t)p->buf[p->need-2] <<  8) |
                             (uint32_t)p->buf[p->need-1];
            if (crc24q(p->buf, p->need - 3) == want) {
                cb(p->buf, p->need, ctx);
            } else {
                fprintf(stderr, "[RTCM] CRC-Fehler, Rahmen verworfen\n");
            }
            p->have = 0;
            p->need = 0;
        }
    }
}

/* ---------------------------------------------------------------
 * Kontext + Callback: RTCM-Rahmen per UDP an 1..n Rover (IPv4)
 * --------------------------------------------------------------- */
#define MAX_DST 8

typedef struct {
    struct sockaddr_in  sa;
    char                name[24];
    bool                warned;
} dest_t;

typedef struct {
    int                 sock;
    dest_t              dst[MAX_DST];
    int                 ndst;
    uint64_t            frames;
    uint64_t            bytes;
    time_t              last_stat;
    /* Basisposition aus der gesendeten RTCM-1005-Nachricht */
    bool                have_pos;
    double              ecef_x, ecef_y, ecef_z;
} udp_ctx_t;

static void send_rtcm_udp(const uint8_t *frame, size_t len, void *vctx)
{
    udp_ctx_t *u = (udp_ctx_t *)vctx;

    for (int i = 0; i < u->ndst; i++) {
        dest_t *d = &u->dst[i];
        if (sendto(u->sock, frame, len, 0,
                   (struct sockaddr *)&d->sa, sizeof(d->sa)) < 0) {
            if (!d->warned) {
                fprintf(stderr, "[UDP] sendto %s: %s\n",
                        d->name, strerror(errno));
                d->warned = true;
            }
        } else {
            d->warned = false;
        }
    }

    u->frames++;
    u->bytes += len;

    /* RTCM-Nachrichtentyp: 12 Bit am Payload-Anfang */
    unsigned type = ((unsigned)frame[3] << 4) | (frame[4] >> 4);

    /* Typ 1005: Antennen-Referenzposition (ECEF, 0.1 mm Aufloesung)
     * Das ist exakt die Position, die der Rover als Basis verwendet. */
    if (type == 1005 && len >= 3 + 19 + 3) {
        const uint8_t *pl = frame + 3;
        double x = (double)getbits_signed(pl,  34, 38) * 1e-4;
        double y = (double)getbits_signed(pl,  74, 38) * 1e-4;
        double z = (double)getbits_signed(pl, 114, 38) * 1e-4;
        bool changed = !u->have_pos ||
                       fabs(x - u->ecef_x) > 0.01 ||
                       fabs(y - u->ecef_y) > 0.01 ||
                       fabs(z - u->ecef_z) > 0.01;
        u->ecef_x = x; u->ecef_y = y; u->ecef_z = z;

        if (x == 0.0 && y == 0.0 && z == 0.0) {
            u->have_pos = false;      /* Survey-In noch nicht fertig */
        } else if (changed) {
            u->have_pos = true;
            double lat, lon, h;
            ecef2lla(x, y, z, &lat, &lon, &h);
            printf("[BASE-POS] gesendete Referenzposition (RTCM 1005):\n"
                   "           ECEF  X=%.4f  Y=%.4f  Z=%.4f m\n"
                   "           WGS84 %.8f\u00B0  %.8f\u00B0  h(ell)=%.3f m\n",
                   x, y, z, lat, lon, h);
        }
    }

    time_t now = time(NULL);
    if (now - u->last_stat >= STAT_PERIOD) {
        printf("[STAT] %llu RTCM-Rahmen, %llu Bytes gesendet "
               "(zuletzt Typ %u, %zu B)%s\n",
               (unsigned long long)u->frames,
               (unsigned long long)u->bytes, type, len,
               u->have_pos ? "" : "  [1005 noch ohne Position]");
        u->last_stat = now;
    }
}

/* ---------------------------------------------------------------
 * ASCII-Zeilen des Moduls mitlesen.
 * Es werden NUR Zeilen mit gueltiger NMEA-Checksumme ausgegeben -
 * binaere RTCM-Bytes, die zufaellig ein '$' enthalten, fallen weg.
 * --------------------------------------------------------------- */
static void print_svin_result(const char *line)
{
    /* $PQTMCFGSVIN,OK,<mode>,<dauer>,<acc>,<X>,<Y>,<Z>*hh */
    int mode; long dur; double acc, x, y, z;
    if (sscanf(line, "$PQTMCFGSVIN,OK,%d,%ld,%lf,%lf,%lf,%lf",
               &mode, &dur, &acc, &x, &y, &z) == 6
        && (x != 0.0 || y != 0.0 || z != 0.0)) {
        double lat, lon, h;
        ecef2lla(x, y, z, &lat, &lon, &h);
        printf("[SVIN]     Modus=%d  Dauer=%ld s  Acc-Limit=%.1f\n"
               "           ECEF  X=%.4f  Y=%.4f  Z=%.4f m\n"
               "           WGS84 %.8f\u00B0  %.8f\u00B0  h(ell)=%.3f m\n",
               mode, dur, acc, x, y, z, lat, lon, h);
    }
}

/* $PQTMSVINSTATUS,<Ver>,<TOW>,<Valid>,<Res0>,<Res1>,<Obs>,<CfgDur>,
 *                 <MeanX>,<MeanY>,<MeanZ>,<MeanAcc>*hh                */
static void print_svin_status(const char *line)
{
    char buf[256];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    char *star = strrchr(buf, '*');
    if (star) *star = '\0';

    char *f[16];
    int nf = 0;
    for (char *p = buf; p && nf < 16; ) {
        f[nf++] = p;
        p = strchr(p, ',');
        if (p) *p++ = '\0';
    }
    if (nf < 12) return;

    int    valid = atoi(f[3]);
    long   obs   = atol(f[6]);
    long   dur   = atol(f[7]);
    double x = atof(f[8]), y = atof(f[9]), z = atof(f[10]);
    double acc = atof(f[11]);

    /* Drosselung: nur alle 15 s ausgeben - außer der Status wechselt */
    static int    last_valid = -1;
    static time_t last_print = 0;
    time_t now = time(NULL);
    if (valid == last_valid && now - last_print < 15) return;
    last_valid = valid;
    last_print = now;

    const char *vs = (valid == 2) ? "GUELTIG (fertig!)" :
                     (valid == 1) ? "laeuft" : "ungueltig";

    if (x != 0.0 || y != 0.0 || z != 0.0) {
        double lat, lon, h;
        ecef2lla(x, y, z, &lat, &lon, &h);
        printf("[SVIN]     Status=%s  Beobachtungen=%ld/%ld  "
               "mittl. Genauigkeit=%.2f m\n"
               "           Mittel ECEF X=%.4f Y=%.4f Z=%.4f\n"
               "           Mittel WGS84 %.8f\u00B0 %.8f\u00B0 h=%.3f m\n",
               vs, obs, dur, acc, x, y, z, lat, lon, h);
    } else {
        printf("[SVIN]     Status=%s  Beobachtungen=%ld/%ld\n",
               vs, obs, dur);
    }
}

static void ascii_feed(const uint8_t *data, size_t len)
{
    static char line[256];
    static size_t pos = 0;
    static bool in_line = false;

    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '$') { in_line = true; pos = 0; line[pos++] = c; continue; }
        if (!in_line) continue;
        if (c == '\r' || c == '\n') {
            line[pos] = '\0';
            if (pos > 6 && nmea_check(line)) {
                if (strncmp(line, "$PQTMSVINSTATUS", 15) == 0) {
                    print_svin_status(line);     /* nur aufbereitet */
                } else {
                    printf("[RX-UART] %s\n", line);
                    if (strncmp(line, "$PQTMCFGSVIN,OK,", 16) == 0)
                        print_svin_result(line);
                }
            }
            in_line = false;
            continue;
        }
        if (pos < sizeof(line) - 1 && c >= 0x20 && c < 0x7F)
            line[pos++] = c;
        else
            in_line = false;      /* Binaerbyte -> keine NMEA-Zeile */
    }
}

/* --------------------------------------------------------------- */
static void usage(const char *prog)
{
    fprintf(stderr,
        "Aufruf: %s <ip[,ip2,ip3...]> [-p port] [-d device] [-b baud]\n"
        "           [-s dauer,acc] [-f x,y,z] [-q]\n"
        "  ip,ip2,...      1..%d Rover-Ziele (IPv4, komma-separiert)\n"
        "  -s 3600,1.5     Survey-In: 3600 s, Ziel-Genauigkeit 1.5 m\n"
        "  -f x,y,z        feste ECEF-Basisposition in Metern\n"
        "  -q              keine Konfiguration senden (nur Weiterleitung)\n",
        prog, MAX_DST);
}

int main(int argc, char **argv)
{
    const char *dev  = DEF_DEVICE;
    int         baud = DEF_BAUD;
    int         port = DEF_PORT;
    bool        quiet_cfg = false;
    long        svin_dur = 0;   double svin_acc = 0.0;
    bool        use_fixed = false;
    double      fx = 0, fy = 0, fz = 0;
    int opt;
    if (argc < 2) { usage(argv[0]); return 1; }
    const char *iplist = argv[1];

    optind = 2;
    while ((opt = getopt(argc, argv, "p:d:b:s:f:q")) != -1) {
        switch (opt) {
        case 'p': port = atoi(optarg); break;
        case 'd': dev  = optarg;       break;
        case 'b': baud = atoi(optarg); break;
        case 'q': quiet_cfg = true;    break;
        case 's':
            if (sscanf(optarg, "%ld,%lf", &svin_dur, &svin_acc) != 2) {
                usage(argv[0]); return 1;
            }
            break;
        case 'f':
            if (sscanf(optarg, "%lf,%lf,%lf", &fx, &fy, &fz) != 3) {
                usage(argv[0]); return 1;
            }
            use_fixed = true;
            break;
        default: usage(argv[0]); return 1;
        }
    }

    signal(SIGINT,  on_sigint);
    signal(SIGTERM, on_sigint);

    /* --- UDP-Socket + Ziel-Liste (IPv4, komma-separiert) --- */
    udp_ctx_t udp = {0};
    udp.sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp.sock < 0) { perror("socket"); return 1; }

    char list[256];
    snprintf(list, sizeof(list), "%s", iplist);
    for (char *tok = strtok(list, ","); tok && udp.ndst < MAX_DST;
         tok = strtok(NULL, ",")) {
        dest_t *d = &udp.dst[udp.ndst];
        memset(&d->sa, 0, sizeof(d->sa));
        d->sa.sin_family = AF_INET;
        d->sa.sin_port   = htons((uint16_t)port);
        if (inet_pton(AF_INET, tok, &d->sa.sin_addr) != 1) {
            fprintf(stderr, "Ungueltige Rover-IP: %s\n", tok);
            return 1;
        }
        snprintf(d->name, sizeof(d->name), "%s", tok);
        udp.ndst++;
        printf("[INFO] Sende RTCM3 per UDP an %s:%d\n", tok, port);
    }
    if (udp.ndst == 0) { usage(argv[0]); return 1; }
    udp.last_stat = time(NULL);

    /* --- UART --- */
    int ufd = uart_open(dev, baud);
    if (ufd < 0) return 1;
    printf("[INFO] UART %s @ %d Baud geoeffnet\n", dev, baud);

    /* --- Modul konfigurieren --- */
    if (!quiet_cfg) {
        nmea_send(ufd, "PQTMVERNO");                 /* Firmware anzeigen */
        /* Survey-In-Statusmeldung aktivieren (ab Werk deaktiviert!) */
        nmea_send(ufd, "PQTMCFGMSGRATE,W,PQTMSVINSTATUS,1,1");

        if (use_fixed) {
            char cmd[192];
            snprintf(cmd, sizeof(cmd),
                     "PQTMCFGSVIN,W,2,0,0,%.4f,%.4f,%.4f", fx, fy, fz);
            nmea_send(ufd, cmd);                      /* feste Position   */
            nmea_send(ufd, "PQTMSAVEPAR");
            printf("[INFO] Feste ECEF-Basisposition gesetzt.\n");
        } else if (svin_dur > 0) {
            char cmd[128];
            snprintf(cmd, sizeof(cmd),
                     "PQTMCFGSVIN,W,1,%ld,%.2f,0,0,0", svin_dur, svin_acc);
            nmea_send(ufd, cmd);                      /* Survey-In        */
            nmea_send(ufd, "PQTMSAVEPAR");
            printf("[INFO] Survey-In gestartet (%ld s / %.2f m). "
                   "Fortschritt: $PQTMSVINSTATUS-Meldungen beachten.\n",
                   svin_dur, svin_acc);
        } else {
            printf("[INFO] Keine Positions-Konfiguration angefordert "
                   "(-s oder -f). Modul laeuft mit gespeicherter Config.\n");
        }
    }

    /* --- Hauptschleife --- */
    rtcm_parser_t parser = {0};
    uint8_t rdbuf[2048];
    time_t  svin_query_t = time(NULL);
    bool    svin_polling = (svin_dur > 0 || use_fixed);

    while (g_run) {
        /* Waehrend/nach Survey-In regelmaessig Position abfragen */
        if (svin_polling && time(NULL) - svin_query_t >= 30) {
            nmea_send(ufd, "PQTMCFGSVIN,R");
            svin_query_t = time(NULL);
        }

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(ufd, &rfds);
        struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };

        int r = select(ufd + 1, &rfds, NULL, NULL, &tv);
        if (r < 0) {
            if (errno == EINTR) continue;
            perror("select");
            break;
        }
        if (r == 0) continue;

        ssize_t n = read(ufd, rdbuf, sizeof(rdbuf));
        if (n <= 0) continue;

        rtcm_feed(&parser, rdbuf, (size_t)n, send_rtcm_udp, &udp);
        ascii_feed(rdbuf, (size_t)n);
    }

    printf("\n[INFO] Beende. Gesamt: %llu Rahmen, %llu Bytes.\n",
           (unsigned long long)udp.frames, (unsigned long long)udp.bytes);
    close(udp.sock);
    close(ufd);
    return 0;
}
