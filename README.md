# LC29H RTK-GPS-System — Base + Rover im Eigenbau

RTK-GPS-System im Eigenbau: Zwei Raspberry Pi Zero 2 W mit Quectel-LC29H-Modulen
(Base + Rover), RTCM3-Korrekturen per UDP übers LAN. C-Programme für Survey-In,
Live-Dashboard, 20x4-LCD und Genauigkeitsstatistik.

**Erreichte Präzision (gemessen, nicht geschätzt):**

```
Genauigkeit:  horizontal ±0.1 cm (1 mm)   vertikal ±0.2 cm (2 mm)  [GST]
Streuung:     horizontal ±0.4 cm  vertikal ±0.5 cm  (empirisch, 300 s)
Abstand Base: horizontal 6.6 cm  — bei ~5 cm realem Antennenabstand
```

Ziel war < 3,0 cm — erreicht wurden **±4 mm** Wiederholgenauigkeit,
bestätigt durch zwei unabhängige Wege (GST-Sigmas des Moduls und
empirische Standardabweichung über 5 Minuten).

---

## Inhalt

1. [Hardware](#1-hardware)
2. [Raspberry-Pi-Vorbereitung (UART)](#2-raspberry-pi-vorbereitung-uart)
3. [Firmware-Update des Rovers (A03S → A04S)](#3-firmware-update-des-rovers-a03s--a04s)
4. [Bauen & Installieren](#4-bauen--installieren)
5. [Basisstation einrichten (Survey-In → Festposition)](#5-basisstation-einrichten)
6. [Rover betreiben](#6-rover-betreiben)
7. [Rover mit 20x4-LCD](#7-rover-mit-20x4-lcd)
8. [Mehrere Rover](#8-mehrere-rover)
9. [Kommandoreferenz LC29H (mit Checksummen)](#9-kommandoreferenz-lc29h)
10. [Erkenntnisse & Stolperfallen](#10-erkenntnisse--stolperfallen)
11. [Genauigkeit richtig bewerten](#11-genauigkeit-richtig-bewerten)
12. [Ausblick: 5/10 Hz](#12-ausblick-510-hz)
13. [Autostart nach dem Booten (systemd)](#13-autostart-nach-dem-booten-systemd)

---

## 1. Hardware

| Rolle  | Rechner              | GNSS-HAT                | Firmware (Stand) |
|--------|----------------------|-------------------------|------------------|
| Base   | Pi Zero 2 W          | Waveshare LC29H(**BS**) | LC29HBSNR11**A01S** (2023-02-13) |
| Rover  | Pi Zero 2 W          | Waveshare LC29H(**DA**) | LC29HDANR11**A04S**_RSA (Upgrade!) |

- Kommunikation Modul ↔ Pi: **UART** `/dev/serial0` @ **115200 Baud**
- Korrekturdaten Base → Rover: **UDP Port 9250** (IPv4-Unicast, LAN)
- Beispiel-IPs im Projekt: Base `192.168.1.197`, Rover `192.168.1.63`
- **I2C wird vom GNSS-Modul NICHT genutzt** (bei LC29H "reserved",
  vom HAT nicht herausgeführt) — der I2C-Bus bleibt frei fürs LCD.
  Die Adresse `0x50` bei `i2cdetect` ist nur das HAT-ID-EEPROM.

**Antennen (wichtig für cm-Genauigkeit):**
- freier Himmel, Abstand zu Wänden/Dächern (Multipath!)
- Ground Plane ≥ 20–30 cm (Blechplatte) unter jeder Antenne
- Basisantenne fest und erhöht montieren
- zwei Antennen nicht direkt nebeneinander (5 cm = Verkopplung);
  besser ≥ 30–50 cm Abstand

## 2. Raspberry-Pi-Vorbereitung (UART)

Auf **beiden** Pis den vollwertigen PL011-UART aktivieren
(die Mini-UART ist zu unzuverlässig):

```bash
# /boot/firmware/config.txt ergänzen:
enable_uart=1
dtoverlay=disable-bt

sudo systemctl disable hciuart
sudo reboot

# Kontrolle: serial0 muss auf ttyAMA0 zeigen (nicht ttyS0!)
ls -l /dev/serial0
#  -> /dev/serial0 -> ttyAMA0
```

In `/boot/firmware/cmdline.txt` darf **kein** `console=serial0,...`
stehen, sonst funkt die Login-Konsole in den GPS-Datenstrom.

**Manueller UART-Test** (Leser zuerst starten!):

```bash
stty -F /dev/serial0 115200 raw
timeout 5 cat /dev/serial0        # NMEA-Sätze müssen lesbar sein
```

Erscheint nur Zeichensalat: Baudrate/Config prüfen, ggf. neu booten.

## 3. Firmware-Update des Rovers (A03S → A04S)

Ohne dieses Update kein stabiles RTK FIXED und **kein GST**!

- Bezug: Quectel-Forum bzw. support@quectel.com
- Flashen mit **QGNSS unter Windows**, UART-Jumper des HATs auf
  **Position A** (direkter USB-UART-Durchgriff)
- A04S bringt laut Release Notes (2025-09-30): GST+ZDA-Ausgabe,
  konsistentes RTK-Fixed nach Kaltstart, Survey-In-Verbesserungen,
  Absturz-Fix bei defekten RTCM-Daten, Jahr-2038-Fix

Die **Base** läuft auch mit der alten A01S-Firmware zuverlässig
(inkl. `$PQTMSVINSTATUS`, siehe Abschnitt 10).

## 4. Bauen & Installieren

```bash
make              # baut base_station, rover, rover-lcd
make rover-lcd    # nur die LCD-Variante
sudo make install # optional: rtk-base / rtk-rover / rtk-rover-lcd
```

Alle Programme kompilieren mit `-Wall -Wextra` warnungsfrei.
Auf der Base wird nur `base_station` gebraucht, auf dem Rover
`rover` bzw. `rover-lcd`.

## 5. Basisstation einrichten

**Das Drei-Schritte-Rezept** (die Reihenfolge ist entscheidend):

### Schritt 1 — Survey-In (einmalig)

```bash
./base_station 192.168.1.63 -s 900,15.0     # Test: 15 min
./base_station 192.168.1.63 -s 3600,1.5     # produktiv: >= 1 h
```

- `[SVIN] Status=laeuft Beobachtungen=123/900 ...` zeigt den
  Live-Fortschritt (alle 15 s)
- **Beide** Bedingungen müssen erfüllt sein: Mindestdauer **und**
  Genauigkeit ≤ Acc-Limit — mit engem Limit kann es länger dauern
- Während des Survey-In **wandert** die gesendete 1005-Position
  (ungemittelte Momentanposition, ±Meter) — der Rover erreicht
  in dieser Phase kein sauberes Ergebnis. Das ist normal.
- Bei `Status=GUELTIG (fertig!)` die finalen **ECEF X/Y/Z**
  aus der `[SVIN]`-Zeile notieren.

Ergebnis dieses Projekts (900 s):

```
X = 4180005.5709   Y = 927119.3879   Z = 4712382.4004
(WGS84 47.93372725° N, 12.50567613° E, h_ell = 592.638 m)
```

### Schritt 2 — Festposition speichern (einmalig!)

```bash
./base_station 192.168.1.63 -f 4180005.5709,927119.3879,4712382.4004
```

Das schreibt **Modus 2 (Festposition)** ins NVM des Moduls.

> **WARUM DAS SO WICHTIG IST:** Solange Modus 1 (Survey-In) im NVM
> gespeichert ist, startet der Survey-In bei **jedem Neustart der
> Base neu** und endet bei einer leicht anderen Position (Meter!).
> Folgen: Rover-Position springt mit, Streuungsstatistik explodiert
> (im Projekt: scheinbar ±65 cm statt real ±0,4 cm).

### Schritt 3 — Regelbetrieb

```bash
./base_station 192.168.1.63 -q
```

`-q` = keine Konfiguration senden. Die Festposition ist im Modul
gespeichert; erneutes Senden würde nur unnötig ins Flash schreiben
(`$PQTMSAVEPAR` bei jedem Start = Flash-Verschleiß).

## 6. Rover betreiben

```bash
./rover           # erster Start: konfiguriert das Modul + speichert
./rover -q        # ab dann: Flash-schonend ohne Konfiguration
```

Dashboard-Zeilen und ihre Bedeutung:

| Zeile | Bedeutung |
|---|---|
| `Fix-Status` | Q=4 **RTK FIXED** (Ziel), Q=5 Float, Q=2 DGPS, Q=1 GPS |
| `Genauigkeit ... [GST]` | echte 1-Sigma-Werte des Moduls (bevorzugt) |
| `Genauigkeit ... [EPE]` | Fallback; **klebt bei dieser Firmware bei ~14,6 cm** — konservativer Schätzer, nicht die reale Präzision! |
| `Streuung` | empirische Standardabweichung der letzten ≤ 300 s, **nur Q=4-Epochen**, nur bei Stillstand aussagekräftig; Reset bei Fix-Verlust oder Basis-Verschiebung > 5 cm |
| `Abstand Base` | Distanz zur Basisantenne, live aus der RTCM-1005 dekodiert; N/O/H-Aufschlüsselung in cm. Höhenvergleich rechnet die Geoid-Separation ein (GGA=MSL, 1005=ellipsoidisch, Differenz ~47 m!) |
| `Letztes Paket` | Alter des letzten RTCM-Pakets; > 5 s = Link tot |

**Fahrplan zum Genauigkeitsnachweis:** Base mit Festposition laufen
lassen, Rover starten, auf Q=4 warten, Rover **5 Minuten stillhalten**,
Streuungszeile ablesen.

## 7. Rover mit 20x4-LCD

`rover-lcd` = `rover` + Anzeige auf HD44780-20x4 mit PCF8574-I2C-Backpack.

**Verkabelung (Pi Zero 2 W):**

| Backpack | Pi-Pin |
|---|---|
| VCC | Pin 2 (5V) |
| GND | Pin 6 |
| SDA | Pin 3 (GPIO 2) |
| SCL | Pin 5 (GPIO 3) |

Kein Konflikt mit dem GPS-HAT (UART, GPIO 14/15).

**Inbetriebnahme:**

```bash
sudo raspi-config          # Interface Options -> I2C -> Enable
sudo i2cdetect -y 1        # 0x27 oder 0x3F muss erscheinen
./rover-lcd -q
```

- Adresse wird automatisch gesucht (0x27, 0x3F); erzwingen mit
  `-a 0x3F`; PCF8574**A**-Backpacks nutzen teils 0x38–0x3E
- `-D` = ohne LCD (nur Konsole); fehlt das LCD, läuft das Programm
  mit Warnung normal weiter
- Nur schwarze Blöcke / nichts sichtbar → **Kontrast-Poti** auf dem
  Backpack drehen
- `0x50` in `i2cdetect` = EEPROM des GPS-HATs, ignorieren

LCD-Inhalt (1 Hz):

```
Q4 FIXED S38 H0.4        Fix, Satelliten, HDOP
GST H  0.4 V  0.5 cm     Genauigkeit
Base  6.6cm N +6 O -2    Abstand zur Base
20:24:23  Link  0.4s     UTC + RTCM-Paketalter ("Link TOT" = Base weg)
```

## 8. Mehrere Rover

Die Base sendet an 1..8 Ziele, **komma-separiert ohne Leerzeichen**:

```bash
./base_station 192.168.1.63,192.168.1.64,192.168.1.65 -q
```

Jedes Ziel wird beim Start bestätigt; eine ungültige IP bricht sofort
ab (vor dem UART-Open). Fällt ein Rover aus, warnt die Base einmalig
und bedient die übrigen weiter. Jeder Rover läuft unverändert.

## 9. Kommandoreferenz LC29H

Checksumme = XOR aller Zeichen zwischen `$` und `*`.
Manuell senden: Leser **zuerst** starten, dann `printf`:

```bash
stty -F /dev/serial0 115200 raw
timeout 8 cat /dev/serial0 | grep -E 'GST|MSGRATE|PAIR001' &
sleep 1
printf '$PQTMCFGMSGRATE,W,GST,1*0B\r\n' > /dev/serial0
wait
```

**Wichtige Kommandos (verifiziert an dieser Hardware):**

| Kommando | Wirkung |
|---|---|
| `$PQTMVERNO*58` | Firmware-Version abfragen |
| `$PQTMCFGSVIN,W,1,900,15.0,0,0,0*..` | Survey-In 900 s / 15 m |
| `$PQTMCFGSVIN,W,2,0,0,X,Y,Z*..` | Festposition (ECEF) |
| `$PQTMCFGSVIN,R*26` | Konfiguration/Ergebnis abfragen (liefert erst **nach** Abschluss echte Werte, vorher 0.0000) |
| `$PQTMCFGMSGRATE,W,PQTMSVINSTATUS,1,1*58` | Survey-In-Statusmeldung **einschalten** (ab Werk aus!) |
| `$PQTMCFGMSGRATE,W,GST,1*0B` | **GST einschalten (A04S)** — OHNE Versionsfeld! |
| `$PQTMCFGMSGRATE,W,GST,1,1*16` | → `ERROR,1` (falsche Syntax, dokumentiert als Negativbeispiel) |
| `$PQTMCFGMSGRATE,W,PQTMEPE,1,2*..` | EPE-Meldung einschalten |
| `$PQTMCFGNMEADP,W,3,8,3,2,3,2*..` | 8 Nachkommastellen in GGA |
| `$PAIR062,0,1*..` / `$PAIR062,3,0*..` | GGA ein / GSV aus |
| `$PAIR050,200*21` / `$PAIR050,100*22` | Fix-Rate 5 Hz / 10 Hz |
| `$PAIR864,0,0,460800*..` | UART-Baudrate (Reboot nötig!) |
| `$PQTMSAVEPAR*5A` | Konfiguration ins NVM (sparsam nutzen — Flash!) |
| `$PQTMRESTOREPAR*13` | Werkseinstellungen |

BS-spezifisch: `$PAIR432` (-1=aus/0=MSM4/1=MSM7), `$PAIR434`
(1005 ein/aus), `$PAIR436` (Ephemeriden). Die BS sendet ab Werk
MSM4 + 1005.

**GGA-Qualitätswerte:** 0=kein Fix, 1=GPS, 2=DGPS/SBAS,
4=**RTK Fixed**, 5=RTK Float.

## 10. Erkenntnisse & Stolperfallen

Chronologisch gesammelt aus der Entwicklung — damit nichts
zweimal gesucht werden muss:

1. **`$PQTMSVINSTATUS` ist ab Werk deaktiviert.** Deshalb kam nie
   eine Fortschrittsmeldung. Einschalten per
   `$PQTMCFGMSGRATE,W,PQTMSVINSTATUS,1,1` — funktioniert sogar auf
   der alten BS-Firmware A01S, obwohl erst in Spec v1.1 dokumentiert.
2. **`$PQTMCFGSVIN,R` liefert vor Abschluss nur Nullen.** Kein
   Fehler — erst nach fertigem Survey-In stehen echte ECEF-Werte drin.
3. **Die 1005 wandert während des Survey-In** (ungemittelte
   Momentanposition, ±Meter, Höhe ±mehrere m). Erst nach Abschluss
   bzw. mit `-f` friert sie ein.
4. **NVM-Falle Modus 1:** Gespeicherter Survey-In-Modus startet den
   Survey-In bei jedem Base-Neustart neu → Referenz verschiebt sich
   jedes Mal um Meter. Lösung: einmal `-f X,Y,Z` (→ Modus 2), danach
   nur noch `-q`.
5. **EPE ≠ echte Genauigkeit:** Der EPE-Wert der DA-Firmware klebt
   konstant bei ~14,6 cm, während die realen Koordinaten mm-stabil
   sind. GST-Sigmas + empirische Streuung sind die Wahrheit.
6. **GST-Syntax:** `$PQTMCFGMSGRATE,W,GST,1` (ohne Versionsfeld).
   Mit Versionsfeld (`...,1,1`) antwortet A04S mit `ERROR,1`.
   GST gibt es erst ab Firmware A04S.
7. **GST zeigt ±1–2 m?** Dann laufen gerade keine RTCM-Korrekturen
   (z. B. Rover-Programm gestoppt → Modul fällt auf DGPS zurück).
   Unter Q=4 fallen die Sigmas auf mm–cm.
8. **Streuungsstatistik verfälscht?** Ursachen waren: Float-Phasen
   im Puffer und Basis-Sprünge durch erneuten Survey-In. Der Code
   zählt daher nur noch Q=4 und resettet bei Basis-Verschiebung.
9. **Höhenvergleich Base↔Rover:** GGA liefert MSL, RTCM-1005
   ellipsoidisch — ohne Geoid-Separation (~47 m hier) läge der
   Höhenanteil völlig daneben. Der Rover rechnet das um.
10. **Flash schonen:** Jeder Start ohne `-q` sendet `$PQTMSAVEPAR`
    und schreibt ins Modul-NVM. Konfigurieren → einmal; Betrieb → `-q`.
11. **UART:** PL011 statt Mini-UART (`dtoverlay=disable-bt`),
    `hciuart` deaktivieren, `console=serial0` aus cmdline.txt
    entfernen. `serial0 -> ttyAMA0` ist das Ziel.
12. **I2C:** Der LC29H kann **nicht** über I2C ausgelesen werden
    (Schnittstelle "reserved", HAT führt sie nicht heraus) — I2C
    gehört exklusiv dem LCD. 115200 Baud UART sind bei 1 Hz zu
    < 10 % ausgelastet.
13. **Ein UDP-Datagramm = ein RTCM-Rahmen.** Dadurch braucht der
    Rover kein Re-Framing und einzelne Paketverluste kosten nur je
    einen Rahmen.

## 11. Genauigkeit richtig bewerten

Zwei getrennte Begriffe, die man nicht verwechseln darf:

- **Relative Präzision** (Rover relativ zur Base): das ist die
  RTK-Stärke — hier ±4 mm. Hängt **nur** davon ab, dass die
  Basisposition **stabil** ist, nicht davon, wie genau sie absolut
  stimmt.
- **Absolute Genauigkeit** (Position im WGS84-Weltrahmen): begrenzt
  durch die Qualität des Survey-In. 900 s Einzelpunkt-Mittelung ≈
  ±9 m absolut. Für bessere absolute Koordinaten: Survey-In über
  Nacht laufen lassen oder mit einem Referenzdienst (z. B. SAPOS)
  abgleichen. Für Streuungs- und Abstandsmessungen ist das egal.

Der Abstand Base↔Rover misst Phasenzentrum zu Phasenzentrum —
kleine Abweichungen zum Zollstockmaß (hier 6,6 cm vs. ~5 cm)
sind normal.

## 12. Ausblick: 5/10 Hz

Der LC29H(DA) unterstützt bis 10 Hz Fix-Rate (`$PAIR050,100`).
Die Base bleibt dabei bei 1 Hz RTCM — Korrekturen altern langsam,
der Rover rechnet dazwischen weiter (Standardverfahren).

Zu beachten bei Umstellung:
- ab 10 Hz UART auf 460800 Baud erhöhen (`$PAIR864`, Reboot,
  beidseitig!)
- Streuungspuffer im Rover-Code vergrößern oder auf 1 Hz dezimieren
  (aktuell 300 Samples = 5 min bei 1 Hz)
- LCD bleibt bei 1 Hz (I2C-Limit, ohnehin ausreichend)
- Für **statische** Messungen bringt mehr Rate nichts — lohnt nur
  für bewegte Trajektorien (z. B. Flächen im Gehen abfahren:
  1 Hz ≈ 1 Punkt/m, 5 Hz ≈ alle 20 cm bei Schrittgeschwindigkeit).

## 13. Autostart nach dem Booten (systemd)

Drei fertige Unit-Dateien liegen bei: `rtk-base.service`,
`rtk-rover.service`, `rtk-rover-lcd.service`.

**Installation** (Beispiel Base; Rover analog auf dem Rover-Pi):

```bash
sudo make install                       # Binaries nach /usr/local/bin
sudo cp rtk-base.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rtk-base    # sofort starten + bei Boot
```

**Bedienung:**

```bash
systemctl status rtk-base               # laeuft er?
journalctl -u rtk-base -f               # Live-Log ([SVIN], [STAT], ...)
sudo systemctl stop rtk-base            # z.B. fuer manuelle Tests
sudo systemctl disable rtk-base         # Autostart wieder aus
```

**Wichtige Punkte:**

- In `rtk-base.service` die **Rover-IP(s)** in der ExecStart-Zeile
  anpassen; alle Units setzen `-q` voraus (Konfiguration bereits
  im Modul-NVM gespeichert, siehe Kap. 5/6).
- Die Dienste laufen als Benutzer `sire` mit Zusatzgruppen
  `dialout` (UART) bzw. `i2c` (LCD) — anderen Benutzernamen ggf.
  in der Unit anpassen.
- `Restart=always` + `RestartSec=5` fangen sowohl Abstuerze als
  auch den Fall ab, dass der UART beim Booten noch nicht bereit war.
- Auf dem Rover nur **einen** der beiden Dienste aktivieren
  (`rtk-rover` **oder** `rtk-rover-lcd`) — beide zusammen streiten
  sich um `/dev/serial0`.
- Die Rover-Units leiten stdout nach `null` (das 1-Hz-Dashboard
  mit ANSI-Redraw wuerde das Journal fluten); Fehler erscheinen
  weiter im Journal. Zum Dashboard-Gucken: Dienst stoppen und
  `./rover -q` im Terminal starten — oder die LCD-Variante nutzen,
  dann ist das Display die Anzeige.

---

## Schnellreferenz

```bash
# ---- Base (nach einmaligem Survey-In + -f) ----
./base_station 192.168.1.63 -q
./base_station 192.168.1.63,192.168.1.64 -q      # zwei Rover

# ---- Rover ----
./rover -q
./rover-lcd -q                                    # mit 20x4-LCD

# ---- Einmalige Einrichtung Base ----
./base_station <ip> -s 3600,1.5                   # Survey-In
./base_station <ip> -f X,Y,Z                      # Ergebnis fixieren
```
