# ===============================================================
# Makefile - LC29H RTK-System (Base + Rover + Rover-LCD)
# ===============================================================
#
# Ziele:
#   make            baut base_station, rover und rover-lcd
#   make base       nur die Basisstation
#   make rover      nur den Rover (Konsole)
#   make rover-lcd  Rover mit 20x4-LCD-Ausgabe (HD44780/PCF8574)
#   make install    kopiert nach /usr/local/bin (rtk-base, rtk-rover,
#                   rtk-rover-lcd) - benoetigt sudo
#   make clean      loescht die Binaries
#
# Hinweise:
#   -lm            Mathe-Bibliothek (sqrt, atan2, ... fuer
#                  ECEF<->WGS84-Umrechnung und Statistik)
#   -std=gnu11     C11 + GNU-Erweiterungen (usleep, strdup, ...)
#   -O2 -Wall -Wextra  Optimierung + alle Warnungen; der Code
#                  kompiliert warnungsfrei.
#
# Auf der Base wird nur base_station benoetigt, auf dem Rover
# rover bzw. rover-lcd. Es schadet aber nicht, ueberall alles
# zu bauen.
# ===============================================================

CC      = cc
CFLAGS  = -O2 -Wall -Wextra -std=gnu11
LDLIBS  = -lm
PREFIX  = /usr/local/bin

all: base_station rover rover-lcd

base: base_station

base_station: base_station.c
	$(CC) $(CFLAGS) -o base_station base_station.c $(LDLIBS)

rover: rover.c
	$(CC) $(CFLAGS) -o rover rover.c $(LDLIBS)

rover-lcd: rover-lcd.c
	$(CC) $(CFLAGS) -o rover-lcd rover-lcd.c $(LDLIBS)

install: all
	install -m 755 base_station $(PREFIX)/rtk-base
	install -m 755 rover        $(PREFIX)/rtk-rover
	install -m 755 rover-lcd    $(PREFIX)/rtk-rover-lcd

clean:
	rm -f base_station rover rover-lcd

.PHONY: all base install clean
