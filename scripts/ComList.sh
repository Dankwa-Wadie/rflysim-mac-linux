#!/usr/bin/env bash
# ComList: serial ports with their USB device names, flight controllers marked (Windows GetComList.exe).
#   scripts/ComList.sh              numbered list; the last line is "Recommended list: 1,2"
#   scripts/ComList.sh --recommended    only the recommended port paths, comma-separated (for scripts)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$ROOT/scripts/platform.sh"
FC_RE='PX4|Pixhawk|ArduPilot|CUAV|Holybro|CubePilot|Cube|3DR|mRo|Hex|FMU|Durandal|Kakute|Matek|SpeedyBee'

# Lines: "<port>\t<description>"
list_ports() {
  if [ "$RFLY_OS" = macos ]; then
    # ioreg: each USB device's product name appears before the serial device's callout path.
    ioreg -r -c IOUSBHostDevice -l -w0 2>/dev/null | awk -F'"' '
      /"USB Product Name" =/ { name = $4 }
      /"IOCalloutDevice" =/ { print $4 "\t" name }'
    for p in /dev/cu.usbmodem* /dev/cu.usbserial* /dev/cu.SLAB* /dev/cu.wchusbserial*; do
      [ -e "$p" ] && echo "$p	"
    done
  else
    for l in /dev/serial/by-id/*; do
      [ -e "$l" ] && printf '%s\t%s\n' "$(readlink -f "$l")" "$(basename "$l" | sed 's/^usb-//; s/-if[0-9]*$//; s/_/ /g')"
    done
    for p in /dev/ttyACM* /dev/ttyUSB*; do [ -e "$p" ] && echo "$p	"; done
  fi
}
# One line per port, keeping the first (named) entry.
PORTS="$(list_ports | awk -F'\t' '!seen[$1]++')"

if [ "${1:-}" = --recommended ]; then
  echo "$PORTS" | awk -F'\t' -v re="$FC_RE" '$1 != "" && $2 ~ re { printf "%s%s", sep, $1; sep = "," } END { print "" }'
  exit 0
fi
if [ -z "$PORTS" ]; then echo "Warning: there is no available serial port"; exit 0; fi
echo "Serial ports:"
echo "$PORTS" | awk -F'\t' -v re="$FC_RE" '$1 != "" { n++; printf "  %d  %-28s %s%s\n", n, $1, ($2 == "" ? "(unknown device)" : $2), ($2 ~ re ? "   <- flight controller" : "") }'
REC="$(echo "$PORTS" | awk -F'\t' -v re="$FC_RE" '$1 != "" { n++; if ($2 ~ re) { printf "%s%d", sep, n; sep = "," } } END { print "" }')"
echo "Recommended list: ${REC:-none (no flight controller recognised)}"
