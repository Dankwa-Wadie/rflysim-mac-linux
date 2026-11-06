#!/usr/bin/env bash
# UdpPortFree (Windows UdpPortFree.bat): shows which programs hold RflySim's simulation ports, and frees them.
#   scripts/UdpPortFree.sh          list
#   scripts/UdpPortFree.sh --kill   stop the programs holding them
# Ports: TCP 4560-4609 (PX4 SITL), UDP 14550 (QGC), 20009-20029 (RflySim3D, 20009 multicast), 20100-20199 (Python/SIL),
#        30100-30199 (true state), 9999 (images), 5502 (FlightGear).
set -u
in_range() { local p=$1; [ "$p" -ge 4560 ] && [ "$p" -le 4609 ] || [ "$p" = 14550 ] || { [ "$p" -ge 20009 ] && [ "$p" -le 20029 ]; } ||
  { [ "$p" -ge 20100 ] && [ "$p" -le 20199 ]; } || { [ "$p" -ge 30100 ] && [ "$p" -le 30199 ]; } || [ "$p" = 9999 ] || [ "$p" = 5502 ]; }
command -v lsof >/dev/null || { echo "UdpPortFree needs lsof"; exit 1; }
found=""
while read -r cmd pid proto addr; do
  port="${addr##*:}"; case "$port" in ''|*[!0-9]*) continue ;; esac
  in_range "$port" || continue
  printf '  %-6s %-5s %-22s %s (pid %s)\n' "$proto" "$port" "$addr" "$cmd" "$pid"
  found="$found $pid"
done < <(lsof -nP -iUDP -iTCP -sTCP:LISTEN 2>/dev/null | awk 'NR > 1 { print $1, $2, $8, $9 }' | sort -u)
if [ -z "$found" ]; then echo "UdpPortFree: all simulation ports are free."; exit 0; fi
if [ "${1:-}" = --kill ]; then
  for p in $(echo "$found" | tr ' ' '\n' | sort -u); do kill "$p" 2>/dev/null; done
  sleep 1; echo "UdpPortFree: stopped the programs above."
else
  echo "Run scripts/UdpPortFree.sh --kill to stop them."
fi
