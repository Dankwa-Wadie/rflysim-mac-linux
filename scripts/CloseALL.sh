#!/usr/bin/env bash
# CloseALL (Windows CloseALL.bat): stops every simulation process: SITLRun/HITLRun, coptersim, PX4 SITL,
# RflySim3D, FlightGear started by RflyTools, and QGroundControl (keep it with --keep-qgc).
set -u
KEEP_QGC=0; [ "${1:-}" = --keep-qgc ] && KEEP_QGC=1
names=(SITLRun.sh HITLRun.sh APMSITLRun.sh FlightGearDemo.sh coptersim "build/sitl/bin/ardu" rflysim3d "bin/px4( |$)" "fgfs.*F450|FlightGear.*F450")
[ $KEEP_QGC = 0 ] && names+=(QGroundControl)
stopped=0
for n in "${names[@]}"; do
  if pgrep -f "$n" >/dev/null 2>&1; then pkill -f "$n"; stopped=$((stopped + 1)); fi
done
sleep 2
for n in "${names[@]}"; do pgrep -f "$n" >/dev/null 2>&1 && pkill -9 -f "$n"; done
echo "CloseALL: done ($stopped kinds of process stopped)."
