#!/usr/bin/env bash
# Test the RflyMac MATLAB package's C client + reference controller against `coptersim --ext`.
# Runs without MATLAB. Exit code 0 on success.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$(mktemp -d)"
SIM="$ROOT/target/release/coptersim"
[ -x "$SIM" ] || (cd "$ROOT" && cargo build --release) || exit 1

clang -std=c99 -O2 -Wall -Wextra -Werror -I"$ROOT/matlab/RflyMac/src" \
  "$ROOT/matlab/RflyMac/src/rflymac_client.c" "$ROOT/matlab/RflyMac/tests/test_client.c" \
  -o "$OUT/test_client" -lm || exit 1

"$SIM" --ext 20100 >"$OUT/coptersim.log" 2>&1 &
SIM_PID=$!
trap 'kill $SIM_PID 2>/dev/null' EXIT
sleep 0.5
"$OUT/test_client"
