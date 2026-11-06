#!/usr/bin/env bash
# Python38Run (Windows Python38Run.bat): runs a Python example from the current folder with the RflySim
# Python environment (Python38Env).
#   Python38Run.sh                 the only .py in this folder, or asks which one
#   Python38Run.sh script.py ARGS  that script
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [ $# -eq 0 ]; then
  files=(*.py); [ -e "${files[0]}" ] || { echo "Python38Run: no .py file in $(pwd)"; exit 1; }
  if [ ${#files[@]} -eq 1 ]; then set -- "${files[0]}"
  else
    i=1; for f in "${files[@]}"; do echo "  $i  $f"; i=$((i + 1)); done
    read -rp "Run which script? " n; set -- "${files[$((n - 1))]}"
  fi
fi
exec "$ROOT/scripts/Python38Env.sh" "$@"
