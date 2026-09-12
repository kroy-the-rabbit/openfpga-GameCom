#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
BDIR="$REPO/build/gamecom"
PY="$BDIR/venv/bin/python3"
[[ -x "$PY" ]] || PY=python3
"$PY" "$REPO/scripts/report.py" "$BDIR"
