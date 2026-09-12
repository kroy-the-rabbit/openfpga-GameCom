#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
BDIR="$REPO/build/gamecom"
WORK="$BDIR/work"

PODMAN=${PODMAN:-podman}
IMAGE=${IMAGE:-localhost/pocket-quartus:25.1std}
QUARTUS_BIN=${QUARTUS_BIN:-}
if [[ -n "$QUARTUS_BIN" ]]; then
  QUARTUS_BIN=$(realpath "$QUARTUS_BIN")
  [[ -x "$QUARTUS_BIN/quartus_sh" && -x "$QUARTUS_BIN/quartus_sta" ]] || {
    echo 'QUARTUS_BIN must contain quartus_sh and quartus_sta' >&2; exit 2; }
  IMAGE=native-quartus
fi

case "$(basename "$PODMAN"):$(id -u)" in
  docker:*) RUNAS=(--user "$(id -u):$(id -g)") ;;
  *:0)      RUNAS=(--security-opt label=disable) ;;
  *)        RUNAS=(--userns=keep-id --security-opt label=disable) ;;
esac

run_quartus() {
  local subdir=$1 program=$2
  shift 2
  if [[ -n "$QUARTUS_BIN" ]]; then
    (cd "$WORK$subdir" && "$QUARTUS_BIN/$program" "$@")
  else
    "$PODMAN" run --rm "${RUNAS[@]}" \
      -v "$WORK:/work" -w "/work$subdir" -e HOME=/tmp \
      "$IMAGE" "$program" "$@"
  fi
}

CORE_DIR=$(ls -d "$REPO/pkg/Cores"/*/ | head -1)
CORE_NAME=$(basename "$CORE_DIR")

VENV="$BDIR/venv"
[[ -x "$VENV/bin/python3" ]] || python3 -m venv "$VENV"
PY="$VENV/bin/python3"
PROVENANCE="$REPO/scripts/build_provenance.py"
BUILD_SETTINGS=(--setting "IMAGE=$IMAGE" --setting "SEED=${SEED:-}" \
  --setting "FITTER_EFFORT=${FITTER_EFFORT:-}" --setting "NPROC=${NPROC:-}")

if [[ -n "${SKIP_COMPILE:-}" ]]; then
  "$PY" "$PROVENANCE" verify "$REPO" "$BDIR" "${BUILD_SETTINGS[@]}"
else
  "$PY" "$PROVENANCE" begin "$REPO" "$BDIR" "${BUILD_SETTINGS[@]}"
fi

rm -f "$BDIR"/*.zip "$BDIR"/*.rbf_r
rm -f "$BDIR/package-manifest.json"
echo "== core=$CORE_NAME image=$IMAGE"

if [[ -z "${SKIP_COMPILE:-}" ]]; then
mkdir -p "$WORK"
rsync -a --delete \
  --exclude 'output_files/' --exclude 'db/' --exclude 'incremental_db/' \
  --exclude 'build_output/' \
  "$REPO/src/" "$WORK/src/"
rsync -a --delete "$REPO/scripts/" "$WORK/scripts/"
cp "$REPO/generate.tcl" "$WORK/generate.tcl"

QSF="$WORK/src/fpga/build/ap_core.qsf"

sed -i 's/^set_global_assignment -name NUM_PARALLEL_PROCESSORS 4$/set_global_assignment -name NUM_PARALLEL_PROCESSORS ALL/' "$WORK/generate.tcl"

if [[ -n "${FITTER_EFFORT:-}" ]]; then
  printf '\nset_global_assignment -name FITTER_EFFORT "%s"\n' "$FITTER_EFFORT" >> "$QSF"
  printf '\nset_global_assignment -name OPTIMIZE_HOLD_TIMING "ALL PATHS"\n' >> "$QSF"
  echo "== fitter effort $FITTER_EFFORT"
fi

if [[ -n "${NPROC:-}" ]]; then
  sed -i "s/^set_global_assignment -name NUM_PARALLEL_PROCESSORS .*$/set_global_assignment -name NUM_PARALLEL_PROCESSORS $NPROC/" "$WORK/generate.tcl"
  echo "== parallel processors $NPROC"
fi

if [[ -n "${SEED:-}" ]]; then
  printf '\nset_global_assignment -name SEED %s\n' "$SEED" >> "$QSF"
  echo "== fitter seed $SEED"
fi

  start=$(date +%s)
  set +e
  run_quartus '' quartus_sh -t generate.tcl 2>&1 | tee "$BDIR/build.log"
  rc=${PIPESTATUS[0]}
  set -e
  echo "$(( $(date +%s) - start ))" > "$BDIR/elapsed"
  [[ $rc -eq 0 ]] || { echo "quartus failed (rc=$rc), see $BDIR/build.log" >&2; exit "$rc"; }
  "$PY" "$PROVENANCE" seal "$REPO" "$BDIR"
  run_quartus /src/fpga/build quartus_sta -t ../../../scripts/inspect_timing.tcl \
    > "$BDIR/path-analysis.log" 2>&1
  run_quartus '' quartus_sh --version 2>/dev/null | sed -n 2p > "$BDIR/quartus.version" || true
else
  echo "== SKIP_COMPILE set, verified existing compiled snapshot without changing staged sources"
fi

BUILD_IDENTITY=$("$PY" "$PROVENANCE" identity "$REPO" "$BDIR")
read -r GIT_SHA GIT_DIRTY <<< "$BUILD_IDENTITY"
[[ "$GIT_DIRTY" == 1 ]] || GIT_DIRTY=""
echo "== compiled commit=$GIT_SHA${GIT_DIRTY:+ (dirty)}"
RBF="$WORK/src/fpga/build/output_files/ap_core.rbf"
test -f "$RBF" || { echo "no .rbf produced, see $BDIR/build.log" >&2; exit 1; }

GIT_SHA="$GIT_SHA" GIT_DIRTY="$GIT_DIRTY" "$HERE/report.sh"

RBF_NAME=$("$PY" -c "import json,sys;print(json.load(open(sys.argv[1]))['core']['cores'][0]['filename'])" "$CORE_DIR/core.json")
VERSION=$("$PY" -c "import json,sys;print(json.load(open(sys.argv[1]))['core']['metadata']['version'])" "$CORE_DIR/core.json")

"$PY" "$REPO/scripts/reverse_bitstream.py" "$RBF" "$BDIR/$RBF_NAME"

rm -rf "$BDIR/sd"
rsync -a "$REPO/pkg/" "$BDIR/sd/"
cp "$BDIR/$RBF_NAME" "$BDIR/sd/Cores/$CORE_NAME/$RBF_NAME"

STAMP="${RELEASE_NAME:-}"
STAMP="${STAMP#v}"
[[ -n "$STAMP" ]] || STAMP="${VERSION}.${GIT_SHA}${GIT_DIRTY:+.dirty}"
"$PY" - "$BDIR/sd/Cores/$CORE_NAME/core.json" "$STAMP" "$(date -u +%Y-%m-%d)" <<'PY'
import json, sys
path, version, date = sys.argv[1:]
assert len(version) <= 31, f"version too long for APF: {version}"
j = json.load(open(path))
j["core"]["metadata"]["version"] = version
j["core"]["metadata"]["date_release"] = date
json.dump(j, open(path, "w"), indent=2)
open(path, "a").write("\n")
print(f"stamped core.json: version={version} date_release={date}")
PY

ZIP="$BDIR/${CORE_NAME}_${STAMP}.zip"
rm -f "$ZIP"
(cd "$BDIR/sd" && zip -qr "$ZIP" .)
"$PY" "$PROVENANCE" package "$REPO" "$BDIR" "$ZIP"

echo
echo "== done"
echo "   bitstream: $BDIR/$RBF_NAME"
echo "   sd tree:   $BDIR/sd/"
echo "   zip:       $ZIP"
echo
