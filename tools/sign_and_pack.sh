#!/usr/bin/env bash
# sign-and-pack.sh — copy (no hidden files), recursively sign with evmctl, and tar with IMA xattrs

set -euo pipefail

usage() {
  cat >&2 <<'EOF'
Usage:
  sudo ./sign-and-pack.sh <key_path.pem> <source_dir>

Does:
  1) Creates a sibling tmp dir next to <source_dir> (same parent).
  2) Copies <source_dir> into the tmp dir, excluding all hidden files/dirs (e.g. .git, .env, etc).
  3) Recursively signs the tmp dir with: evmctl -r sign --imasig --key <key>.
  4) Produces a tarball that includes IMA/EVM xattrs.

Outputs:
  - <basename>-signed-YYYYmmddHHMMSS.tar in the same parent as <source_dir>.

Notes:
  - Must run as root (will re-exec via sudo if needed).
  - Requires: evmctl, rsync, tar (GNU tar with xattrs support).
  - To keep the temporary directory for inspection, set KEEP_TMP=1 in the environment.
    Example: KEEP_TMP=1 sudo ./sign-and-pack.sh key.pem /path/to/src
  - To restore xattrs on extract, extract as root with:
      sudo tar --xattrs --xattrs-include='*' -xvf your.tar -C /dest
EOF
}

if [[ "${1:-}" =~ ^(-h|--help)$ || $# -ne 2 ]]; then
  usage
  exit $([[ $# -eq 2 ]] && echo 0 || echo 1)
fi

KEY="$1"
SRC="$2"

# Re-exec as root if needed
if [[ $EUID -ne 0 ]]; then
  exec sudo --preserve-env=PATH,KEEP_TMP "$0" "$@"
fi

# Tool checks
for bin in evmctl rsync tar realpath mktemp; do
  command -v "$bin" >/dev/null 2>&1 || { echo "ERROR: '$bin' not found in PATH." >&2; exit 1; }
done

# Basic checks
[[ -f "$KEY" ]] || { echo "ERROR: Key not found: $KEY" >&2; exit 1; }
[[ -d "$SRC" ]] || { echo "ERROR: Source directory not found: $SRC" >&2; exit 1; }

ABS_SRC=$(realpath -m "$SRC")
PARENT=$(dirname "$ABS_SRC")
BASE=$(basename "$ABS_SRC")

# Create sibling tmp dir
TMPDIR=$(mktemp -d -p "$PARENT" "${BASE}.signed.XXXXXXXX")
echo "Temp directory: $TMPDIR"

# Copy non-hidden content (skip all .* at any depth)
# -a preserves perms/times/links; -A/-X preserve ACLs and xattrs if present
# --exclude='**/.*' drops all hidden files/dirs like .git, .env, .cache, etc.
# The trailing slashes ensure we copy CONTENTS of SRC into TMPDIR/<BASE> (nice packaging shape).
mkdir -p "$TMPDIR/$BASE"
rsync -aAX --prune-empty-dirs --exclude='**/.*' "$ABS_SRC"/ "$TMPDIR/$BASE"/

# Recursively sign the tmp copy
echo "Signing files under: $TMPDIR/$BASE"
evmctl -r sign --imasig --key "$KEY" "$TMPDIR/$BASE"

# Build tar with xattrs (so security.ima is included)
ts=$(date +%Y%m%d%H%M%S)
OUT_TAR="$PARENT/${BASE}-signed-${ts}.tar"

echo "Creating tarball with xattrs: $OUT_TAR"
# Use -C to store paths relative to the parent of the tmp dir; pack only the copied folder ($BASE)
tar --xattrs --xattrs-include='*' -cvf "$OUT_TAR" -C "$TMPDIR" "$BASE"

echo "Done."
echo "Tarball: $OUT_TAR"
if [[ "${KEEP_TMP:-0}" == "1" ]]; then
  echo "Keeping temp directory (KEEP_TMP=1): $TMPDIR"
else
  rm -rf "$TMPDIR"
  echo "Temp directory removed."
fi
