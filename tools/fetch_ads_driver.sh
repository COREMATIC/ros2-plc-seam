#!/usr/bin/env bash
# Fetch b»robotized's Beckhoff ADS driver at a pinned commit, apply patches/beckhoff_ads_driver/
# as commits on a local branch, and fetch ads_vendor. Needed for plc:=ads only. Safe to run again.
#
#   tools/fetch_ads_driver.sh            # clones next to this repository, in the workspace's src/
#   tools/fetch_ads_driver.sh DIR        # clones into DIR
#
# Then build the workspace with colcon as usual. Each run rebuilds the local branch from the pin
# and the patches, so a changed pin or patch takes effect on the next run.
set -euo pipefail
repo="$(cd "$(dirname "$0")/.." && pwd)"
dest="${1:-$(dirname "$repo")}"

DRIVER_URL=https://github.com/b-robotized/beckhoff_ads_driver.git
DRIVER_COMMIT=5aa1f59e6b7428f7ef868d67a6b8ab06f666f3a4    # head of rolling on 22 November 2026
# The driver's jazzy repos file asks for an ads_vendor "jazzy" branch, which does not exist.
VENDOR_URL=https://github.com/b-robotized/ads_vendor.git
VENDOR_COMMIT=e18e45cf281bca74cdafeddbddd5248107597586    # rolling, tag 1.0.2
BRANCH=plc_seam_fixes

clone_at() {   # dir url commit
    [ -d "$1/.git" ] || git clone --quiet "$2" "$1"
    git -C "$1" cat-file -e "$3^{commit}" 2>/dev/null || git -C "$1" fetch --quiet origin
}

driver="$dest/beckhoff_ads_driver"
clone_at "$driver" "$DRIVER_URL" "$DRIVER_COMMIT"
# A patch that failed on an earlier run leaves `git am` in progress: end it, or this run fails too.
git -C "$driver" am --abort 2>/dev/null || true
git -C "$driver" checkout --quiet -B "$BRANCH" "$DRIVER_COMMIT"
git -C "$driver" -c user.name=plc_seam -c user.email=plc_seam@localhost \
    am --quiet "$repo"/patches/beckhoff_ads_driver/*.patch
echo "driver at ${DRIVER_COMMIT:0:7} with $(ls "$repo"/patches/beckhoff_ads_driver/*.patch | wc -l) fixes (branch $BRANCH)"

vendor="$dest/ads_vendor"
clone_at "$vendor" "$VENDOR_URL" "$VENDOR_COMMIT"
git -C "$vendor" checkout --quiet --detach "$VENDOR_COMMIT"
echo "ads_vendor at ${VENDOR_COMMIT:0:7}. Driver and ads_vendor are in $dest: build with colcon"
