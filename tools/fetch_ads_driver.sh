#!/usr/bin/env bash
# Fetch b»robotized's Beckhoff ADS driver at a pinned commit, apply patches/beckhoff_ads_driver/
# as commits on a local branch, and fetch ads_vendor. Needed for plc:=ads only. Safe to run again.
#
#   tools/fetch_ads_driver.sh            # clones next to this repository, in the workspace's src/
#   tools/fetch_ads_driver.sh DIR        # clones into DIR
#
# Then build the workspace with colcon as usual. To move to another driver commit, delete the
# local branch (git -C .../beckhoff_ads_driver branch -D plc_seam_fixes) and change the pin.
set -euo pipefail
repo="$(cd "$(dirname "$0")/.." && pwd)"
dest="${1:-$(dirname "$repo")}"

DRIVER_URL=https://github.com/b-robotized/beckhoff_ads_driver.git
DRIVER_COMMIT=5aa1f59e6b7428f7ef868d67a6b8ab06f666f3a4    # rolling, 23 February 2026
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
if git -C "$driver" rev-parse --quiet --verify "refs/heads/$BRANCH" >/dev/null; then
    git -C "$driver" checkout --quiet "$BRANCH"
    echo "driver fixes already applied (branch $BRANCH)"
else
    git -C "$driver" checkout --quiet -b "$BRANCH" "$DRIVER_COMMIT"
    git -C "$driver" -c user.name=plc_seam -c user.email=plc_seam@localhost \
        am --quiet "$repo"/patches/beckhoff_ads_driver/*.patch
    echo "driver at ${DRIVER_COMMIT:0:7} with $(ls "$repo"/patches/beckhoff_ads_driver/*.patch | wc -l) fixes (branch $BRANCH)"
fi

vendor="$dest/ads_vendor"
clone_at "$vendor" "$VENDOR_URL" "$VENDOR_COMMIT"
git -C "$vendor" checkout --quiet --detach "$VENDOR_COMMIT"
echo "ads_vendor at ${VENDOR_COMMIT:0:7}. Driver and ads_vendor are in $dest: build with colcon"
