#!/bin/bash
# Build a complete, buildable SDK tree with the T-Deck port applied.
# Usage: ./apply.sh [destination-dir]   (default: ./muse-gadget-sdk)
set -euo pipefail

UPSTREAM_COMMIT="fa073045a5572a8e3367fae142c1b7a7c554f222"
DEST="${1:-muse-gadget-sdk}"
HERE="$(cd "$(dirname "$0")" && pwd)"

if [ -d "$DEST" ]; then
    echo "Destination $DEST already exists; remove it or pass another dir." >&2
    exit 1
fi

git clone https://github.com/facebookincubator/muse-gadget-sdk.git "$DEST"
cd "$DEST"
git checkout "$UPSTREAM_COMMIT"

# Copy the port files over the upstream tree.
cp "$HERE/esp32/devices/sdkconfig.muse-lilygo-t-deck" esp32/devices/
cp "$HERE/esp32/components/muse/boards/board_lilygo_t_deck.c" esp32/components/muse/boards/
cp "$HERE/esp32/components/muse/Kconfig" \
   "$HERE/esp32/components/muse/CMakeLists.txt" \
   "$HERE/esp32/components/muse/idf_component.yml" esp32/components/muse/
cp "$HERE/esp32/tools/muse/board.sh" \
   "$HERE/esp32/tools/muse/ports.py" \
   "$HERE/esp32/tools/muse/avatar.py" esp32/tools/muse/
# (Upstream doc tables esp32/AGENTS.md and esp32/devices/README.md gain a
#  T-Deck row each; see tdeck-muse-gadget-guide.md. Cosmetic only.)

echo
echo "Done. Next:"
echo "  cd $DEST/esp32"
echo "  . ~/esp/esp-idf-v6.0.1/export.sh   # ESP-IDF v6.0.1"
echo "  tools/muse/board.sh build tdeck"
