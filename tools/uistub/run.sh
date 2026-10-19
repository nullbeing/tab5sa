#!/bin/sh
#
# Build and run the host preview of main/ui.c.
#
#   ./run.sh                                  # default view
#   OUT=out/flip.png FLIP=1 ./run.sh          # 180° flipped (IMU)
#   FREQ=121500000 MODE=2 ./run.sh            # 121.5 MHz, AM
#   TAPS="20,150,30" ./run.sh                 # tap the frequency readout
#   TAPS="20,150,30;40,320,300" ./run.sh      # then press keypad "7"
#
# Everything is compiled fresh from ../../main/ui.c, so the preview can never
# drift from the firmware.
set -eu

here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
out=${OUT:-$here/out/preview.png}
mkdir -p "$(dirname "$out")" "$here/.build"

# The binary always emits a raw PPM; PNG conversion is a convenience step.
ppm=${out%.png}.ppm
bin=$here/.build/uistub

cc=${CC:-cc}
$cc -std=gnu11 -O1 -g -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare \
    -I"$here/stubs" -I"$repo/main" \
    "$repo/main/ui.c" "$here/stub_main.c" \
    -o "$bin" -lm

OUT="$ppm" "$bin" "$@"

if command -v magick >/dev/null 2>&1; then
    magick "$ppm" "$out"
elif command -v sips >/dev/null 2>&1; then
    sips -s format png "$ppm" --out "$out" >/dev/null 2>&1
fi
echo "wrote $out"
