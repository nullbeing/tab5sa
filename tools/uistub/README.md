# uistub — host preview for `main/ui.c`

Renders the real `ui.c` on your Mac so layout can be iterated without a
flash-and-look cycle on the device.

```
./run.sh
```

It compiles `../../main/ui.c` from source every run, so the preview can never
drift from the firmware. Nothing here is part of the ESP-IDF build.

## Output

A 720×1280 PNG (the physical panel size) in `out/`. The PPM it is converted from
is kept alongside as the raw artefact.

The stub PPA really performs the scale and rotate, so the image is what the glass
shows — including the 180° IMU flip. A bug in the rotation or in the touch
mapping shows up here instead of only on hardware.

## Options

| Env | Meaning | Example |
|---|---|---|
| `OUT` | output path | `OUT=out/x.png` |
| `FREQ` | centre frequency, Hz | `FREQ=121500000` |
| `MODE` | 0 = WFM, 1 = NFM, 2 = AM | `MODE=2` |
| `VOL` | volume step, 0..8 | `VOL=3` |
| `FLIP` | render the flipped (270°) orientation | `FLIP=1` |
| `FRAMES` | frames to render before capturing | `FRAMES=400` |
| `TAPS` | scripted taps, see below | `TAPS="160,150,30"` |

### Scripted taps

`TAPS` is a `;`-separated list of `at,lx,ly[,hold]`:

- `at` — poll index at which the finger lands
- `lx, ly` — **logical landscape** coordinates (the same space the layout
  constants in `ui.c` use, 640×360), not panel pixels
- `hold` — polls held down (default 4, minimum 3)

```
TAPS="160,150,30"                  # tap the frequency readout -> keypad opens
TAPS="160,150,30;175,208,206"      # ...then press the "7" key
```

Two gotchas, both learned the hard way:

1. **`at` must be ≥ 150.** `handle_touch()` ignores the first 150 polls as a
   power-on grace period (the real touch controller emits stray points then).
   A tap scheduled earlier is silently swallowed.
2. **Taps need a gap between them.** A press is only recognised on the
   transition to pressed, so consecutive taps must not overlap; leave at least
   `hold + 1` polls between them.

## What is stubbed

`stubs/` shadows the ESP-IDF headers `ui.c` includes. Everything ui.c calls into
is implemented in `stub_main.c`:

- `dsp_*` — a synthetic spectrum on the same **nominal dBm** scale the firmware's
  dsp.c publishes (raw FFT dB plus its offset), because ui.c plots those values
  directly and applies no offset of its own
- `sdr_*` — a fake R820T that is always present
- `bsp_tab5_*` — display, touch, battery and IMU
- `audio_set_volume` — no-op

If `ui.c` starts calling a new BSP function, the preview build breaks here. That
is intentional: it keeps the stub honest instead of letting it drift.

## Layout facts worth knowing

The panel is portrait 720×1280; `ui.c` draws a 640×360 landscape buffer that the
PPA scales ×2 and rotates 90°. So the preview looks "sideways" — that is correct,
it is the physical orientation.

Touch coordinates map as:

```
upright:  lx = (PANEL_H - p.y)/2,  ly = p.x/2
flipped:  lx = p.y/2,              ly = (PANEL_W - p.x)/2
```

`stub_main.c` uses the inverse of these, which is how a tap at logical `(lx,ly)`
ends up hitting the intended control.
