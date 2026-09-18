# Setup
TODO:


# Calibrating Stereo Cameras

Two-step process: print a checkerboard, then show it to both cameras while
`calibrate_stereo.py` automatically captures poses and calibrates. You don't
need to read either script to do this — this covers everything you need.

## 1. Print a checkerboard

```bash
python generate_checkerboard.py
```

This writes a PNG to `calibration/` sized for US Letter paper (e.g.
`calibration/checkerboard_9x6_23.9mm.png`) and prints the exact square size
to the console. **Print it at Actual Size / 100%** — never "Fit to Page" —
or the square size on paper won't match what the console printed, which
throws off the whole calibration's sense of scale. Tape or glue it to
something stiff and flat (a paper-only board will curl and hurt accuracy).

If you use a different board size or margin (`--board-cols`, `--board-rows`,
`--margin-in`), the printed square size will differ — that's fine, just use
the value it prints in the next step.

## 2. Run the calibration

```bash
python calibrate_stereo.py --cam-a 0 --cam-b 1
```

`--cam-a`/`--cam-b` are the two cameras' device indices (0, 1, 2, ...) — if
you're not sure which is which, try swapping them if the preview looks
wrong. If your printed board didn't use the default size, add
`--square-size <meters>` using the value `generate_checkerboard.py` printed.

A window opens showing both camera feeds side by side. **Hold the
checkerboard so both cameras can see it, then hold it still** — it captures
automatically, you don't press anything. Move it to a new position, depth,
or tilt between captures; a poorly-varied set of poses makes for a bad
calibration no matter how many captures you take.

What's on screen:
- **Top-left box**: how many captures you have, and the live reprojection
  error (in pixels) once there are enough captures to compute one. Lower is
  better — well under 1px is good, a few px means something's off (see
  Troubleshooting).
- **Top-middle banner**: what's happening right now — waiting for the board,
  holding still, just captured, or "too similar to a previous capture, move
  further" (the diversity check rejecting a near-duplicate pose).
- **Both boxes are red** until you've reached the minimum capture count,
  **green** once you have — that's your signal it's safe to finish.
- Faint outlines mark where you've already captured, so you can see which
  parts of the frame still need coverage.

Press **q** or **Enter** once you're in the green to finish and write
`calibration/stereo_calib.json`. **Esc** aborts without saving anything.

## Tips for a good calibration

- Cover the whole frame, not just the center — corners and edges especially,
  since that's where lens distortion is worst and least constrained without
  data there.
- Vary tilt and distance, not just position — a board that's always dead-on
  and centered doesn't tell the calibration much.
- Keep the room well-lit and avoid moving the board while it's mid-capture
  (that's what the "hold still" gate is for — don't fight it).
- If the live error looks high and isn't improving as you add captures,
  it's worth restarting rather than pushing through — a bad early capture
  (blur, a bent board) can drag the whole result down.

## Troubleshooting

- **"Could not open both cameras"** — try different `--cam-a`/`--cam-b`
  values; camera indices aren't always 0 and 1, and can shift after a
  reboot or replugging a USB camera.
- **Reprojection error stuck high (a few px or more)** — almost always
  either an insufficiently flat/rigid board, too little pose variety, or a
  board printed at the wrong scale (double check you printed at Actual
  Size). See `generate_checkerboard.py`'s printed instructions if unsure.
- **It won't auto-capture even though the board looks steady** — lighting
  or focus issues can make corner detection jittery frame-to-frame even
  when the board itself isn't moving; try brighter, more even lighting.

## Advanced options

Both scripts have further flags for tuning (checkerboard size/margin/DPI,
capture count targets, stillness/diversity thresholds) — run either with
`--help` to see them. The defaults are reasonable starting points and
shouldn't need changing for a typical desktop setup.
