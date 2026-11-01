# Phase-2 event-level + e2e RTL tests (research, rev 2)

**Scope:** `isp_circle_hough2.v` + `isp_grad_event96.v`.
Event-level vectors are synthetic 4×4-block events; the e2e TB drives a
synthetic 1280×720 grayscale frame through the real front end.  No
synthesis/P&R result is claimed by this file (see §7.6 of the research doc:
the core does **not** fit the phase-1 budget).

## Build / run (ModelSim 10.3c)

```bat
cd lab_ex4_ext_SC520\sim
vlib work_core
vlog -work work_core ..\user_source\hdl_source\isp\isp_grad_event96.v ^
     ..\user_source\hdl_source\isp\isp_circle_hough2.v ^
     tb\tb_circle_hough2.v tb\tb_frontend_e2e.v

rem event level
vsim -c -L work_core work_core.tb_circle_hough2 +EVFILE=model\ev_r60.hex ^
     +ECX=640 +ECY=360 +ER=60 +EVALID=1 -do "run -all; quit -f"

rem pixel level (one frame; ~15-30 s)
vsim -c -L work_core work_core.tb_frontend_e2e +CX=640 +CY=360 +R=60 +EVALID=1
```

`tb_frontend_e2e` scene plusargs: `SCENE` (0 disk, 1 rect, 2 ellipse,
3 two disks, 4 flat, 5 disk+rect), `RA/RB`, `HW/HH`, `CX2/CY2/R2`,
`GAPW` (occluding slab on the right), `NOISE` (uniform ±).

## rev-2 architecture (what the tests exercise)

- Front end: 4-row block, 8 signed Sobel samples summed for direction,
  block energy gate 512, double-banked 1R/1W block RAMs, one frame_end.
- Core: 3-direction fan coarse vote (0/±12°) → median-radius init →
  12 half-step radial Gauss-Newton refinements (positions only) →
  1px radial histogram + **position-angle** sector map → gates
  (cov≥10, quadrants, peak≥12, fill≥0.5, aligned≥1/8, peak_r≥26,
  longest empty sector run ≤5).

## Recorded results (2026-11-01, rev 2)

### Event level (ideal positions/directions)

| Case | Result |
|---|---|
| r25 ×4 phases | all reject |
| r26 ×4 phases | 2/4 pass (r=26, center ≤2 px); o1/o3 miss |
| r60 / r100 / r160 | pass, r=59/99/159 |
| circle+ellipse, circle+rect | selects circle |
| empty / checkerboard | reject |
| dark-on-light r60 | pass |
| 180° arc (ideal, dense) | **passes** (known limitation; pixel arc rejects) |
| two circles r40/r100 | selects the **larger** r99 (documented) |

Log: `logs/tb_circle_hough2_matrix.log`.

### Pixel-level e2e (1280×720, one frame per run)

| Scene | Result |
|---|---|
| r25 (4 phases) | 4/4 reject |
| r26 (4 phases) | 2/4 pass (r=26/27) |
| r60 / r100 / r160 (4 phases each) | 12/12 pass (r±1, center ±1–2) |
| 400×400 square, no circle | reject (401 events) |
| ellipse 80×50 | reject |
| half circle (GAPW=70) | reject (fill 0.37) |
| 3/4 circle (GAPW=30) | pass r=59 |
| r60 + uniform noise ±8 | pass r=60 (642 events, 536 noise) |
| noise only ±10 | reject (overflow 1024) |
| r60 + square | selects the circle |
| two circles r40/r100 | selects the larger r100 |

Logs: `logs/e2e_matrix_phase2.log`, `logs/e2e_negatives_phase2.log`.

### Front-end direction quality (e2e, after the block-word packing fix)

| r | events | >14° | normal-line >16px |
|---|---|---|---|
| 60 | 102 | 2 | 1 |
| 100 | 170 | 6 | 27 |
| 160 | 276 | 7 | 83 |

The fixed packing removed the gross direction corruption that rev 1 saw.

## Known limitations

- r=26 is a 1px boundary: the strict ρ≥26 window is required to reject r25
  (a ρ≥24 window falsely detects r25 in 2/4 phases, measured).
- Ideal (exactly-on-circle, dense) 180° arcs still pass the event-level gate.
- Two complete circles: the larger one wins on score.
- Uniform noise ±10 overflows EVMAX and rejects the frame (intentional).

## rev 2b/2c (after RAM/ERAM and timing fixes)

- Block RAMs moved into `isp_dpram` instances with address widths equal to the
  depth (acc 16 bits, ev_ram 10, hmem 8). TD falls back to distributed LUT RAM
  when the port address is wider than the depth needs; that alone had cost
  8,448 LUTs.
- Coverage/gate logic pipelined (dot magnitude during sqrt, gate decision
  registered, longest-gap scan over 16 cycles), fan direction registered,
  emission staged (read/normalise/reciprocal/multiply).
- Core gate: 29 ERAM, 1 DSP, 2,357 LUT6, 986 slices, WNS **+0.866 ns**.
- Front end gate: 5 ERAM, 2 DSP, 1,272 LUT, 1,115 slices, WNS **+1.992 ns**.
- Throughput (103.5 MHz): r160 276 events = 681,252 cycles (6.6 ms);
  near-cap 1002 events = 978,732 cycles (9.5 ms); frame budget 1.725 M cycles.
  Single event bank, so a frame arriving during processing is dropped
  (2-frame update cadence), as documented.

### Pixel-level e2e matrix (final rev 2c)

| Scene | Result |
|---|---|
| r25 ×4 phases | 4/4 reject |
| r26 ×4 phases | 3/4 pass (o1 misses) |
| r60 ×4 | 4/4 pass (r=59/60, centre ±1) |
| r100 ×4 | 4/4 pass (r=99/100) |
| r160 ×4 | 4/4 pass (r=159, centre ±1) |
| square no circle | reject (401 ev) |
| ellipse 80×50 | reject |
| 180° arc | reject |
| 300° arc | pass r=59 |
| r60 + noise ±8 | pass r=59 (198 ev) |
| noise only ±10 | reject (1002 ev, gates) |
| circle + square | picks circle |
| two circles r40/r100 | picks larger r100 (documented) |

Log: `logs/e2e_matrix_rev2c.log`.

## rev 2f — delivery-prep fixes (final)

- Coarse vote: single direction (fan removed), 4 px x 40 steps, 12-bit
  accumulator (no saturation). A 5-bit `k` counter silently truncated 40
  steps to 8; fixed to 6 bits (this was the hidden r160/competition failure).
- r26: `quad` gate removed; 9-point (±2 px) local centre search for r0<=40;
  final-pass result register race fixed. e2e r26 is now 4/4.
- Two circles: `SCORE_MARGIN=0` (strict most-circular). The r40 circle now
  scores 4351 vs 4348 for r100 after the local search, so it wins on score.
  Small/rounder/big ellipses vs a true circle all select the circle.
- Final full build (EVT_CIRCLE=1, `camera_to_dsi_display_evt.sdc`):
  syn WNS +0.257 ns, P&R WNS +0.112 ns, hold +0.020 ns, 0 violations,
  slices 17,715/21,216, ERAM 85/108, DSP 27/40.
  Bit SHA256 `78bced1a...846189` (supersedes `96974d37...`).
- EVT_CIRCLE=0 build with the original SDC: ccl_clk generated and constrained,
  40 ERAM / 31 DSP / 20,126 LUT, WNS +0.320 ns.
- Unconstrained check: same 37/68 IO-delay items and one I2C no-clock net as
  phase 1 (no new items); `report_timing -unconstrained` unsupported in TD 6.2.

### Final pixel-level matrix (rev 2f)

r25 x4 reject; r26 x4 pass; r60/r100/r160 x4 pass; square/ellipse/180-deg arc
reject; 300-deg arc, circle+square, +-8 noise pass; noise-only rejected;
two circles -> smaller r40; small/rounder/big ellipse vs circle -> circle.
Log: `logs/e2e_matrix_rev2f.log`. Event level 17/18 (ideal dense 180-deg arc
still passes; the pixel-level arc rejects).

## rev 2g — board-feedback stability fixes

### Frame handoff defect (reproduced, then fixed)

With one event bank, every other captured frame starts while the previous
frame is being processed.  The old capture path latched `cap_drop=1` but still
started a "processing" for that dropped frame if `proc_done` arrived before
its `frame_end`; `S_DONE` then wrote an invalid result, clearing the box for
about two display frames -> visible box flicker.

Fix:
- capture never starts a frame that was dropped (`frame_end && !proc_busy &&
  !cap_drop`);
- `S_IDLE` clears the previous frame's `best_*` before the empty/overflow
  short-circuit (an empty frame used to reuse stale `best_ok=1` and refresh
  the hold forever);
- bounded hold: a processed frame without a valid circle keeps the previous
  box for `HOLD=2` processed frames, then clears.  Skipped frames do not
  reach `S_DONE`, so they neither age the hold nor write an invalid result.

`tb_handoff2.v` reproduces the defect on 5afe931 (dropped frame emits an
invalid O_evt; no hold) and passes after the fix: busy-time frame start,
processing finishing inside the frame, dropped frame end, recovery, empty
frames (hold 2 -> 1 -> 0 -> clear), overflow frame aging the hold, reset.

`tb_multiframe_e2e.v` drives five real-timing 1280x720 frames (f1 circle,
f2 arrives while f1 is processing, f3..f5 background).  It checks that f2
produces no O_evt, that O_valid never drops before the clear (glitch samples
= 0), that f3/f4 hold the r60 box, and that f5 clears it.  PASS.

### Colour / low-contrast investigation (no threshold change)

Exact shared luma is `Y = (R>>2)+(G>>1)+(B>>2)` (`isp_rgb2y96`).

| RGB scene (r60) | Y contrast | RTL result |
|---|---|---|
| red (200,40,40) on 140 grey | 60 | detected r=60 |
| blue (40,40,200) on 140 grey | 60 | detected |
| green (40,200,40) on 140 grey | 20 | missed (26 events) |
| green on dark 60 grey | 60 | detected |
| dim (80,100,80) on 120 grey | 30 | missed (56 events) |

Lowering the block energy gate was measured and rejected: EV_TH=384 floods
sigma=8 noise (1721 events -> EVMAX overflow, circle lost) and EV_TH=320
floods worse (4853).  A block direction-coherence gate was prototyped in RTL
(`|sum g| >= 0.5 * sum|g|`); it did not reduce the sigma=8 flood (1717
events) and green20 still missed, so it was reverted.  `EDGE_TH=48` and
`EV_TH=512` are unchanged.  The likely future fix is a 5x5/Scharr prefilter
with real-frame validation; this is recorded, not claimed.

### Display snapshot / CDC

The wrapper stages a whole result group at the ISP frame event and transfers
it with a toggle/ack handshake.  The event detector updates `O_det_*` only at
`S_DONE`, several ms after `frame_end`, while the staging samples at
`frame_end` (+4 cycles) and the next capture is one frame later, so the group
is always sampled while stable; `O_evt` is not part of the CDC path.

### Bounded-timeout semantics (overflow / skipped frames)

- **Processed frame with no valid circle** (empty, or `cap_ovf` due to the
  EVMAX event cap): the core walks the short-circuit `S_IDLE -> S_DONE`, which
  ages the hold by one.  After `HOLD=2` such processed frames the box clears
  (`HOLD+1` processed frames total; with the 2-frame capture cadence that is
  about 5-6 display frames, ~80-100 ms).
- **Skipped frame** (`cap_drop`: frame started while the previous one was
  still being processed): not processed at all - no `O_evt`, no result write,
  and the hold is not aged.  The previous box stays.
- **Sustained skipping cannot persist** in this timing: measured worst-case
  processing is 897k cycles (8.7 ms at 103.5 MHz) for 1002 events
  (EVMAX=1024), while a video frame is 1.725M cycles (16.7 ms).  A skipped
  frame is therefore always followed by a captured one, so the hold is aged
  or refreshed at least every other frame.  If the clock/video timing is ever
  changed such that processing can span a full frame, this argument (and the
  need for a hard skip watchdog) must be re-checked.

## rev 2h — green miss diagnosis + safe changes

- Chain measurement: red/blue (Y contrast 60) detect; green on 140 grey
  (contrast 20) and dim (30) miss.  For green the events are on-contour with
  good directions and cov=16/maxrun=0, but `peak_v=29` vs expected 94 ->
  fill=0.31, so the only failing gate is fill>=0.5.  A half arc has fill 0.37,
  so lowering fill would admit the arc: the fix must be more events (front-end
  SNR), not a threshold.
- Rejected with evidence: EV_TH 384/320 (sigma=8 noise 1721/4853 -> overflow),
  a direction-coherence gate (noise unchanged), and a 3-row 1-2-1 prefilter
  (after fixing an inverted gy sign it still left green at fill 0.31 and the
  RTL sigma=8 count at 1721) — all reverted.
- Shipped: EVMAX 1024 -> 1536 (overflow headroom; ~1.37M cycles estimated at
  1536 events < 1.725M frame), HUD flags[7] now exposes proc_ovf/drop.
  EDGE_TH=48 / EV_TH=512 / POP / AWB / AE unchanged.
- tb_handoff2 (1600-event overflow), tb_multiframe_e2e, event and single-frame
  matrices, original regressions: PASS.
- Build: syn WNS +0.246 ns; P&R WNS +0.164 ns, hold +0.020 ns, 0 violations,
  ERAM 86/108, DSP 27/40, slices 17,803/21,216.  New independent bit
  `camera_to_dsi_display_phase2_diag.bit` SHA256 `c1570200...e522a` (not flashed).

## rev 2i — HUD diagnostics: drop vs overflow, result-snapshot count

### Defects fixed

- `O_det_flags[7]` was `drop_seen | proc_drop | proc_ovf`, so the board could
  not tell a skipped frame from a real EVMAX overflow.  `proc_drop` was dead
  (a skipped frame never reaches `S_DONE`, so it can never produce a result).
- `O_det_cnt` was `S_ev_cnt[9:0]`: a 1536-event frame wrapped to 512, 1000..1023
  rendered as the wrong three-digit value, and `O_count_dbg` was updated at
  `frame_end`, i.e. before the result was ready.  The HUD could therefore show
  a newer frame's count next to an older frame's flags/box.
- `drop_seen` was set only when `proc_busy` was still high at `frame_end`, so a
  skipped frame whose processing happened to finish inside it was not reported.

### RTL change

- New `isp_evt_hud_map.v` (used by `isp_top` only when `EVT_CIRCLE=1`):
  `[7]=proc_ovf`, `[6]=live`, `[5]=drop_seen`, `[4:2]=0`, `[1]=pass`,
  `[0]=valid`; `O_det_cnt = min(I_cnt, 999)` (saturate, not truncate).
  999 means ">=999".  valid/pass semantics are unchanged.
- `isp_circle_hough2`: `O_count_dbg` is latched in `S_DONE` with
  `O_valid/O_flags/bbox` (exact 0..EVMAX); `drop_seen` is now set by the
  per-frame `cap_drop` marker at `frame_end`; dead `proc_drop` removed.

### Bounded tests (`tb_diag_hud.v`)

Instantiates the real detector + real HUD mapping.  All checks PASS.

| Case | Result |
|---|---|
| full load 1536, real 1.725M-cycle frame boundary | count=1536, HUD cnt=999, ovf=0, live=1, pass/valid=1 |
| over limit 1537 | captured count=1536, ovf=1, drop=0, short-circuit, box held |
| skipped frame (started while busy) | no `O_evt`; next result keeps its own count and sets drop=1 |
| drop clears on next processed result | drop=0, own count |
| bounded hold | 2 -> 1 -> 0 -> clear; overflow ages the hold; skip does not |
| reset | count/flags/valid cleared |

**Throughput correction:** measured full-load processing is **2,596,848 cycles**
from frame start (1.50 frame periods), not the ~1.37M figure estimated in
rev 2h.  Per-event cost measured from r60/r100/r160 is ~1,526 cycles/event plus
~246k fixed.  Therefore a 1536-event frame does NOT finish inside one frame:
the next real frame is skipped, and the HUD now reports `drop=1, ovf=0`
instead of hiding it.  This is a diagnostic result, not a claim that low
contrast is fixed.

### Regression

- `tb_handoff2`: PASS (updated to print `drop_seen`; `proc_drop` removed).
- `tb_multiframe_e2e`: PASS.  f2 is skipped with no `O_evt`; f3's held result
  is `flags=08` (drop=1), f4 returns to `flags=00`; glitch samples = 0.
- Event-level matrix r25/r26/r60/r100/r160/empty/checker: PASS.  Checker
  (2048 events) reports count=1536 and `flags=02` (ovf), per the new layout.
- Original regressions `tb_pop` / `tb_gray_share` / `tb_det_result` /
  `tb_ae_i2c`: PASS.

### Build (rev 2i)

Fresh tree `D:\FPGA\phase2_build_20261102`, final sources, EVT SDC:
- Synthesis: WNS +0.041 ns, hold +0.114 ns, ERAM 86/108, DSP 27/40.
  (The synthesis critical path moved to the pre-existing
  `u_isp_top/u_chroma_denoise`, not the new HUD logic.)
- P&R: WNS +0.231 ns, TNS 0, 0 endpoints; hold +0.020 ns, 0 violations;
  slices 17,787/21,216 (83.84%), ERAM 86/108, DSP 27/40,
  LUT6 17,058/21,216 (80.40%).
- Unconstrained profile identical to phase 1: 1 I2C no-clock-root net,
  37 input / 68 output ports without IO delay, 0 invalid constraints.
- New bit `camera_to_dsi_display_phase2_hud.bit`, SHA256
  `18a747cf3f2c85ddbba2408940fdf6a961d7a5b6d66bda21b0825ceedb0c5030`
  (not flashed; older bits untouched).

## rev 2j — CIRCLE text overlay + radius >32

**Scope / honesty:** the only detector-core change is `R_MIN 26 -> 33`
(reject radius 26..32 small targets).  There is **no verified low-risk fix
for the low-contrast miss** and this version does **not** claim recognition
stability is solved.  The rest is a display/diagnostic overlay plus bounded
tests.  HOLD is unchanged (2).

### Text overlay (`tb/tb_circle_text.v`)

Instantiates the real `hdmi_mixer` (1280x720, DEBUG_MODE=0) and captures one
output frame per case.  All checks PASS:

| Case | Result |
|---|---|
| `X+123 Y-045` (centre box) | glyph sequence exact; green 381 px (green bbox 8 px tall, y0+1..y0+8), black 639 px; black bbox starts at `(x0+3, y0)`; 100% of green pixels have a black neighbour within ±2 px |
| `X+000 Y+000` | zero shows `+000`; rendered |
| `X-045 Y+123` | negative sign/digits rendered |
| four screen corners | x clamps to 0 / 1096, y places below/above; text bbox stays inside 0..1279 x 0..719 |
| `I_det_valid=0` | text enable cleared; zero green/black text pixels |
| multi-frame A→B→invalid | B's glyphs replace A's; A's old region is empty; invalid frame removes B |

**Bit-order root cause and visual evidence.** The display selects
`glyph_row[gx]` with `gx` increasing right, so the font must be LSB-left
(bit0 = leftmost).  The first 32-px version used MSB-left C/R/L patterns
while E/digits were already LSB-left, so the board showed `CIЯↃ⅃E`.
Only C/R/L were changed to LSB-left; E/digits were left alone.  The test
now writes the actual captured pixels at 1 char/display-pixel to
`logs/text_render.txt` and `logs/text_render.ppm` (converted to
`logs/text_render.png`); per-character inspection confirms
`C I R C L E` and every digit/sign orientation, with the 1-px outline
surrounding (not overwriting) the glyph pixels.

Run:
```bat
vlog -work work ..\user_source\hdl_source\anlogic_logo_rom.v ^
     ..\user_source\hdl_source\hdmi_mixer.v tb\tb_circle_text.v
vsim -c work.tb_circle_text -do "run -all; quit -f"
```
The test is a synthetic display test; it does not prove a board result.

### Radius 32/33 boundary

`isp_circle_hough2.R_MIN` changed 26 -> **33**: the final event window is
`rho >= 33`, i.e. strictly `>32px`; `R_MAX=160`.  New vectors
`ev_r32/r33/r34.hex` are generated by `model/gen_phase2_vectors.py`.

Event level (real core):

| Input radius | Result | Reported r |
|---|---|---|
| 26 | reject | 0 |
| 32 | reject (cov 8 < 10) | 0 |
| 33 | **accept** | 32 |
| 34 | accept | 33 |
| 60/100/160 | accept | 59/100/159 |

Pixel level (`tb_frontend_e2e`, real front end + core; one frame each):
r32 reject, r33 accept with `cx=640 cy=360 r=33`.

**Quantization note:** the final 3-bin peak reports its LOWER bin
(`peak_bin+B0`), so a true radius 33 can be reported as 32 at the event
level; the acceptance window is still `rho >= 33`.  The pixel-level r33
reported 33.  The output bbox radius is therefore 1 px conservative at some
phases; this is a reporting/quantisation effect, not an acceptance of r<=32.

### Regression after the radius change

- `tb_diag_hud` PASS (full load 1536, over-limit 1537, skip/hold/snapshot).
- `tb_handoff2` PASS.
- Event matrix: r25/r26 reject; r60/r100/r160 accept; empty/checker reject.
- `tb_circle_text` PASS (above).
- Original `tb_pop`/`tb_gray_share`/`tb_det_result`/`tb_ae_i2c` unchanged by
  the detector radius change; `tb_det_result`/`tb_pop`/`tb_gray_share` PASS
  in this round (see logs).

### Build

See research section 14 for the independent synthesis/P&R, resource and bit
hash record.

### Build (rev 2j)

Fresh tree `D:\FPGA\phase2_build_20261103`, final sources (diff-verified),
EVT SDC, sequential synthesis then P&R:
- Synthesis: WNS +0.001 ns, hold +0.114 ns, ERAM 86/108, DSP 27/40,
  LUT6 9,754.  Critical path was the pre-existing chroma-denoise logic.
- P&R: WNS +0.139 ns, TNS 0, 0 endpoints; hold +0.020 ns, 0 violations;
  slices 18,130/21,216 (85.45%), LUT6 17,441/21,216 (82.21%),
  ERAM 86/108, DSP 27/40.  P&R critical path is the existing
  `u_evt_circle/u_acc` RAM read, not the text overlay.
- Unconstrained profile unchanged (1 I2C no-clock root, 37/68 IO delays,
  4 dangling, 0 invalid constraints).
- New bit `camera_to_dsi_display_phase2_text.bit`, SHA256
  `b8e9145f4d052b3de6a534f980f592afdb2fd4cbc9c55c0a9bcf0e0c93128607`
  (not flashed; older bits untouched).

## rev 2k — CIRCLE glyph bit-order fix + 8px

### Root cause (pixel-level, not character numbering)

The display selects `glyph_row[gx]` with `gx` increasing to the right, so
the font must be LSB-left (bit0 = leftmost).  The first text font mixed
conventions: C/R/L were MSB-left while E/digits were already LSB-left.
The board showed exactly `CIЯↃ⅃E` (only C/R/L mirrored).  A blanket
bit reversal was tested and rejected: it corrected C/R/L but mirrored the
already-correct E/digits.  Final fix: rewrite only C/R/L to LSB-left;
E/digits keep their proven patterns.

### Size

- `TXT_SCALE 4 -> 1`: 8x8 glyph at 1:1 = **8 px high** (quarter of 32 px);
  10x10 cell with 1-px outline; string 180x10 (was 720x40).
- Placement clamp x 0..1096, y 0..710; above/below avoidance, same-frame
  snapshot, valid-gated disappearance unchanged.

### Pixel evidence

`tb_circle_text.v` dumps the actual captured `hdmi_mixer` output at
1 char/display-pixel to `logs/text_render.txt` and `.ppm` (converted to
`.png`).  Per-character windows aligned to the 10-px cells show normal
`C I R C L E`, digits `1 2 3 0 4 5` and signs `+ -`; the 1-px outline
surrounds the glyphs (green 381 px, black 639 px, 100% of green pixels
have a black neighbour within ±2 px) and does not erase `#` glyph pixels.
All cases PASS: positive/negative/zero, four corners, invalid-box
disappearance, A->B->invalid multi-frame.

### Build

Fresh tree `D:\FPGA\phase2_build_20261104`, final sources (diff-verified),
EVT SDC, sequential syn then P&R:
- Synthesis: WNS +0.204 ns, hold +0.114 ns, ERAM 86/108, DSP 27/40,
  LUT6 9,845; critical path is the existing detector (`u_evt_circle/gate_ok`).
- P&R: WNS +0.129 ns, TNS 0, 0 endpoints; hold +0.020 ns, 0 violations;
  slices 18,218/21,216 (85.87%), LUT6 17,463/21,216 (82.31%),
  ERAM 86/108, DSP 27/40; critical path is the existing
  `u_evt_circle/u_acc` RAM read, not the text.
- Unconstrained profile unchanged (1 I2C no-clock root, 37/68 IO delays,
  4 dangling, 0 invalid).
- New bit `camera_to_dsi_display_phase2_text8.bit`, SHA256
  `f86560f64091fd64470b53f9ea2306795a6c43a10531f344ea6f21464a556ac2`
  (not flashed; all older bits preserved).
- No changes to the circle detector, R_MIN/EVMAX/thresholds, exposure,
  POP/AWB/AE in this round.

## rev 2l — cross-frame ball tracking + GREEN + runtime DD:HH:MM:SS

Scope: detector core, R_MIN/EVMAX/thresholds, exposure, POP/AWB/AE are
unchanged.  New CSI-domain tracker/colour sampler, wrapper green CDC bit,
runtime RTC + OSD.

### Unit tests (all PASS)

- `tb_ball_track.v` (MS_DIV=10 scaled, HOLD_FRAMES=4,
  HOLD_TIMEOUT_MS=200): 10 consecutive detections -> stable; miss -> hold;
  hold expires after **4 real camera-frame pulses** and clears; hold does
  not count as a detection; far detection ignored during hold; global
  recovery after expiry; radius +20% normal / +31% relaxed / +50% hold;
  position relaxed twice then hold; fast reacquire; no-frame stall clears
  via the 200 ms timeout; reset.
- **Stability gate is separate from the motion gate.**  The 10-frame
  counter uses a candidate **anchor** (first detection): position <=16 px
  and radius <=~10%+2.  A detection still inside the 1.5r+16 motion gate
  but outside the anchor window re-anchors and resets the count to 1 while
  tracking continues.  Tests: +5 px per detection for 20 detections
  (each step inside the motion gate, total 100 px) -> `stable=0` and still
  displayed; stop + 10 static -> stable; 5 static + a 20 px jump (outside
  16 px anchor, inside 106 px motion) -> count reset/stable=0 but still
  tracked, then 10 static -> stable.  All PASS.
- **Target-identity colour isolation.**  The tracker forces `out_green=0`
  and emits `out_id_chg` on a new candidate, a motion-gate restart, an
  anchor reset, or a relaxed jump outside the tight colour gate
  (position <=16 px, radius <=~10%+2).  Tests: stable green -> red target
  150 px away (inside the relaxed 182 px motion gate, outside the 16 px
  colour gate, with `green_in` deliberately still 1) -> `out_green=0`
  immediately while still tracked; red samples stay 0; a green target at
  the new position re-confirms GREEN.  During a hold the old green box
  keeps its GREEN (same target); after the 4-frame hold the far new target
  starts with GREEN cleared.  PASS.
- `tb_green_sample.v`: `id_chg` clears GREEN, a mid-sample identity change
  aborts the in-flight sample, and an invalid edge grid clears GREEN
  instead of retaining the old value.  PASS.

**Frame-rate / clock / core-hold verification.**  The SC520 config table
header is `1280x720@60fps, 24MHz MCLK, 2-Lane RAW10`; the HUD `FPS:60`
is the ISP-tuser count, and the SDC/timing report says
`MIPI_RX_BYTE_CLK = 103.520 MHz` (`MS_DIV=103520` is correct).  The photo's
`RAW 30` is the mode row (RAW + calibration status/blank), not the frame
rate.  The hold now counts 4 real camera frames from `O_frame_start`
(67 ms at 60 fps, 133 ms if the sensor were 30 fps), plus a 200 ms hard
timeout, so it is independent of the frame-rate assumption.  The core's
internal `HOLD=2` frames have `O_flags[0]=0`; the tracker uses only
`O_valid && O_flags[0]`, so stale core holds are not counted, do not
refresh `last_frame/last_ms`, and the display valid comes only from the
tracker - no extra clear delay.
- `tb_green_sample.v`: user HUD samples (12,23,17)/(22,45,35)/(15,30,22)
  are GREEN; (10,10,10)/(10,13,10)/(10,20,25)/(1,3,1) are not; 2-sample
  hysteresis; track loss clears GREEN.
- `tb_rtc_cdc.v` (CLK_DIV=20 scaled, asynchronous clocks): 58->59,
  59->00 minute carry, 23:59:59 -> day+1 00:00:00, saturation at
  99:23:59:59, coherent group, reset clears.
- `tb_circle_text.v`: dynamic 18/24-char CIRCLE (GREEN only when
  I_det_green=1), positive/negative/zero, four corners, 8 px glyph,
  green+black outline/alignment, valid=0 removal, A(GREEN)->B(non-green)
  ->invalid multi-frame; runtime 01:02:03:04 colon/position, box-over-
  runtime conflict with coherent fields.  PASS.

### Interface/timing notes

- Colour: `S_beatq_r/g/b` = 4-pixel sum, **post-AWB / pre-gamma** 8-bit
  RGB, same data as the HUD centre ROI; 16-beat (64 px) interior grid.
- Detection: `O_evt` = core S_DONE; `O_flags[0]=pass` only for a fresh
  detection; the core's own hold is not counted.
- `O_det_green` joins the existing S_hdmi_user bbox/cx/cy/valid CDC group.
- Runtime: 50 MHz fixed divider, 1 Hz; `{day,hour,min,sec}` moved as one
  group with a request/ack toggle to the HDMI pixel clock.

### Build (rev 2l)

Fresh tree `D:\FPGA\phase2_build_20261105`, final RTL/SDC snapshot
(`diff -rq` verified), EVT SDC, sequential syn then P&R:
- Syn: WNS +0.212 ns, hold +0.114 ns, ERAM 86/108, DSP 27/40,
  LUT6 10,512; critical path `u_ball_track/out_green_reg`.
- First P&R without the RTC CDC exception: setup +0.162 ns but
  **hold -0.668 ns** on `u_rtc/ack -> sn_day` (50 MHz <-> HDMI CDC).
  That bit is not delivered.
- Added targeted `set_false_path` through `u_rtc/*` between
  `sys_clk_50m` and `HDMI_PIXEL_CLK` in both SDCs (2FF + req/ack
  handshake; snapshot stable until ack).  This is a constraint fix, not
  a functional change.
- Final P&R: setup WNS +0.110 ns, TNS 0; hold WNS +0.020 ns, 0
  violations; slices 18,999/21,216 (89.55%), LUT6 18,461/21,216
  (87.01%), ERAM 86/108, DSP 27/40.  Critical path is the existing
  `u_evt_circle/u_acc` RAM read.
- Unconstrained profile unchanged (1 I2C no-clock root, 37/68 IO
  delays, 4 dangling, 0 invalid).
- Final bit `camera_to_dsi_display_phase2_track.bit`, SHA256
  `05d906fa91daa6ba03fcc1825f793848551d5562604635813e952def4f4f119a`
  (not flashed; older bits preserved).  Resource headroom ~10%;
  future logic additions need a new timing/resource check.

## rev 2m — HUD GREEN G glyph fix + buzzer preview

- `txt_font` 5'd25 G changed to the HUD-style open-mouth pattern
  `3C,66,03,73,63,63,66,3C`; the old full middle bar made it read as S
  at 8 px.  Actual `hdmi_mixer` pixel render (`logs/text_render_green.png`,
  `logs/green_word_zoom.png`) confirms G R E E N and the full
  `CIRCLE GREEN X+123 Y-045`; `tb_circle_text` GREEN case PASS.  No other
  RTL changed; no long unrelated simulations.
- Buzzer preview: `audio_preview/one_more_time_approx_buzzer.wav`
  (mono 16-bit 22050 Hz square, 123 BPM, 17 beats, ~8.29 s) plus its
  generator and README.  It is explicitly an **approximate/inspired**
  excerpt (no reliable score/MIDI verified), not a faithful transcription;
  the passive buzzer is not wired and its type is unverified.

### Build (rev 2m)

Fresh tree `D:\FPGA\phase2_build_20261106`, final G-fixed RTL/SDC snapshot
(diff-verified), EVT SDC, sequential syn then P&R:
- Syn: WNS +0.128 ns, hold +0.114 ns, ERAM 86/108, DSP 27/40,
  LUT6 10,655; critical path `u_ball_track/out_green_reg`.
- P&R: setup WNS +0.135 ns, TNS 0; hold WNS +0.020 ns, 0 violations;
  slices 19,138/21,216 (90.21%), LUT6 18,550/21,216 (87.43%),
  ERAM 86/108, DSP 27/40.  Critical path is the existing
  `u_evt_circle/ls_bx`; no negative hold paths.
- Unconstrained profile unchanged (1 I2C no-clock root, 37/68 IO delays,
  4 dangling, 0 invalid).
- New bit `camera_to_dsi_display_phase2_track_g.bit`, SHA256
  `8d2603606ed666454fbca0962f77b99be04580e34b6955a462eb344a0b00bca3`
  (not flashed; all older bits preserved).
- Resource headroom is now ~9.8%; further logic needs a new build and
  timing/resource check.

## rev 2n/2o — Two Tigers buzzer on the green-box target

- New `isp_two_tigers.v`: 32-entry compact note ROM ({note[2:0],
  duration[2:0]} in 0.25 s half-beats, 120 BPM, 64 units = 16 s), 50 MHz
  square-wave divider, 20 ms articulation gap.  It does not contain/play
  the WAV.
- **rev 2n bug**: the first wiring used `isp_top.O_det_stable` (tracker
  `out_stable`, 10 detections inside a 16 px anchor window).  A moving
  ball may be detected every frame and still never satisfy that static
  gate, so it could never trigger.  Rev 2o removes the extra
  `O_det_stable` port.
- **rev 2o trigger**: the green-box display level `isp_top.O_det_valid`
  (EVT = tracker `out_valid`, fresh/candidate plus its bounded 4-frame
  hold; CCL = `S_ccl_det_valid`) is 2FF-synchronized from the CSI clock
  into the 50 MHz domain; a rising edge starts one song.  A held box only
  keeps the level high -- it creates no new edge, so hold is never
  mistaken for a new detection.  Playing ignores new edges (no
  restart/stacking).  A 1 s absence re-arms; the still-present target
  after the song does not auto-replay.  Reset clears the sequencer and
  forces the output low.
- Pin: `O_buzzer` -> C12 (LVCMOS33, 8 mA); C12 was unused in `pin.adc`.
- `tb_two_tigers.v` (CLK_HZ=500 kHz scaled, real note frequencies and
  16 s duration): reset silence; async target start; first C5, 956-cycle
  period, 20 ms gap; 1/2/0.5-beat durations; 32-note ROM sequence; brief
  loss during playback no restart; completion; no auto-replay; 1 s
  re-arm; mid-song reset silent.  PASS.
- `tb_two_tigers_integration.v` (real `isp_ball_track` + `isp_two_tigers`,
  asynchronous 333.33 kHz CSI / 50 MHz clocks, 60 fps processed frames):
  a moving ball (50 px/frame, `stable` never asserted) still raises the
  green box and starts exactly one song; brief loss while still a moving
  candidate drops the box but never restarts the song; after a static
  track is confirmed, a 2-frame miss is HELD (`out_hold=1`) with no new
  start; full 16 s/32-note sequence, C5 956-cycle period, 20 ms gap,
  1/2/0.5-beat durations; no replay with the target still present;
  absence > 1 s re-arms and reacquire starts a new song at note 0;
  mid-song reset is silent; CDC latency 2 system cycles.  PASS.
- OSD regression: `tb_circle_text.v` PASS, and the final `hdmi_mixer`
  PPM/TXT dumps are byte-identical to the rev-2m baseline
  (`ce89b431...` / `685e1db1...`).  The timing fixes pipeline the
  frame-static text decode and the runtime OSD from `S_x/S_y` (two pixels
  ahead), so neither changes a displayed pixel.
- Detector/HUD/timer/tracking unchanged.  Final build in
  `D:\FPGA\phase2_build_rev2o_two_tigers`: syn +162 ps / hold +114 ps,
  route +137 ps / hold +20 ps, 0 violations, slices 19,150/21,216
  (90.26 %), bit SHA256 `1eae3a4d...e042900` (not flashed).  See research
  section 18 for the full record.
