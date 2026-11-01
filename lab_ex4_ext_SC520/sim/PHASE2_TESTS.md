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
