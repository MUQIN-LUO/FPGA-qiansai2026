# Phase-2 circle-detection research model

**This is a Python algorithm/design model, not RTL, not synthesis, not board
evidence.**  See `../../PHASE2_CIRCLE_RESEARCH_20261101.md`.

## Run

```bat
cd lab_ex4_ext_SC520\sim\model
python run_phase2_suite.py
```

Python 3.8+; pure Python, no third-party packages.  The suite prints a CSV
table and a summary; `phase2_model_results.log` is a saved run.

## Files

| File | Purpose |
|---|---|
| `circle_model3.py` | 4×4 block events → 4px center vote → top-K → projection fine center → 1px radial histogram → score |
| `circle_model4.py` | direction quantization study (256/128/64/32 bins) |
| `circle_model2.py` | earlier projection/variant experiments (kept for comparison) |
| `circle_model.py` | first 8px/coarse model (kept for comparison) |
| `run_phase2_suite.py` | concise 27-case suite (r25/r26/r60/r100/r160, competition, arc, empty, polarity, two circles) |

## Current result (model only)

- 8-bit quantized direction, top-6, projection fine center, 1px bins:
  **27/27** cases match expectation/tolerance.
- r25 rejected in 4 phases; r26 passes all 4 at r=26; r60/r100/r160 pass;
  circle vs ellipse/rect pass; checkerboard/half-arc/empty reject.
- 6-bit direction is insufficient (10/20 circles, circle-vs-rect fails).

**Not done:** event-level RTL core, `O_dg_y4` front end, RTL directed tests,
synthesis, timing, board.  Do not quote the model as hardware evidence.
