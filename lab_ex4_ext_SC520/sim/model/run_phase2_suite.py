#!/usr/bin/env python3
# Concise phase-2 research suite.  Model only; NOT RTL/synthesis evidence.
import io, contextlib, math, random
import circle_model3 as m
from circle_model4 import quant

random.seed(1)
rows = []

def case(name, ev, truth=None, expect=True, tol_c=3, tol_r=2):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        ok, res = m.run(name, ev, truth, k=6, fine=True)
    b = res[0] if res else None
    err_c = err_r = None
    if truth and b:
        err_c = math.hypot(b["cx"] - truth[0], b["cy"] - truth[1])
        err_r = abs(b["r"] - truth[2])
    good = (ok == expect)
    if expect and truth is not None:
        good = good and (err_c is not None and err_c <= tol_c and err_r <= tol_r)
    rows.append((name, "PASS" if ok else "REJECT", expect, err_c, err_r, good))
    return good

nok = 0; n = 0
for r in (25,):
    for off in (0,1,2,3):
        ev=[]; m.add_circle(ev,400+off,360+off,r)
        n+=1; nok += case(f"r{r} off{off}", quant(ev,8), (400+off,360+off,r), expect=False)
for r in (26,60,100,160):
    for off in (0,1,2,3):
        ev=[]; m.add_circle(ev,400+off,360+off,r)
        n+=1; nok += case(f"r{r} off{off}", quant(ev,8), (400+off,360+off,r), expect=True,
                          tol_c=3, tol_r=2)
ev=[]; m.add_circle(ev,900,360,54); m.add_ellipse(ev,400,360,80,50)
n+=1; nok += case("circle vs ellipse", quant(ev,8), (900,360,54), expect=True)
ev=[]; m.add_circle(ev,640,360,60); m.add_rect(ev,200,200,500,500)
n+=1; nok += case("circle vs rect", quant(ev,8), (640,360,60), expect=True)
ev=[]; m.add_checker(ev)
n+=1; nok += case("checkerboard", quant(ev,8), None, expect=False)
ev=[]; m.add_circle(ev,640,360,60,gap_deg=90)
n+=1; nok += case("half arc r60", quant(ev,8), None, expect=False)
n+=1; nok += case("empty", [], None, expect=False)
ev=[]; m.add_circle(ev,640,360,60,polarity=-1)
n+=1; nok += case("dark circle r60", quant(ev,8), (640,360,60), expect=True)
ev=[]; m.add_circle(ev,400,360,40); m.add_circle(ev,900,360,100)
n+=1; nok += case("two circles r40/r100", quant(ev,8), None, expect=True)

print("case,model,expect,err_c,err_r,ok")
for name, got, exp, ec, er, good in rows:
    print(f"{name},{got},{'PASS' if exp else 'REJECT'},"
          f"{'' if ec is None else f'{ec:.1f}'},{'' if er is None else er},{'OK' if good else 'FAIL'}")
print(f"SUMMARY {nok}/{n} cases matched expectation/tolerance")
print("NOTE: synthetic 4x4 events, ideal 8-bit quantized directions; no RTL, no synthesis.")
