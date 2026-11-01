#!/usr/bin/env python3
# Direction-quantization study: how many direction bins does the architecture
# need to keep r25/26, r60/r100/r160 and competition results?
import math, random
import circle_model3 as m

def quant(ev, bits):
    n = 1 << bits
    out = []
    for (x, y, ux, uy) in ev:
        a = math.atan2(uy, ux)
        b = int(round(a / (2 * math.pi) * n)) & (n - 1)
        aa = b * 2 * math.pi / n
        out.append((x, y, math.cos(aa), math.sin(aa)))
    return out

def suite(bits):
    print(f"\n===== direction bins = {1<<bits} =====")
    random.seed(1)
    for r in (25, 26, 60, 100, 160):
        for off in (0, 2):
            ev = []; m.add_circle(ev, 400 + off, 360 + off, r)
            m.run(f"r{r} off{off}", quant(ev, bits), (400 + off, 360 + off, r))
    ev = []; m.add_circle(ev, 900, 360, 54); m.add_ellipse(ev, 400, 360, 80, 50)
    m.run("circle vs ellipse", quant(ev, bits), (900, 360, 54))
    ev = []; m.add_circle(ev, 640, 360, 60); m.add_rect(ev, 200, 200, 500, 500)
    m.run("circle vs rect", quant(ev, bits), (640, 360, 60))
    ev = []; m.add_checker(ev)
    m.run("checkerboard", quant(ev, bits))
    ev = []; m.add_circle(ev, 640, 360, 60, gap_deg=90)
    m.run("half arc r60", quant(ev, bits))

if __name__ == "__main__":
    for b in (8, 6, 5, 4):
        suite(b)
