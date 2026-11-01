#!/usr/bin/env python3
# Phase-2 research model: 4x4 block events -> 2D center vote -> radial histogram.
# This is a design/tuning model, NOT RTL evidence.  It mirrors the intended
# fixed-point/8px-grid algorithm closely enough to test radius/scene limits.
import math, random, sys

W, H = 1280, 720
GRID = 4
AW, AH = W // GRID, H // GRID          # 160 x 90 center cells
RMIN, RMAX = 26, 160

def dir16(dx, dy):
    a = math.atan2(dy, dx)
    return int(round(a / (2 * math.pi) * 16)) & 15

def dir_vec(d):
    a = d * 2 * math.pi / 16
    return math.cos(a), math.sin(a)

def add_circle(events, cx, cy, r, polarity=1, gap_deg=0.0, noise=0.0):
    # one event per 4x4 block whose centre is within ~2.9px of the circle
    for by in range(H // 4):
        for bx in range(W // 4):
            x = bx * 4 + 2.0
            y = by * 4 + 2.0
            dx, dy = x - cx, y - cy
            d = math.hypot(dx, dy)
            if abs(d - r) <= 2.9 and d > 1e-6:
                ang = math.degrees(math.atan2(dy, dx)) % 360
                if gap_deg and (ang < gap_deg or ang > 360 - gap_deg):
                    continue
                # closest point on circle to the block centre, rounded to pixel
                ex = int(round(cx + dx / d * r))
                ey = int(round(cy + dy / d * r))
                if not (0 <= ex < W and 0 <= ey < H):
                    continue
                if noise:
                    ex += random.randint(-noise, noise)
                    ey += random.randint(-noise, noise)
                # gradient points outward; polarity flips it
                events.append((ex, ey, dir16(ex - cx, ey - cy) if polarity > 0 else dir16(cx - ex, cy - ey)))

def add_rect(events, x0, y0, x1, y1):
    # axis-aligned rectangle edges, one event per 4x4 block near the border
    for by in range(H // 4):
        for bx in range(W // 4):
            x = bx * 4 + 2.0
            y = by * 4 + 2.0
            if x0 - 2 <= x <= x1 + 2 and y0 - 2 <= y <= y1 + 2:
                # distance to each side; pick the nearest side if within 2.9
                ds = [(abs(y - y0), 0, (x, y0), 0), (abs(y - y1), 8, (x, y1), 0),
                      (abs(x - x0), 12, (x0, y), 1), (abs(x - x1), 4, (x1, y), 1)]
                d, di, p, _ = min(ds)
                if d <= 2.9:
                    events.append((int(round(p[0])), int(round(p[1])), di))

def add_ellipse(events, cx, cy, a, b):
    for by in range(H // 4):
        for bx in range(W // 4):
            x = bx * 4 + 2.0
            y = by * 4 + 2.0
            # normalized ellipse distance approx
            nx, ny = (x - cx) / a, (y - cy) / b
            d = math.hypot(nx, ny)
            if abs(d - 1.0) < 3.2 / min(a, b) * 1.0:
                # outward normal of ellipse
                gx, gy = nx / a, ny / b
                if abs(gx) + abs(gy) > 1e-9:
                    events.append((int(round(x)), int(round(y)), dir16(gx, gy)))

def add_checker(events, cell=4):
    for by in range(H // 4):
        for bx in range(W // 4):
            x, y = bx * 4 + 2, by * 4 + 2
            # dense texture: every block has an edge, directions alternate
            d = (0 if (bx + by) & 1 else 8)
            if (bx & 1):
                d = 4 if (by & 1) else 12
            events.append((x, y, d))

def center_vote(events):
    acc = [[0] * AH for _ in range(AW)]
    for (ex, ey, d) in events:
        cx, sy = dir_vec(d)
        for pol in (1, -1):
            for k in range(1, 18):
                vx = ex + pol * cx * k * GRID
                vy = ey + pol * sy * k * GRID
                if 0 <= vx < W and 0 <= vy < H:
                    ax, ay = int(vx) // GRID, int(vy) // GRID
                    if 0 <= ax < AW and 0 <= ay < AH:
                        if acc[ax][ay] < 255:
                            acc[ax][ay] += 1
    return acc

def topk(acc, k=4, sep=3):
    cands = []
    for ay in range(AH):
        for ax in range(AW):
            c = acc[ax][ay]
            if c == 0:
                continue
            # sharpness: subtract neighbours
            n = 0
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    xx, yy = ax + dx, ay + dy
                    if 0 <= xx < AW and 0 <= yy < AH:
                        n += acc[xx][yy]
            score = c
            cands.append((score, c, ax, ay))
    cands.sort(reverse=True)
    out = []
    for sc, c, ax, ay in cands:
        if all((ax - x) ** 2 + (ay - y) ** 2 >= sep * sep for _, _, x, y in out):
            out.append((sc, c, ax, ay))
            if len(out) == k:
                break
    return out

def verify(events, centers, sample_half=True, refine=True):
    res = []
    total = 0
    for (sc, c, ax, ay) in centers:
        cx, cy = ax * GRID + GRID / 2, ay * GRID + GRID / 2
        hist = [0] * (RMAX + 3)
        sect = [0] * (RMAX + 3)
        sx = sy = sr = cnt = 0
        for i, (ex, ey, d) in enumerate(events):
            if sample_half and ((ex // 4 + ey // 4) & 1):
                continue
            total += 1
            dx, dy = ex - cx, ey - cy
            r = int(round(math.hypot(dx, dy)))
            if r < RMIN - 1 or r > RMAX + 1:
                continue
            rd = dir16(dx, dy)
            dd = (d - rd) & 15
            if dd in (0, 1, 15, 8, 7, 9):
                if RMIN <= r <= RMAX:
                    hist[r] += 1
                    sect[r] |= 1 << ((rd * 16) // 16)  # 16 sectors = dir bins
                    sx += ex; sy += ey; sr += r; cnt += 1
        if cnt == 0:
            continue
        if refine:
            # second pass with the centroid as the refined centre
            cx2, cy2 = sx / cnt, sy / cnt
            hist = [0] * (RMAX + 3)
            sect = [0] * (RMAX + 3)
            sx = sy = sr = cnt = 0
            for i, (ex, ey, d) in enumerate(events):
                if sample_half and ((ex // 4 + ey // 4) & 1):
                    continue
                dx, dy = ex - cx2, ey - cy2
                r = int(round(math.hypot(dx, dy)))
                if r < RMIN - 1 or r > RMAX + 1:
                    continue
                rd = dir16(dx, dy)
                dd = (d - rd) & 15
                if dd in (0, 1, 15, 8, 7, 9):
                    if RMIN <= r <= RMAX:
                        hist[r] += 1
                        sect[r] |= 1 << rd
                        sx += ex; sy += ey; sr += r; cnt += 1
            if cnt == 0:
                continue
        # peak with +-1 smoothing
        peak_r, peak_v = RMIN, 0
        for r in range(RMIN, RMAX + 1):
            v = hist[r - 1] + hist[r] + hist[r + 1]
            if v > peak_v:
                peak_v, peak_r = v, r
        cov = bin(sect[peak_r] | sect[peak_r - 1] | sect[peak_r + 1]).count("1")
        expected = max(1.0, 2 * math.pi * peak_r / 4.0 / (2 if sample_half else 1))
        fill = peak_v / expected
        sharp = peak_v / max(1.0, peak_v + hist[peak_r - 2] + hist[peak_r + 2])
        align = cnt / max(1, total)
        score = (cov / 16.0) * sharp * min(fill, 1.0)
        res.append(dict(cx=sx / cnt, cy=sy / cnt, gx=cx, gy=cy,
                        r=peak_r, score=score, cov=cov, fill=fill,
                        sharp=sharp, align=align, peak=peak_v, cnt=cnt))
    return res

def run(name, ev, truth=None, sample_half=True):
    acc = center_vote(ev)
    tops = topk(acc)
    res = verify(ev, tops, sample_half)
    res.sort(key=lambda x: x["score"], reverse=True)
    ok = False
    if res:
        b = res[0]
        ok = b["cov"] >= 10 and b["fill"] >= 0.25 and b["peak"] >= 12
    print(f"{name:28s} events={len(ev):6d} " + ("PASS " if ok else "REJECT") +
          (f" best=(cx={b['cx']:.1f},cy={b['cy']:.1f},r={b['r']},cov={b['cov']},fill={b['fill']:.2f},sharp={b['sharp']:.2f},align={b['align']:.2f},score={b['score']:.2f})" if res else ""))
    if truth and res:
        b = res[0]
        print(f"    truth=({truth[0]},{truth[1]},r={truth[2]}) err_c=({b['cx']-truth[0]:+.1f},{b['cy']-truth[1]:+.1f}) err_r={b['r']-truth[2]:+d}")
    return ok, res

if __name__ == "__main__":
    random.seed(1)
    for r in (25, 26, 60, 100, 160):
        for off in (0, 1, 2, 3):
            ev = []
            add_circle(ev, 400 + off, 360 + off, r, 1)
            run(f"circle r{r} off{off}", ev, (400 + off, 360 + off, r))
    # scenes
    ev = []; add_circle(ev, 900, 360, 54); add_ellipse(ev, 400, 360, 80, 50)
    run("circle vs ellipse", ev, (900, 360, 54))
    ev = []; add_circle(ev, 640, 360, 60); add_rect(ev, 200, 200, 500, 500)
    run("circle vs rect", ev, (640, 360, 60))
    ev = []; add_checker(ev)
    run("checkerboard", ev)
    ev = []; add_circle(ev, 640, 360, 60, gap_deg=90)
    run("half arc r60", ev)
    ev = []
    run("empty", ev)
    ev = []; add_circle(ev, 640, 360, 60, polarity=-1)
    run("dark circle r60", ev, (640, 360, 60))
    ev = []; add_circle(ev, 400, 360, 40); add_circle(ev, 900, 360, 100)
    run("two circles r40/r100", ev)
