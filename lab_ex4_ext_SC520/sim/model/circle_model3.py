#!/usr/bin/env python3
# Research model v3: exact signed-gradient direction, no 16-bin quantization.
# Tests the algorithmic ceiling before choosing RTL direction precision.
import math, random

W, H = 1280, 720
GRID = 4
AW, AH = W // GRID, H // GRID
RMIN, RMAX = 26, 160

def add_circle(ev, cx, cy, r, polarity=1, gap_deg=0.0):
    for by in range(H // 4):
        for bx in range(W // 4):
            x, y = bx * 4 + 2.0, by * 4 + 2.0
            dx, dy = x - cx, y - cy
            d = math.hypot(dx, dy)
            if abs(d - r) <= 2.9 and d > 1e-6:
                ang = math.degrees(math.atan2(dy, dx)) % 360
                if gap_deg and (ang < gap_deg or ang > 360 - gap_deg):
                    continue
                ex, ey = int(round(cx + dx / d * r)), int(round(cy + dy / d * r))
                if 0 <= ex < W and 0 <= ey < H:
                    ux, uy = (ex - cx) / r, (ey - cy) / r
                    n = math.hypot(ux, uy) or 1
                    ev.append((ex, ey, polarity * ux / n, polarity * uy / n))

def add_rect(ev, x0, y0, x1, y1):
    for by in range(H // 4):
        for bx in range(W // 4):
            x, y = bx * 4 + 2.0, by * 4 + 2.0
            if x0 - 2 <= x <= x1 + 2 and y0 - 2 <= y <= y1 + 2:
                ds = [(abs(y - y0), (x, y0), (0, -1)), (abs(y - y1), (x, y1), (0, 1)),
                      (abs(x - x0), (x0, y), (-1, 0)), (abs(x - x1), (x1, y), (1, 0))]
                d, p, u = min(ds)
                if d <= 2.9:
                    ev.append((int(round(p[0])), int(round(p[1])), u[0], u[1]))

def add_ellipse(ev, cx, cy, a, b):
    for by in range(H // 4):
        for bx in range(W // 4):
            x, y = bx * 4 + 2.0, by * 4 + 2.0
            nx, ny = (x - cx) / a, (y - cy) / b
            d = math.hypot(nx, ny)
            if abs(d - 1.0) < 3.0 / min(a, b):
                gx, gy = nx / a, ny / b
                n = math.hypot(gx, gy) or 1
                ev.append((int(round(x)), int(round(y)), gx / n, gy / n))

def add_checker(ev):
    for by in range(H // 4):
        for bx in range(W // 4):
            x, y = bx * 4 + 2, by * 4 + 2
            if (bx + by) & 1:
                u = (1.0, 0.0)
            else:
                u = (0.0, 1.0)
            ev.append((x, y, u[0], u[1]))

def center_vote(ev, step=8):
    acc = [[0] * AH for _ in range(AW)]
    for (ex, ey, ux, uy) in ev:
        for pol in (1, -1):
            for k in range(1, int(RMAX / step) + 2):
                vx, vy = ex + pol * ux * k * step, ey + pol * uy * k * step
                if 0 <= vx < W and 0 <= vy < H:
                    ax, ay = int(vx) // GRID, int(vy) // GRID
                    if 0 <= ax < AW and 0 <= ay < AH and acc[ax][ay] < 255:
                        acc[ax][ay] += 1
    return acc

def topk(acc, k=6, sep=2):
    c = []
    for ay in range(AH):
        for ax in range(AW):
            if acc[ax][ay]:
                c.append((acc[ax][ay], ax, ay))
    c.sort(reverse=True)
    out = []
    for v, ax, ay in c:
        if all((ax - x) ** 2 + (ay - y) ** 2 >= sep * sep for _, x, y in out):
            out.append((v, ax, ay))
            if len(out) == k:
                break
    return out

def fine_vote(ev, cx0, cy0, win=16, step=1):
    # Project the coarse centre onto each event's normal line, then step +-win.
    acc = [[0] * (2 * win + 1) for _ in range(2 * win + 1)]
    for (ex, ey, ux, uy) in ev:
        t = (cx0 - ex) * ux + (cy0 - ey) * uy
        px, py = ex + t * ux, ey + t * uy
        for j in range(-win, win + 1, step):
            vx, vy = px + j * ux, py + j * uy
            ix, iy = int(round(vx - (cx0 - win))), int(round(vy - (cy0 - win)))
            if 0 <= ix <= 2 * win and 0 <= iy <= 2 * win and acc[ix][iy] < 255:
                acc[ix][iy] += 1
    best = (-1, 0, 0)
    for iy in range(2 * win + 1):
        for ix in range(2 * win + 1):
            if acc[ix][iy] > best[0]:
                best = (acc[ix][iy], ix, iy)
    return (cx0 - win + best[1], cy0 - win + best[2], best[0])

def verify(ev, centers, sample_half=True):
    out = []
    total = 0
    for (cx, cy) in centers:
        hist = [0] * (RMAX + 3)
        sect = [0] * (RMAX + 3)
        sx = sy = cnt = 0
        for (ex, ey, ux, uy) in ev:
            if sample_half and ((ex // 4 + ey // 4) & 1):
                continue
            total += 1
            dx, dy = ex - cx, ey - cy
            r = int(round(math.hypot(dx, dy)))
            if r < RMIN - 1 or r > RMAX + 1 or r == 0:
                continue
            rx, ry = dx / r, dy / r
            dot = ux * rx + uy * ry
            if abs(dot) >= math.cos(math.radians(20)):
                if RMIN <= r <= RMAX:
                    hist[r] += 1
                    sect[r] |= 1 << (int((math.atan2(dy, dx) / (2 * math.pi) * 16)) & 15)
                    sx += ex; sy += ey; cnt += 1
        if cnt == 0:
            continue
        pr, pv = RMIN, 0
        for r in range(RMIN, RMAX + 1):
            v = hist[r - 1] + hist[r] + hist[r + 1]
            if v > pv:
                pv, pr = v, r
        cov = bin(sect[pr] | sect[pr-1] | sect[pr+1] | sect[pr-2] | sect[pr+2]).count("1")
        expected = max(1.0, 2 * math.pi * pr / 4.0 / (2 if sample_half else 1))
        fill = pv / expected
        sharp = pv / max(1.0, pv + hist[pr - 2] + hist[pr + 2])
        align = cnt / max(1, total)
        score = (cov / 16.0) * sharp * min(fill, 1.0)
        out.append(dict(cx=cx, cy=cy, r=pr, cov=cov, fill=fill,
                        sharp=sharp, align=align, peak=pv, score=score))
    return out

def run(name, ev, truth=None, k=6, fine=True):
    acc = center_vote(ev)
    tops = topk(acc, k=k)
    centers = []
    for (v, ax, ay) in tops:
        cx0, cy0 = ax * GRID + GRID / 2, ay * GRID + GRID / 2
        if fine:
            fx, fy, _ = fine_vote(ev, cx0, cy0)
            if all((fx - x) ** 2 + (fy - y) ** 2 > 16 for x, y in centers):
                centers.append((fx, fy))
        else:
            centers.append((cx0, cy0))
    res = verify(ev, centers)
    res.sort(key=lambda x: (x["score"], -x["r"]), reverse=True)
    b = res[0] if res else None
    ok = bool(b and b["cov"] >= 10 and b["fill"] >= 0.25 and b["peak"] >= 12 and b["align"] >= 0.15)
    print(f"{name:24s} ev={len(ev):6d} " + ("PASS " if ok else "REJECT") +
          (f" c=({b['cx']:.1f},{b['cy']:.1f}) r={b['r']} cov={b['cov']} fill={b['fill']:.2f} sharp={b['sharp']:.2f} align={b['align']:.2f} peak={b['peak']} score={b['score']:.2f}" if b else ""))
    if truth and b:
        print(f"    truth=({truth[0]},{truth[1]},r={truth[2]}) err_c=({b['cx']-truth[0]:+.1f},{b['cy']-truth[1]:+.1f}) err_r={b['r']-truth[2]:+d}")
    return ok, res

if __name__ == "__main__":
    random.seed(1)
    for r in (25, 26, 60, 100, 160):
        for off in (0, 1, 2, 3):
            ev = []; add_circle(ev, 400 + off, 360 + off, r)
            run(f"circle r{r} off{off}", ev, (400 + off, 360 + off, r))
    ev = []; add_circle(ev, 900, 360, 54); add_ellipse(ev, 400, 360, 80, 50)
    run("circle vs ellipse", ev, (900, 360, 54))
    ev = []; add_circle(ev, 640, 360, 60); add_rect(ev, 200, 200, 500, 500)
    run("circle vs rect", ev, (640, 360, 60))
    ev = []; add_checker(ev)
    run("checkerboard", ev)
    ev = []; add_circle(ev, 640, 360, 60, gap_deg=90)
    run("half arc r60", ev)
    run("empty", [])
    ev = []; add_circle(ev, 640, 360, 60, polarity=-1)
    run("dark circle r60", ev, (640, 360, 60))
    ev = []; add_circle(ev, 400, 360, 40); add_circle(ev, 900, 360, 100)
    run("two circles r40/r100", ev)
