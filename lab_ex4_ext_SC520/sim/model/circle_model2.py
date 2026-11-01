#!/usr/bin/env python3
# Research model v2: sparse event list + coarse center vote + local fine center
# + radial histogram.  Still a model, not RTL evidence.
import math, random
import circle_model as m

def fine_vote(events, cx0, cy0, win=16, step=1):
    acc = [[0] * (2 * win + 1) for _ in range(2 * win + 1)]
    for (ex, ey, d) in events:
        ux, uy = m.dir_vec(d)
        for pol in (1, -1):
            for k in range(1, int(win * 2 / step) + 1):
                vx = ex + pol * ux * k * step
                vy = ey + pol * uy * k * step
                ix = int(round(vx - (cx0 - win)))
                iy = int(round(vy - (cy0 - win)))
                if 0 <= ix <= 2 * win and 0 <= iy <= 2 * win:
                    if acc[ix][iy] < 255:
                        acc[ix][iy] += 1
    best = (-1, 0, 0)
    for iy in range(2 * win + 1):
        for ix in range(2 * win + 1):
            if acc[ix][iy] > best[0]:
                best = (acc[ix][iy], ix, iy)
    return (cx0 - win + best[1], cy0 - win + best[2], best[0])

def verify_at(events, centers, sample_half=True):
    out = []
    total = 0
    for (cx, cy) in centers:
        hist = [0] * (m.RMAX + 3)
        sect = [0] * (m.RMAX + 3)
        sx = sy = sr = cnt = 0
        for (ex, ey, d) in events:
            if sample_half and ((ex // 4 + ey // 4) & 1):
                continue
            total += 1
            dx, dy = ex - cx, ey - cy
            r = int(round(math.hypot(dx, dy)))
            if r < m.RMIN - 1 or r > m.RMAX + 1:
                continue
            rd = m.dir16(dx, dy)
            dd = (d - rd) & 15
            if dd in (0, 1, 15, 8, 7, 9):
                if m.RMIN <= r <= m.RMAX:
                    hist[r] += 1
                    sect[r] |= 1 << rd
                    sx += ex; sy += ey; sr += r; cnt += 1
        if cnt == 0:
            continue
        peak_r, peak_v = m.RMIN, 0
        for r in range(m.RMIN, m.RMAX + 1):
            v = hist[r - 1] + hist[r] + hist[r + 1]
            if v > peak_v:
                peak_v, peak_r = v, r
        cov = bin(sect[peak_r] | sect[peak_r - 1] | sect[peak_r + 1]).count("1")
        expected = max(1.0, 2 * math.pi * peak_r / 4.0 / (2 if sample_half else 1))
        fill = peak_v / expected
        sharp = peak_v / max(1.0, peak_v + hist[peak_r - 2] + hist[peak_r + 2])
        align = cnt / max(1, total)
        score = (cov / 16.0) * sharp * min(fill, 1.0) * min(align * 2.0, 1.0)
        out.append(dict(cx=sx / cnt, cy=sy / cnt, fx=cx, fy=cy,
                        r=peak_r, score=score, cov=cov, fill=fill,
                        sharp=sharp, align=align, peak=peak_v, cnt=cnt))
    return out

def run2(name, ev, truth=None, k=6, sample_half=True):
    acc = m.center_vote(ev)
    tops = m.topk(acc, k=k, sep=2)
    fines = []
    for (sc, c, ax, ay) in tops:
        cx0, cy0 = ax * m.GRID + m.GRID / 2, ay * m.GRID + m.GRID / 2
        fx, fy, fp = fine_vote(ev, cx0, cy0)
        if all((fx - x) ** 2 + (fy - y) ** 2 > 16 for x, y, _ in fines):
            fines.append((fx, fy, fp))
    res = verify_at(ev, [(x, y) for x, y, _ in fines], sample_half)
    res.sort(key=lambda x: x["score"], reverse=True)
    ok = False
    b = res[0] if res else None
    if b:
        ok = b["cov"] >= 10 and b["fill"] >= 0.25 and b["peak"] >= 12 and b["align"] >= 0.15
    print(f"{name:28s} ev={len(ev):6d} " + ("PASS " if ok else "REJECT") +
          (f" best=(c={b['cx']:.1f},{b['cy']:.1f} r={b['r']} cov={b['cov']} fill={b['fill']:.2f} sharp={b['sharp']:.2f} align={b['align']:.2f} peak={b['peak']} score={b['score']:.2f})" if b else ""))
    if truth and b:
        print(f"    truth=({truth[0]},{truth[1]},r={truth[2]}) err_c=({b['cx']-truth[0]:+.1f},{b['cy']-truth[1]:+.1f}) err_r={b['r']-truth[2]:+d}")
    return ok, res

if __name__ == "__main__":
    random.seed(1)
    for r in (25, 26, 60, 100, 160):
        for off in (0, 1, 2, 3):
            ev = []
            m.add_circle(ev, 400 + off, 360 + off, r, 1)
            run2(f"circle r{r} off{off}", ev, (400 + off, 360 + off, r))
    ev = []; m.add_circle(ev, 900, 360, 54); m.add_ellipse(ev, 400, 360, 80, 50)
    run2("circle vs ellipse", ev, (900, 360, 54))
    ev = []; m.add_circle(ev, 640, 360, 60); m.add_rect(ev, 200, 200, 500, 500)
    run2("circle vs rect", ev, (640, 360, 60))
    ev = []; m.add_checker(ev)
    run2("checkerboard", ev)
    ev = []; m.add_circle(ev, 640, 360, 60, gap_deg=90)
    run2("half arc r60", ev)
    run2("empty", [])
    ev = []; m.add_circle(ev, 640, 360, 60, polarity=-1)
    run2("dark circle r60", ev, (640, 360, 60))
    ev = []; m.add_circle(ev, 400, 360, 40); m.add_circle(ev, 900, 360, 100)
    run2("two circles r40/r100", ev)
