#!/usr/bin/env python3
# Pixel-level front-end + core research model (phase 2, 2026-11-01).
# Mirrors the RTL structure so front-end variants and the radial Gauss-Newton
# fine-centre scheme can be compared before touching RTL.
import math, sys
import numpy as np

W, H = 1280, 720
EDGE_TH = 48
GRID = 4
AW, AH = W // GRID, H // GRID
RMIN, RMAX = 26, 160


# ---------------- image model ----------------
def disk(cx, cy, r, ins=200, out=40):
    yy, xx = np.mgrid[0:H, 0:W]
    d2 = (xx - cx) ** 2 + (yy - cy) ** 2
    img = np.full((H, W), out, np.int16)
    img[d2 <= r * r] = ins
    return img


def arc(cx, cy, r, gap_deg, ins=200, out=40, gap_side=0.0):
    img = disk(cx, cy, r, ins, out)
    yy, xx = np.mgrid[0:H, 0:W]
    ang = (np.degrees(np.arctan2(yy - cy, xx - cx)) - gap_side) % 360.0
    wedge = (ang < gap_deg / 2) | (ang > 360 - gap_deg / 2)
    img[wedge] = out
    return img


def rect(cx, cy, hw, hh, ins=200, out=40):
    img = np.full((H, W), out, np.int16)
    img[cy - hh:cy + hh, cx - hw:cx + hw] = ins
    return img


def ring(cx, cy, r, w, ins=200, out=40):
    yy, xx = np.mgrid[0:H, 0:W]
    d2 = (xx - cx) ** 2 + (yy - cy) ** 2
    img = np.full((H, W), out, np.int16)
    m = (d2 <= (r + w) ** 2) & (d2 >= (r - w) ** 2)
    img[m] = ins
    return img


def ellipse(cx, cy, a, b, ins=200, out=40):
    yy, xx = np.mgrid[0:H, 0:W]
    img = np.full((H, W), out, np.int16)
    img[((xx - cx) / a) ** 2 + ((yy - cy) / b) ** 2 <= 1.0] = ins
    return img


def checker(cell=8, ins=200, out=40):
    yy, xx = np.mgrid[0:H, 0:W]
    return np.where((((xx // cell) + (yy // cell)) & 1) == 0, ins, out).astype(np.int16)


# ---------------- Sobel ----------------
def sobel_all(img):
    # 3x3 Sobel on the whole image (int), edges replicate.
    p = np.pad(img.astype(np.int32), 1, mode='edge')
    a = p[:-2, :-2]; b = p[:-2, 1:-1]; c = p[:-2, 2:]
    d = p[1:-1, :-2]; f = p[1:-1, 2:]
    g = p[2:, :-2]; h = p[2:, 1:-1]; i = p[2:, 2:]
    gx = (c + 2 * f + i) - (a + 2 * d + g)
    gy = (g + 2 * h + i) - (a + 2 * b + c)
    return gx, gy


def to_q8(gx, gy):
    ax, ay = abs(gx), abs(gy)
    mag = max(ax, ay) + (min(ax, ay) >> 1)
    if mag == 0:
        return 0, 0
    ux = int(round(gx * 256.0 / mag))
    uy = int(round(gy * 256.0 / mag))
    ux = max(-256, min(255, ux)); uy = max(-256, min(255, uy))
    return ux, uy


# ---------------- front end ----------------
def frontend(img, variant='max'):
    """Return events (x,y,ux,uy) for one frame.
    variant:
      'max'   : current RTL: strongest of 6 samples, its own direction
      'sum'   : block-sum direction, position = strongest sample
      'sumc'  : block-sum direction, position = block centre
      'sumw'  : block-sum direction, weighted-centroid position (sub-block -> round)
    Only rows y0+1..y0+3 and columns x0+1,x0+2 are sampled (RTL window).
    """
    gx, gy = sobel_all(img)
    ev = []
    for k in range(H // 4):
        y0 = k * 4
        for bx in range(W // 4):
            x0 = bx * 4
            smax = -1; sxp = x0 + 1; syp = y0 + 1; sgx = 0; sgy = 0
            sx = sy = sw = 0
            agx = agy = 0
            for yy in range(y0 + 1, y0 + 4):
                for xx in (x0 + 1, x0 + 2):
                    a = int(gx[yy, xx]); b = int(gy[yy, xx])
                    m = abs(a) + abs(b)
                    agx += a; agy += b
                    if m > smax:
                        smax = m; sxp = xx; syp = yy; sgx = a; sgy = b
                    sx += m * xx; sy += m * yy; sw += m
            if smax < EDGE_TH:
                continue
            if variant == 'max':
                px, py = sxp, syp
                ux, uy = to_q8(sgx, sgy)
            else:
                ux, uy = to_q8(agx, agy)
                if variant == 'sum':
                    px, py = sxp, syp
                elif variant == 'sumc':
                    px, py = x0 + 2, y0 + 2
                else:
                    px, py = int(round(sx / sw)), int(round(sy / sw))
            ev.append((px, py, ux, uy))
    return ev


# ---------------- direction diagnostics ----------------
def dir_stats(ev, cx, cy, r):
    off = bad = 0
    errs = []
    for (x, y, ux, uy) in ev:
        dx, dy = x - cx, y - cy
        d = math.hypot(dx, dy)
        if abs(d - r) > 4:
            off += 1
            continue
        dot = (ux * dx + uy * dy) / (256.0 * d)
        ang = math.degrees(math.acos(max(-1.0, min(1.0, abs(dot)))))
        ang = min(ang, 180 - ang)
        errs.append(ang)
        if ang > 14:
            bad += 1
    return off, bad, errs


# ---------------- core model ----------------
def coarse(ev, step=8, nmax=None, sat=255):
    acc = np.zeros((AH, AW), np.int32)
    for (x, y, ux, uy) in ev:
        uxx, uyy = ux / 256.0, uy / 256.0
        for pol in (1, -1):
            for k in range(1, 21):
                vx = int(round(x + pol * uxx * k * step))
                vy = int(round(y + pol * uyy * k * step))
                if 0 <= vx < W and 0 <= vy < H:
                    ax, ay = vx // GRID, vy // GRID
                    if acc[ay, ax] < sat:
                        acc[ay, ax] += 1
    return acc


def topk(acc, k=4, sep=2):
    idx = np.argwhere(acc > 0)
    vals = acc[idx[:, 0], idx[:, 1]]
    order = np.argsort(-vals)
    out = []
    for o in order:
        ay, ax = idx[o]
        if all(max(abs(ax - x), abs(ay - y)) >= sep for _, x, y in out):
            out.append((int(vals[o]), int(ax), int(ay)))
            if len(out) == k:
                break
    return out


def radial_hist(ev, cx, cy, gate_deg=35.0):
    hist = np.zeros(RMAX + 4, np.int32)
    sect = np.zeros(RMAX + 4, np.int64)
    cg = math.cos(math.radians(gate_deg))
    for (x, y, ux, uy) in ev:
        dx, dy = x - cx, y - cy
        r2 = dx * dx + dy * dy
        if r2 < (RMIN - 4) ** 2 or r2 > (RMAX + 4) ** 2:
            continue
        r = math.sqrt(r2)
        if r == 0:
            continue
        dot = (ux * dx + uy * dy) / (256.0 * r)
        if abs(dot) < cg:
            continue
        ri = int(round(r))
        if RMIN <= ri <= RMAX:
            hist[ri] += 1
            sect[ri] |= 1 << (int((math.atan2(dy, dx) / (2 * math.pi) * 16)) & 15)
    return hist, sect


def peak_hist(hist):
    pv, pr = 0, RMIN
    for r in range(RMIN, RMAX + 1):
        v = hist[r - 1] + hist[r] + hist[r + 1]
        if v > pv:
            pv, pr = v, r
    return pv, pr


def refine(ev, cx, cy, r0, band=6, iters=2, ridge=64.0, maxstep=8.0,
           gate=True):
    """Radial Gauss-Newton: minimise sum (|p-c|-r)^2.  Direction-agnostic."""
    cg = math.cos(math.radians(45.0))
    n = 0
    for it in range(iters):
        hxx = hxy = hyy = gx_ = gy_ = 0.0
        n = 0
        for (x, y, ux, uy) in ev:
            dx, dy = x - cx, y - cy
            rho = math.hypot(dx, dy)
            if rho < 1 or abs(rho - r0) > band:
                continue
            if gate and abs((ux * dx + uy * dy) / (256.0 * rho)) < cg:
                continue
            e = rho - r0
            w = 1.0 / (rho * rho)
            hxx += dx * dx * w; hxy += dx * dy * w; hyy += dy * dy * w
            gx_ += e * dx / rho; gy_ += e * dy / rho
            n += 1
        if n < 8:
            break
        lam = ridge * n
        hxx += lam; hyy += lam
        det = hxx * hyy - hxy * hxy
        if abs(det) < 1e-9:
            break
        sx = (gx_ * hyy - gy_ * hxy) / det
        sy = (hxx * gy_ - hxy * gx_) / det
        sx = max(-maxstep, min(maxstep, sx))
        sy = max(-maxstep, min(maxstep, sy))
        cx += sx; cy += sy
        # update radius as the mean of inlier radii
        rs = [math.hypot(x - cx, y - cy) for (x, y, ux, uy) in ev]
        rs = [r for r in rs if abs(r - r0) <= band + 3]
        if rs:
            r0 = sum(rs) / len(rs)
    return cx, cy, r0, n


def evaluate(ev, cx, cy, peak_v, peak_r, sect, cnt_align, proc):
    cov = bin(sect[peak_r - 2] | sect[peak_r - 1] | sect[peak_r] |
              sect[peak_r + 1] | sect[peak_r + 2]).count("1")
    exp = 2 * math.pi * peak_r / 4.0
    fill = peak_v / max(1.0, exp)
    rexp = (201 * peak_r) >> 7          # RTL: 201*r/128
    ok = (cov >= 10 and peak_v >= 12 and peak_v * 4 >= rexp and
          cnt_align * 8 >= proc)
    return cov, fill, ok


def run(img, label, truth, variant='sum', verbose=False, k=4):
    ev = frontend(img, variant)
    if not ev:
        if verbose: print(f"{label:28s} {variant:5s} ev=0      REJECT")
        return False, None
    acc = coarse(ev)
    tops = topk(acc, k=k)
    cands = []
    for (v, ax, ay) in tops:
        cx0, cy0 = ax * GRID + 2.0, ay * GRID + 2.0
        hist, sect = radial_hist(ev, cx0, cy0)
        pv, pr = peak_hist(hist)
        if pr == RMIN and pv == 0:
            continue
        cx, cy, r0, n = refine(ev, cx0, cy0, float(pr))
        hist, sect = radial_hist(ev, cx, cy)
        pv, pr = peak_hist(hist)
        cnt = int(hist[int(RMIN):int(RMAX) + 1].sum())
        cov, fill, ok = evaluate(ev, cx, cy, pv, pr, sect, cnt, len(ev))
        score = (min(cov, 16) / 16.0) * min(fill, 1.0)
        cands.append(dict(cx=cx, cy=cy, r=pr, cov=cov, fill=fill, peak=pv,
                          ok=ok, score=score, coarse=(cx0, cy0), n=n))
    cands.sort(key=lambda c: (c['score'], -c['r']), reverse=True)
    b = cands[0] if cands else None
    ok = bool(b and b['ok'])
    if verbose:
        s = f"{label:28s} {variant:5s} ev={len(ev):4d} " + ("PASS " if ok else "REJECT")
        if b:
            s += (f" c=({b['cx']:.0f},{b['cy']:.0f}) r={b['r']} cov={b['cov']} "
                  f"fill={b['fill']:.2f} peak={b['peak']} coarse=({b['coarse'][0]:.0f},{b['coarse'][1]:.0f})")
            if truth:
                s += f" err_c=({b['cx']-truth[0]:+.0f},{b['cy']-truth[1]:+.0f}) err_r={b['r']-truth[2]:+d}"
        print(s)
    return ok, b


def scenarios():
    out = []
    for r in (25, 26, 60, 100, 160):
        for off in (0, 1, 2, 3):
            out.append((f"r{r}_off{off}", disk(400 + off, 360 + off, r),
                        (400 + off, 360 + off, r), r >= 26))
    out.append(("ellipse", ellipse(400, 360, 80, 50), None, False))
    out.append(("rect_no_circle", rect(640, 360, 200, 200), None, False))
    out.append(("checker8", checker(8), None, False))
    out.append(("arc90_r60", arc(640, 360, 60, 90), None, False))
    out.append(("arc45_r60", arc(640, 360, 60, 45), None, True))
    out.append(("ring_r60_w6", ring(640, 360, 60, 6), (640, 360, 60), True))
    out.append(("empty", np.full((H, W), 40, np.int16), None, False))
    out.append(("dark_r60", disk(640, 360, 60, ins=40, out=200), (640, 360, 60), True))
    return out


if __name__ == "__main__":
    variants = sys.argv[1:] or ['max', 'sum', 'sumc', 'sumw']
    for v in variants:
        print(f"===== variant {v} =====")
        for (name, img, truth, exp) in scenarios():
            ok, b = run(img, name, truth, v, verbose=True)
            if (ok != exp):
                print(f"    !! unexpected: expected {'PASS' if exp else 'REJECT'}")
        # direction stats on a few circles
        for r in (26, 60, 100, 160):
            ev = frontend(disk(640, 360, r), v)
            off, bad, errs = dir_stats(ev, 640, 360, r)
            errs.sort()
            p90 = errs[int(0.9 * (len(errs) - 1))] if errs else 0
            print(f"  dir r{r}: n={len(ev)} off4={off} bad14={bad} "
                  f"({100.0*bad/max(1,len(ev)):.0f}%) p90={p90:.1f}")
