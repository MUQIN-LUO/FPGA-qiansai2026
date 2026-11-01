#!/usr/bin/env python3
# Generate 48-bit event vectors for the event-level RTL TB.
import math
import circle_model3 as m
from circle_model4 import quant

def emit(path, ev, label, cap=2048):
    q = quant(ev, 8)[:cap]
    words = []
    for (x, y, ux, uy) in q:
        iux = max(-256, min(255, int(round(ux * 256))))
        iuy = max(-256, min(255, int(round(uy * 256))))
        sec = int(round(math.degrees(math.atan2(uy, ux)) / 360.0 * 16)) & 15
        w = ((x & 0xFFF) << 36) | ((y & 0xFFF) << 24) | \
            ((iux & 0x3FF) << 14) | ((iuy & 0x3FF) << 4) | sec
        words.append(w)
    with open(path, 'w') as f:
        f.write(f'{len(words):012x}\n')
        for w in words:
            f.write(f'{w:012x}\n')
    print(f'{label}: {len(words)} events -> {path}')

emit('ev_r25.hex', (lambda e: (m.add_circle(e,400,360,25), e)[1])([]), 'r25')
emit('ev_r26.hex', (lambda e: (m.add_circle(e,400,360,26), e)[1])([]), 'r26')
# rev 2j radius boundary: accept rho >= 33 (strictly > 32 px).
emit('ev_r32.hex', (lambda e: (m.add_circle(e,640,360,32), e)[1])([]), 'r32')
emit('ev_r33.hex', (lambda e: (m.add_circle(e,640,360,33), e)[1])([]), 'r33')
emit('ev_r34.hex', (lambda e: (m.add_circle(e,640,360,34), e)[1])([]), 'r34')
emit('ev_r60.hex', (lambda e: (m.add_circle(e,640,360,60), e)[1])([]), 'r60')
emit('ev_r100.hex', (lambda e: (m.add_circle(e,640,360,100), e)[1])([]), 'r100')
emit('ev_r160.hex', (lambda e: (m.add_circle(e,640,360,160), e)[1])([]), 'r160')
ev=[]; m.add_circle(ev,900,360,54); m.add_ellipse(ev,400,360,80,50)
emit('ev_ellipse.hex', ev, 'circle+ellipse')
ev=[]; m.add_circle(ev,640,360,60); m.add_rect(ev,200,200,500,500)
emit('ev_rect.hex', ev, 'circle+rect')
ev=[]; m.add_checker(ev)
emit('ev_checker.hex', ev, 'checker(cap2048)')
ev=[]; m.add_circle(ev,640,360,60,gap_deg=90)
emit('ev_arc.hex', ev, 'half arc')
emit('ev_empty.hex', [], 'empty')
ev=[]; m.add_circle(ev,640,360,60,polarity=-1)
emit('ev_dark.hex', ev, 'dark circle')
ev=[]; m.add_circle(ev,400,360,40); m.add_circle(ev,900,360,100)
emit('ev_two.hex', ev, 'two circles')
