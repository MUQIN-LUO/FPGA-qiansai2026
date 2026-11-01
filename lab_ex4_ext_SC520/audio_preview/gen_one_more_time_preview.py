#!/usr/bin/env python3
# Generate a short, single-voice square-wave preview for a passive buzzer.
#
# IMPORTANT / 说明:
#   This is an APPROXIMATE, "inspired by" melodic excerpt in the spirit of
#   Daft Punk's "One More Time" (123 BPM, B-minor pentatonic contour of the
#   vocal hook).  It is NOT a verified transcription of the original notes:
#   no reliable score/MIDI was checked in this session.  Do not present it as
#   a faithful reproduction.
#
#   The WAV is mono, 16-bit PCM, 22050 Hz, one note at a time (square wave),
#   with short attack/release ramps and gaps for articulation.  It is a
#   listening preview only; the passive-buzzer hardware is not wired/tested.
#
# Run:  python gen_one_more_time_preview.py
# Out:  one_more_time_approx_buzzer.wav (same directory)

import math
import os
import struct
import wave

SR = 22050
BPM = 123.0
BEAT = 60.0 / BPM
AMP = 0.28          # of full scale
ATTACK = 0.005      # s
RELEASE = 0.005     # s
GAP = 0.025         # s between notes

# Equal-tempered frequencies (A4=440), octave 5 mostly (740..1175 Hz).
F = {
    "F#5": 739.99,
    "A5":  880.00,
    "B5":  987.77,
    "C#6": 1108.73,
    "D6":  1174.66,
}

# (note, beats) -- approximate/inspired phrase, not a transcription
MELODY = [
    # "one more time"
    ("F#5", 0.5), ("F#5", 0.5), ("F#5", 1.0),
    # "we're gonna celebrate"
    ("A5", 0.5), ("B5", 0.5), ("D6", 0.5), ("C#6", 0.5), ("B5", 1.0),
    # "oh yeah"
    ("F#5", 0.5), ("A5", 0.5), ("B5", 1.0),
    # "don't stop the dancing"
    ("D6", 0.5), ("C#6", 0.5), ("B5", 0.5), ("A5", 0.5), ("F#5", 1.0),
    # short repeat of the first 8 beats + a held closing note
    ("F#5", 0.5), ("F#5", 0.5), ("F#5", 1.0),
    ("A5", 0.5), ("B5", 0.5), ("D6", 0.5), ("C#6", 0.5), ("B5", 1.0),
    ("B5", 2.0),
]


def square_samples(freq, dur, on_dur):
    """Return float samples for a square note, with short ramps."""
    n_on = max(1, int(on_dur * SR))
    n_all = max(1, int(dur * SR))
    out = [0.0] * n_all
    period = SR / freq
    for i in range(n_on):
        phase = (i % period) / period
        v = AMP if phase < 0.5 else -AMP
        # linear attack/release to avoid clicks
        if i < ATTACK * SR:
            v *= i / (ATTACK * SR)
        elif i > n_on - RELEASE * SR:
            v *= max(0.0, (n_on - i) / (RELEASE * SR))
        out[i] = v
    return out


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(here, "one_more_time_approx_buzzer.wav")
    samples = []
    for name, beats in MELODY:
        dur = beats * BEAT
        on_dur = max(0.02, dur - GAP)
        samples.extend(square_samples(F[name], dur, on_dur))

    pcm = b"".join(
        struct.pack("<h", max(-32768, min(32767, int(round(v * 32767)))))
        for v in samples
    )
    with wave.open(out_path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SR)
        w.writeframes(pcm)
    print(f"wrote {out_path}: {len(samples)/SR:.2f} s, {len(pcm)} bytes, "
          f"{BPM:.0f} BPM")
    for name, beats in MELODY:
        print(f"  {name:4s} {F[name]:7.2f} Hz  {beats:3.1f} beat "
              f"({beats*BEAT:.3f} s)")


if __name__ == "__main__":
    main()
