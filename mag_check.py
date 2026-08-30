#!/usr/bin/python
"""
mag_check.py — diagnostyka mapowania osi magnetometru (MMC5983MA vs ISM330DHCX).

WERSJA 2 — stosuje kalibracje z calibration.json (tak jak locator_imu.py) oraz
kontroluje jakosc pomiaru:
  • |B| (modul pola) — przy czystym obrocie MUSI byc ~staly. Skacze => zla
    kalibracja hard/soft iron albo zaklocenia (metal, elektronika, kable).
  • poziom (z akcelerometru) — plytka musi lezec PLASKO; inaczej wynik bez sensu.

── TRYB 1: obrot ────────────────────────────────────────────────────────────
    python3 mag_check.py

Poloz plytke PLASKO na niemetalowej powierzchni (drewno) i obracaj powoli
o 360 st. W PRAWO, NIE przechylajac. Skrypt pilnuje poziomu i |B|.
Szukaj kolumny, w ktorej heading ROSNIE plynnie 0->90->180->270.

── TRYB 2: punkty kardynalne (rozstrzygajacy) ───────────────────────────────
    python3 mag_check.py --cardinal

Kompas w telefonie jako odniesienie. Wybierz DOWOLNA krawedz plytki jako
"przod", kieruj ja kolejno na N, E, S, W. Skrypt wskaze najlepsze mapowanie
i wypisze gotowa linijke do locator_imu.py.
"""

import os
import json
import math
import time
import argparse

import smbus2

ISM_ADDR       = 0x6b
MMC_ADDR       = 0x30
MMC_REG_XOUT0  = 0x00
MMC_REG_STATUS = 0x08
MMC_REG_CTRL0  = 0x09
MMC_NULL_FIELD = 131072
ACCEL_SCALE    = 0.061 / 1000.0
DECL           = 5.5                 # deklinacja (Krakow) — jak w locator_imu.py
CALIBRATION_FILE = "calibration.json"

bus = smbus2.SMBus(1)
CAL = None


def load_cal():
    global CAL
    if not os.path.exists(CALIBRATION_FILE):
        print(f"!! Brak {CALIBRATION_FILE} — pracuje na surowych danych (wyniki gorsze).")
        return
    with open(CALIBRATION_FILE) as f:
        CAL = json.load(f)
    print(f"Wczytano {CALIBRATION_FILE} (hard/soft iron beda zastosowane).")


def apply_mag_cal(mx, my, mz):
    """Ta sama korekta co w locator_imu.apply_cal() — pelna macierz, gdy jest."""
    if not CAL:
        return mx, my, mz
    m = CAL["mag"]
    cx = mx - m["hard_iron_x"]
    cy = my - m["hard_iron_y"]
    cz = mz - m["hard_iron_z"]
    M = m.get("soft_iron_matrix")
    if M:
        return (M[0][0]*cx + M[0][1]*cy + M[0][2]*cz,
                M[1][0]*cx + M[1][1]*cy + M[1][2]*cz,
                M[2][0]*cx + M[2][1]*cy + M[2][2]*cz)
    return (cx * m["soft_iron_scale_x"],
            cy * m["soft_iron_scale_y"],
            cz * m["soft_iron_scale_z"])


def s16(v):
    return v - 65536 if v > 32767 else v


def init():
    bus.write_byte_data(ISM_ADDR, 0x10, 0xA0)
    bus.write_byte_data(ISM_ADDR, 0x11, 0xA0)
    time.sleep(0.1)


def read_accel():
    d = bus.read_i2c_block_data(ISM_ADDR, 0x22, 12)
    return (s16(d[7] << 8 | d[6]) * ACCEL_SCALE,
            s16(d[9] << 8 | d[8]) * ACCEL_SCALE,
            s16(d[11] << 8 | d[10]) * ACCEL_SCALE)


def read_mag():
    bus.write_byte_data(MMC_ADDR, MMC_REG_CTRL0, 0x21)
    for _ in range(50):
        time.sleep(0.001)
        if bus.read_byte_data(MMC_ADDR, MMC_REG_STATUS) & 0x10:
            break
    else:
        return None
    d = bus.read_i2c_block_data(MMC_ADDR, MMC_REG_XOUT0, 7)
    mx = ((d[0] << 10) | (d[1] << 2) | ((d[6] >> 6) & 0x03)) - MMC_NULL_FIELD
    my = ((d[2] << 10) | (d[3] << 2) | ((d[6] >> 4) & 0x03)) - MMC_NULL_FIELD
    mz = ((d[4] << 10) | (d[5] << 2) | ((d[6] >> 2) & 0x03)) - MMC_NULL_FIELD
    if mx == -MMC_NULL_FIELD and my == -MMC_NULL_FIELD:
        return None                      # pusty odczyt
    return apply_mag_cal(float(mx), float(my), float(mz))


MAPPINGS = [
    ("A", "( mx,  my)", lambda x, y: ( x,  y)),
    ("B", "( mx, -my)", lambda x, y: ( x, -y)),
    ("C", "(-mx,  my)", lambda x, y: (-x,  y)),
    ("D", "(-mx, -my)", lambda x, y: (-x, -y)),
    ("E", "( my,  mx)", lambda x, y: ( y,  x)),
    ("F", "( my, -mx)", lambda x, y: ( y, -x)),
    ("G", "(-my,  mx)", lambda x, y: (-y,  x)),
    ("H", "(-my, -mx)", lambda x, y: (-y, -x)),
]


def heading_for(f, mx, my, mz, ax, ay, az):
    X, Y = f(mx, my)
    roll  = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    xh = X * math.cos(pitch) + mz * math.sin(pitch)
    yh = (X * math.sin(roll) * math.sin(pitch)
          + Y * math.cos(roll)
          - mz * math.sin(roll) * math.cos(pitch))
    return (math.degrees(math.atan2(-yh, xh)) + DECL) % 360.0


def circ_diff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def level_ok(ax, ay, az):
    """Plytka plasko: grawitacja prawie w calosci na osi Z."""
    tilt = math.degrees(math.atan2(math.sqrt(ax * ax + ay * ay), abs(az)))
    return tilt, tilt < 12.0


# ── TRYB 1 ───────────────────────────────────────────────────────────────────
def mode_rotate():
    print("\nPoloz plytke PLASKO (drewno, z dala od metalu) i obracaj powoli")
    print("o 360 st. W PRAWO. NIE przechylaj plytki.\n")
    print("Kontrola: 'poziom' ma byc OK, |B| ma byc ~STALE.")
    print("Szukaj kolumny, w ktorej heading rosnie plynnie 0->90->180->270.\n")
    print("     " + "".join(f"{n:>6}" for n, _, _ in MAPPINGS) + f"{'|B|':>8}{'przechyl':>10}")
    print("-" * 78)
    mags = []
    try:
        while True:
            m = read_mag()
            if m is None:
                time.sleep(0.1)
                continue
            mx, my, mz = m
            ax, ay, az = read_accel()
            tilt, ok = level_ok(ax, ay, az)
            B = math.sqrt(mx * mx + my * my + mz * mz)
            mags.append(B)
            hs = [heading_for(f, mx, my, mz, ax, ay, az) for _, _, f in MAPPINGS]
            flag = "" if ok else "  <-- PRZECHYLONA!"
            print("     " + "".join(f"{h:6.0f}" for h in hs)
                  + f"{B:8.0f}{tilt:9.0f}st{flag}", flush=True)
            time.sleep(0.3)
    except KeyboardInterrupt:
        if len(mags) > 5:
            lo, hi = min(mags), max(mags)
            print(f"\n|B|: min={lo:.0f}  max={hi:.0f}  wahanie={100*(hi-lo)/hi:.0f}%")
            if (hi - lo) / hi > 0.25:
                print("!! |B| waha sie >25% — kalibracja hard/soft iron jest zla")
                print("!! albo w poblizu jest metal. Wynik NIEWIARYGODNY.")
            else:
                print("|B| stabilne — pomiar wiarygodny.")


# ── TRYB 2 ───────────────────────────────────────────────────────────────────
def mode_cardinal():
    print("\nKompas w telefonie jako odniesienie. Plytka PLASKO, z dala od metalu.")
    print("Wybierz DOWOLNA krawedz plytki jako 'przod' i kieruj ja kolejno na:\n")
    targets = [("POLNOC (N)", 0.0), ("WSCHOD (E)", 90.0),
               ("POLUDNIE (S)", 180.0), ("ZACHOD (W)", 270.0)]
    samples, mags = [], []

    for name, true_h in targets:
        input(f"  Skieruj 'przod' na {name:12s} i nacisnij ENTER...")
        acc = []
        for _ in range(15):
            m = read_mag()
            if m is None:
                continue
            a = read_accel()
            acc.append((m[0], m[1], m[2], a[0], a[1], a[2]))
            time.sleep(0.05)
        if not acc:
            print("    !! brak odczytu magnetometru")
            continue
        avg = [sum(c[i] for c in acc) / len(acc) for i in range(6)]
        tilt, ok = level_ok(avg[3], avg[4], avg[5])
        B = math.sqrt(avg[0]**2 + avg[1]**2 + avg[2]**2)
        mags.append(B)
        warn = "" if ok else f"  !! PRZECHYL {tilt:.0f}st — powtorz plasko!"
        print(f"    zapisano   |B|={B:.0f}  przechyl={tilt:.0f}st{warn}")
        samples.append((true_h, avg))

    if len(samples) < 3:
        print("\nZa malo probek.")
        return

    if mags:
        lo, hi = min(mags), max(mags)
        if (hi - lo) / hi > 0.25:
            print(f"\n!! UWAGA: |B| waha sie o {100*(hi-lo)/hi:.0f}% miedzy pomiarami.")
            print("!! Kalibracja magnetometru jest zla lub sa zaklocenia magnetyczne.")
            print("!! Wynik ponizej moze byc niewiarygodny — najpierw popraw kalibracje.\n")

    print(f"\n{'mapowanie':<18}" + "".join(f"{t[0][:1]:>7}" for t in targets)
          + f"{'sr.blad':>10}")
    print("-" * 60)
    results = []
    for key, expr, f in MAPPINGS:
        hs, errs = [], []
        for true_h, s in samples:
            h = heading_for(f, *s)
            hs.append(h)
            errs.append(abs(circ_diff(h, true_h)))
        e = sum(errs) / len(errs)
        results.append((e, key, expr))
        print(f"{key + ': ' + expr:<18}" + "".join(f"{h:7.0f}" for h in hs) + f"{e:9.1f}st")

    results.sort()
    e, key, expr = results[0]
    print("\n" + "=" * 60)
    print(f"NAJLEPSZE MAPOWANIE: {key}: {expr}   (sredni blad {e:.1f} st.)")
    if e > 25:
        print("\n!! Blad >25 st. — najpierw popraw kalibracje magnetometru,")
        print("!! potem powtorz ten test.")
    else:
        print(f"\nW locator_imu.py, w read_mag_raw(), tuz przed 'return mx, my, mz':")
        print(f"    mx, my = {expr.replace('mx','mx').replace('my','my')}")
        print(f"    mz = -mz     # zgodnosc osi Z z ISM330DHCX")
    print("=" * 60)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--cardinal", action="store_true")
    a = ap.parse_args()
    load_cal()
    init()
    mode_cardinal() if a.cardinal else mode_rotate()
