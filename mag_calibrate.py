#!/usr/bin/python
"""
mag_calibrate.py — kalibracja magnetometru MMC5983MA (hard iron + soft iron).

WERSJA 2 — dopasowanie ELIPSOIDY metoda najmniejszych kwadratow (zamiast min/max).
Metoda min/max jest bardzo wrazliwa na pokrycie orientacji i pojedyncze odstajace
probki; dopasowanie elipsoidy wykorzystuje WSZYSTKIE punkty i daje duzo lepszy
wynik przy niepelnym pokryciu.

!! KLUCZOWE: kalibruj w DOKLADNIE tej konfiguracji, w ktorej urzadzenie bedzie
   uzywane — Pi, powerbank, kable, obudowa, modem: wszystko podlaczone.

UZYCIE:
    python3 mag_calibrate.py                # 60 s
    python3 mag_calibrate.py --seconds 90

Obracaj urzadzenie POWOLI we wszystkich kierunkach — osemki, obroty wokol kazdej
z trzech osi, do gory nogami. Wyobraz sobie, ze malujesz wnetrze kuli.

Jesli po kalibracji wahanie |B| nadal > 15%, uruchom najpierw:
    python3 mag_static_test.py
zeby sprawdzic, czy zaklocenia nie sa DYNAMICZNE (wtedy kalibracja nie pomoze).
"""

import os
import json
import math
import time
import shutil
import argparse

import numpy as np
import smbus2

MMC_ADDR       = 0x30
MMC_REG_XOUT0  = 0x00
MMC_REG_STATUS = 0x08
MMC_REG_CTRL0  = 0x09
MMC_NULL_FIELD = 131072
CALIBRATION_FILE = "calibration.json"

bus = smbus2.SMBus(1)


def read_mag_raw():
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
        return None
    return float(mx), float(my), float(mz)


def fit_ellipsoid(P):
    """
    Dopasowanie elipsoidy do chmury punktow (najmniejsze kwadraty).
    Zwraca (srodek, macierz_transformacji) — punkty po korekcie leza na sferze.
    Model: x'Ax + b'x + c = 0
    """
    x, y, z = P[:, 0], P[:, 1], P[:, 2]
    # macierz projektowa dla ogolnej kwadryki
    D = np.column_stack([x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z, 2*x, 2*y, 2*z,
                         np.ones_like(x)])
    # najmniejszy wektor wlasny D'D -> rozwiazanie |v|=1
    _, _, Vt = np.linalg.svd(D, full_matrices=False)
    v = Vt[-1]
    a, b, c, f, g, h, p, q, r, d = v

    A = np.array([[a, f, g],
                  [f, b, h],
                  [g, h, c]])
    vec = np.array([p, q, r])

    center = np.linalg.solve(-A, vec)                 # srodek elipsoidy = hard iron
    # przesun do srodka: wartosc stalej po translacji
    d_c = d + vec @ center
    # macierz znormalizowana
    Ac = A / (-d_c)
    evals, evecs = np.linalg.eigh(Ac)
    if np.any(evals <= 0):
        raise ValueError("dopasowanie nie jest elipsoida (za male pokrycie orientacji)")
    radii = 1.0 / np.sqrt(evals)                      # polosie
    r_mean = float(np.mean(radii))
    # transformacja: obrot -> skalowanie do sfery -> obrot z powrotem
    T = evecs @ np.diag(r_mean / radii) @ evecs.T
    return center, T, radii, r_mean


def spread(v):
    lo, hi = float(np.min(v)), float(np.max(v))
    return 100.0 * (hi - lo) / hi if hi else 0.0


def octants(P):
    c = (P.max(axis=0) + P.min(axis=0)) / 2
    return len({tuple(t) for t in (P > c)})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--out", default=CALIBRATION_FILE)
    a = ap.parse_args()

    print("=" * 68)
    print("KALIBRACJA MAGNETOMETRU (dopasowanie elipsoidy)")
    print("=" * 68)
    print("!! Urzadzenie w DOCELOWEJ konfiguracji: Pi + powerbank + kable + obudowa.")
    print("!! Z dala od zewnetrznego metalu (drewniany stol, nie przy grzejniku).\n")
    print(f"Przez {a.seconds:.0f} s obracaj POWOLI we wszystkich kierunkach:")
    print("  osemki w powietrzu, obroty wokol kazdej z 3 osi, do gory nogami.\n")
    input("ENTER, aby zaczac...")

    S = []
    t0 = time.time()
    last = 0.0
    while time.time() - t0 < a.seconds:
        m = read_mag_raw()
        if m:
            S.append(m)
        el = time.time() - t0
        if el - last >= 1.0:
            last = el
            P = np.array(S) if len(S) > 10 else None
            oc = octants(P) if P is not None else 0
            bar = "#" * int(30 * el / a.seconds)
            print(f"\r  [{bar:<30}] {el:4.0f}/{a.seconds:.0f}s  "
                  f"probek={len(S):4d}  oktanty={oc}/8", end="", flush=True)
    print()

    P = np.array(S)
    if len(P) < 100:
        print("\n!! Za malo probek.")
        return

    oc = octants(P)
    print(f"\nZebrano {len(P)} probek, pokryto {oc}/8 oktantow.")
    if oc < 6:
        print("!! Slabe pokrycie — obracaj w WIECEJ kierunkow i powtorz.")

    try:
        center, T, radii, r_mean = fit_ellipsoid(P)
    except (np.linalg.LinAlgError, ValueError) as e:
        print(f"\n!! Dopasowanie nie powiodlo sie: {e}")
        return

    B_before = np.linalg.norm(P, axis=1)
    Pc = (P - center) @ T.T
    B_after = np.linalg.norm(Pc, axis=1)

    # locator_imu.py obsluguje tylko skale DIAGONALNE -> sprawdz, czy wystarcza
    diag = np.diag(T)
    offdiag = T - np.diag(diag)
    offdiag_rel = float(np.abs(offdiag).max() / np.abs(diag).max())

    Pd = (P - center) * diag
    B_diag = np.linalg.norm(Pd, axis=1)

    print("\n" + "-" * 68)
    print(f"Hard iron (srodek):  x={center[0]:9.1f}  y={center[1]:9.1f}  z={center[2]:9.1f}")
    print(f"Polosie elipsoidy :  {radii[0]:.0f}, {radii[1]:.0f}, {radii[2]:.0f}"
          f"   (srednia {r_mean:.0f})")
    print(f"Soft iron (diag)  :  x={diag[0]:8.4f}  y={diag[1]:8.4f}  z={diag[2]:8.4f}")
    print("-" * 68)
    print(f"Wahanie |B| PRZED kalibracja        : {spread(B_before):6.1f}%")
    print(f"Wahanie |B| PO (model diagonalny)   : {spread(B_diag):6.1f}%   <- ten trafia do pliku")
    print(f"Wahanie |B| PO (pelna macierz)      : {spread(B_after):6.1f}%   (teoretyczne max)")
    print("-" * 68)

    if offdiag_rel > 0.15:
        print(f"\n!! Soft iron NIE jest osiowo-zgodny (pozadiagonalne {100*offdiag_rel:.0f}%).")
        print("!! Model diagonalny (jedyny obslugiwany przez locator_imu.py) nie odda")
        print("!! pelnej korekty. Roznica widoczna wyzej.")

    if spread(B_diag) > 20:
        print("\n!! Wahanie nadal duze. Uruchom:  python3 mag_static_test.py")
        print("!! Jesli |B| skacze przy BEZRUCHU -> zaklocenia DYNAMICZNE (prad")
        print("!! z modemu/CPU/powerbanku) i ZADNA kalibracja tego nie usunie.")
        if input("\nZapisac mimo to? [t/N]: ").strip().lower() != "t":
            print("Anulowano — plik nie zmieniony.")
            return

    cal = {}
    if os.path.exists(a.out):
        with open(a.out) as f:
            cal = json.load(f)
        shutil.copy(a.out, a.out + ".bak")
        print(f"\nKopia starego pliku: {a.out}.bak")

    cal["mag"] = {
        "hard_iron_x": float(center[0]),
        "hard_iron_y": float(center[1]),
        "hard_iron_z": float(center[2]),
        # model diagonalny — zachowany dla wstecznej zgodnosci
        "soft_iron_scale_x": float(diag[0]),
        "soft_iron_scale_y": float(diag[1]),
        "soft_iron_scale_z": float(diag[2]),
        # pelna macierz 3x3 — uzywana, gdy soft iron nie jest osiowo-zgodny.
        # locator_imu.py preferuje ja, jesli jest obecna.
        "soft_iron_matrix": [[float(T[i][j]) for j in range(3)] for i in range(3)],
    }
    with open(a.out, "w") as f:
        json.dump(cal, f, indent=2)

    print(f"Zapisano do {a.out}  (sekcje gyro/accel zachowane)")
    if offdiag_rel > 0.15:
        print("Zapisano tez pelna macierz soft_iron_matrix — locator_imu.py jej uzyje")
        print(f"i osiagnie {spread(B_after):.1f}% zamiast {spread(B_diag):.1f}%.")
    print("\nSprawdz:  python3 mag_check.py")


if __name__ == "__main__":
    main()
