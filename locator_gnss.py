#!/usr/bin/env python3

"""
https://github.com/ldab/u-blox_GNSS
https://github.com/HoverflyHampton/GPSSpy

Lokalizator GNSS z wykrywaniem jammingu/spoofingu (profil pieszy)
"""

import argparse
import math
import sys
import time
import threading
import logging
from datetime import datetime, timezone
from typing import Optional

from serial import Serial
from serial.serialutil import SerialException
from pyubx2 import UBXReader, UBX_PROTOCOL, UBXMessage

from position import Position

logger = logging.getLogger(__name__)

# ===== Pełna wiadomość NAV_SAT ==========================
# przy True skrypt wypisze dla kazdego satelity pelne dane
PRINT_FULL_NAV_SAT_DEFAULT = False
# ========================================================

# ---- Kolorowy output ----
COLOR_RESET = "\033[0m"
COLOR_GREEN = "\033[32m"
COLOR_YELLOW = "\033[33m"
COLOR_RED = "\033[31m"
COLOR_BOLD = "\033[1m"
ANSI_CLEAR_TO_EOL = "\033[K"


def color_state(text: str, state: str, use_color: bool) -> str:
    """Nadaje kolor tekstowi w zależności od stanu bezpieczeństwa sygnału."""
    if not use_color:
        return text
    if state == "OK":
        return f"{COLOR_GREEN}{text}{COLOR_RESET}"
    if state == "DEGRADED":
        return f"{COLOR_YELLOW}{text}{COLOR_RESET}"
    return f"{COLOR_RED}{COLOR_BOLD}{text}{COLOR_RESET}"  # jamming, spoofing na czerwono


def print_dashboard_line(line: str, one_line: bool):
    """
    Jeśli one_line=True: nadpisuje bieżącą linię w terminalu.
    W przeciwnym razie: zwykły wydruk z nową linią.
    """
    if one_line:
        print("\r" + line + ANSI_CLEAR_TO_EOL, end="", flush=True)
    else:
        print(line)


def find_ublox_port():
    """
    Skanuje dostępne porty szukając urządzenia u-blox
    u-blox Vendor ID (VID) 0x1546.
    """
    UBLOX_VID = 0x1546
    ports = serial.tools.list_ports.comports()
    
    for port in ports:
        if port.vid == UBLOX_VID:
            print(f"Found u-blox device: {port.device} ({port.description})")
            return port.device
            
    return None


# ----------------------------
# Utilities
# ----------------------------

def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def msg_full_dict(msg):
    """
    Zamienia surowe dane z pyubx2 na Python dict
    """
    if hasattr(msg, "to_dict"):
        try:
            return msg.to_dict()
        except Exception:
            pass
    if hasattr(msg, "__dict__"):
        return {k: v for k, v in msg.__dict__.items() if not k.startswith("_")}
    return {"repr": repr(msg)}


def _maybe_int(v):
    try:
        return int(v)
    except Exception:
        return None


def _maybe_float(v):
    try:
        return float(v)
    except Exception:
        return None


def haversine_m(lat1, lon1, lat2, lon2):
    """
    oblicza najkrótsza odległość między punktami na powierzchni kuli ziemskiej
    używając wzoru Haversine'a   
    """
    R = 6371000.0
    # konwersja na radiany
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    # wzór Haversine'a
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlmb / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


# ----------------------------
# Aktywne konstelacje (VALGET)
# ----------------------------

def print_enabled_constellations_valget(ser, ubr, timeout_s=2.0, layer=0):
    """
    Wyświetla flagi włączonych konstelacji za pomocą komendy UBX-CFG-VALGET.
    Layer=0 (RAM) pokazuje aktualnie aktywną konfigurację.
    """
    keys = [
        "CFG_SIGNAL_GPS_ENA",
        "CFG_SIGNAL_GAL_ENA",
        "CFG_SIGNAL_GLO_ENA",
        "CFG_SIGNAL_BDS_ENA",
    ]

    try:
        poll = UBXMessage.config_poll(layer, 0, keys)
    except Exception as e:
        print(f"Aktywne konstelacje: nie mozna pobrać konstelacji przez CFG-VALGET: {e})\n")
        return

    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    ser.write(poll.serialize())
    ser.flush()

    t0 = time.time()
    while time.time() - t0 < timeout_s:
        _, msg = ubr.read()
        if msg is None:
            continue
        if getattr(msg, "identity", "") != "CFG-VALGET":
            continue

        d = msg_full_dict(msg)

        def get_key(k):
            if k in d:
                return d[k]
            cfg = d.get("cfgData")
            if isinstance(cfg, dict) and k in cfg:
                return cfg[k]
            if isinstance(cfg, list):
                for item in cfg:
                    if isinstance(item, dict) and item.get("key") == k:
                        return item.get("value")
                    if isinstance(item, (list, tuple)) and len(item) == 2 and item[0] == k:
                        return item[1]
            return None

        def onoff(v):
            try:
                return "ENABLED" if int(v) else "disabled"
            except Exception:
                return "?"

        gps = get_key("CFG_SIGNAL_GPS_ENA")
        gal = get_key("CFG_SIGNAL_GAL_ENA")
        glo = get_key("CFG_SIGNAL_GLO_ENA")
        bds = get_key("CFG_SIGNAL_BDS_ENA")

        print("Aktywne konstelacje (z UBX-CFG-VALGET):")
        print(f"  GPS     : {onoff(gps)}")
        print(f"  Galileo : {onoff(gal)}")
        print(f"  GLONASS : {onoff(glo)}")
        print(f"  BeiDou  : {onoff(bds)}")
        print()
        return

    print("Aktywne konstelacje: (brak odpowiedzi CFG-VALGET)\n")


# ----------------------------
# NAV-SAT: podsumowanie widocznych/używanych satelitów
# ----------------------------

def parse_nav_sat_seen_used(msg):
    """Analizuje komunikat NAV-SAT pod kątem liczby satelitów."""
    d = msg_full_dict(msg)
    num_svs = _maybe_int(d.get("numSvs")) or 0

    GNSS_NAMES = {0: "GPS", 2: "GAL", 3: "BDS", 6: "GLO"}
    seen = {"GPS": 0, "GAL": 0, "GLO": 0, "BDS": 0}
    used = {"GPS": 0, "GAL": 0, "GLO": 0, "BDS": 0}

    for i in range(1, num_svs + 1):
        idx = f"{i:02d}"
        gnss_id = _maybe_int(d.get(f"gnssId_{idx}"))
        if gnss_id is None:
            continue
        g = GNSS_NAMES.get(gnss_id)
        if g is None:
            continue
        seen[g] += 1
        if bool(d.get(f"svUsed_{idx}", 0)):
            used[g] += 1

    return seen, used


def nav_sat_oneline(seen, used):
    """Formatuje dane o satelitach do jednej linii."""
    return (
        "Konstelacje (widziane/uzyte):"
        f"GPS {seen['GPS']}/{used['GPS']}  "
        f"GAL {seen['GAL']}/{used['GAL']}  "
        f"GLO {seen['GLO']}/{used['GLO']}  "
        f"BDS {seen['BDS']}/{used['BDS']}"
    )


# ----------------------------
# NAV-PVT: uproszczony wydruk pozycji i czasu
# ----------------------------

def pvt_extract_from_dict(d):
    """Wyciąga kluczowe parametry nawigacyjne z ramki PVT."""
    fix_type = _maybe_int(d.get("fixType"))
    num_sv = _maybe_int(d.get("numSV"))
    h_acc = _maybe_float(d.get("hAcc"))
    lat = _maybe_float(d.get("lat"))
    lon = _maybe_float(d.get("lon"))
    flags = d.get("flags")

    gnss_fix_ok = None
    if "gnssFixOK" in d:
        gnss_fix_ok = bool(d.get("gnssFixOK"))
    else:
        fi = _maybe_int(flags)
        if fi is not None:
            gnss_fix_ok = bool(fi & 0x01)

    if lat is not None and abs(lat) > 180:
        lat /= 1e7
    if lon is not None and abs(lon) > 180:
        lon /= 1e7
    if h_acc is not None and h_acc > 1000:
        h_acc /= 1000.0

    return {
        "fixType": fix_type,
        "gnssFixOK": gnss_fix_ok,
        "numSV": num_sv,
        "lat": lat,
        "lon": lon,
        "hAcc_m": h_acc,
    }


def print_nav_pvt_summary(p):
    """Wyświetla podsumowanie UBX-NAV-PVT."""
    FIX_TYPE_NAMES = {0: "brak fixa", 1: "tylko IMU", 2: "2D", 3: "3D", 4: "GNSS+DR", 5: "tylko czas"}
    ft = p.get("fixType")
    ft_name = FIX_TYPE_NAMES.get(ft, "nieznany")

    print("UBX-NAV-PVT:")
    print(f"  fixType: {ft} ({ft_name})" if ft is not None else "  fixType: ?")
    print(f"  gnssFixOK: {p.get('gnssFixOK')}" if p.get("gnssFixOK") is not None else "  gnssFixOK: ?")
    print(f"  numSV: {p.get('numSV')}" if p.get("numSV") is not None else "  numSV: ?")
    print(f"  lat: {p.get('lat'):.6f}" if p.get("lat") is not None else "  lat: ?")
    print(f"  lon: {p.get('lon'):.6f}" if p.get("lon") is not None else "  lon: ?")
    print(f"  hAcc: {p.get('hAcc_m'):.2f} m" if p.get("hAcc_m") is not None else "  hAcc: ?")


# ----------------------------
# NAV-STATUS (krótkie podsumowanie)
# ----------------------------

def print_nav_status_summary(msg):
    """Wyświetla status odbiornika."""
    d = msg_full_dict(msg)
    gps_fix = d.get("gpsFix")
    flags = d.get("flags")
    fix_stat = d.get("fixStat")
    flags2 = d.get("flags2")
    ttff = d.get("ttff")
    msss = d.get("msss")

    print("UBX-NAV-STATUS:")
    if gps_fix is not None:
        print(f"  gpsFix: {gps_fix}")
    if flags is not None:
        print(f"  flags: {flags}")
    if fix_stat is not None:
        print(f"  fixStat: {fix_stat}")
    if flags2 is not None:
        print(f"  flags2: {flags2}")
    if ttff is not None:
        print(f"  ttff: {ttff}")
    if msss is not None:
        print(f"  msss: {msss}")


# ----------------------------
# NAV-SIG (krótkie podsumowanie sygnałów)
# ----------------------------

def nav_sig_summary(msg):
    """Wyświetla średnie poziomy sygnałów C/N0 dla konstelacji."""
    d = msg_full_dict(msg)
    num_sigs = _maybe_int(d.get("numSigs")) or 0

    GNSS = {0: "GPS", 2: "GAL", 3: "BDS", 6: "GLO", 1: "SBAS", 5: "QZSS"}
    stats = {}  # nazwa -> [liczba, suma_cno]

    for i in range(1, num_sigs + 1):
        idx = f"{i:02d}"
        gnss_id = _maybe_int(d.get(f"gnssId_{idx}"))
        cno = _maybe_int(d.get(f"cno_{idx}"))
        if gnss_id is None:
            continue
        name = GNSS.get(gnss_id, f"GNSS{gnss_id}")
        stats.setdefault(name, [0, 0])
        stats[name][0] += 1
        if cno is not None:
            stats[name][1] += cno

    parts = []
    for key in ("GPS", "GAL", "GLO", "BDS"):
        if key in stats and stats[key][0] > 0:
            cnt, ssum = stats[key]
            avg = ssum / cnt if cnt else 0
            parts.append(f"{key}:{cnt} avgC/N0={avg:.1f}")

    return "NAV-SIG: " + "  ".join(parts) if parts else "NAV-SIG: (brak sygnałów)"


# ----------------------------
# MON/SEC (diagnostyka sprzętowa i bezpieczeństwo)
# ----------------------------

def _pick_fields(d, include_substrings, exclude_substrings=()):
    inc = tuple(s.lower() for s in include_substrings)
    exc = tuple(s.lower() for s in exclude_substrings)
    out = {}
    for k, v in d.items():
        kl = str(k).lower()
        if any(s in kl for s in inc) and not any(s in kl for s in exc):
            out[k] = v
    return out


def _pretty_kv(d):
    return ", ".join(f"{k}={v}" for k, v in d.items())


def print_mon_hw_like_before(msg):
    """Wyświetla stan sprzętowy (AGC, jamming)."""
    d = msg_full_dict(msg)
    agc_fields = _pick_fields(d, include_substrings=("agc", "agccnt"))
    jam_fields = _pick_fields(d, include_substrings=("jam", "jamm", "cw"))
    status_fields = _pick_fields(d, include_substrings=("status", "flags", "aStatus", "jammState", "jamState"))

    print("UBX-MON-HW:")
    if agc_fields:
        print(f"  AGC: { _pretty_kv(agc_fields) }")
    else:
        print("   AGC: (brak pól AGC)")

    merged = {}
    merged.update(jam_fields)
    merged.update({k: v for k, v in status_fields.items() if k not in merged})
    if merged:
        print(f"  status/jam: { _pretty_kv(merged) }")


def print_mon_rf_like_before(msg):
    d = msg_full_dict(msg)

    jam_fields = _pick_fields(d, include_substrings=("jam", "cw", "jamm"))
    aux_fields = _pick_fields(d, include_substrings=("noise", "agc", "ant", "rf"))

    jam_ind = None
    for k in ("jamInd", "jamInd_01", "jamInd_1", "cwJamInd", "cwJammingIndicator"):
        if k in d:
            jam_ind = d.get(k)
            break

    header = "UBX-MON-RF"
    if jam_ind is not None:
        print(f"{header}: Wskaznik jamInd = {jam_ind}")
    else:
        print(f"{header}: (no explicit jamInd field found; showing detected jam-related fields)")

    if jam_fields:
        print(f"  jam fields: { _pretty_kv(jam_fields) }")

    if aux_fields:
        if set(aux_fields.keys()) - set(jam_fields.keys()):
            extra = {k: v for k, v in aux_fields.items() if k not in jam_fields}
            print(f"  aux fields: { _pretty_kv(extra) }")


def _consolidated_jam_flag(fields):
    for k, v in fields.items():
        kl = str(k).lower()
        if "jam" not in kl:
            continue
        if v is None:
            continue
        try:
            if int(v) != 0:
                return True
        except Exception:
            if bool(v):
                return True
    return False


def print_sec_sig_like_before(msg):
    """Wyświetla status bezpieczeństwa sygnału z wiadomości SEC-SIG."""
    d = msg_full_dict(msg)
    jam_fields = _pick_fields(d, include_substrings=("jam", "jamm", "cw"))
    any_jam = _consolidated_jam_flag(jam_fields) if jam_fields else False

    print("UBX-SEC-SIG:")
    print(f"  consolidated any_jam = {any_jam}")
    if jam_fields:
        print(f"  jam detectors: { _pretty_kv(jam_fields) }")
    else:
        print("  jam detectors: (no jam-related fields found in decoded message)")


# ----------------------------
# Switch logic (dla profilu pieszego)
# ----------------------------

def decide_gnss_switch(latest, prev, cfg):
    pvt = latest.get("pvt") or {}
    sec = latest.get("sec") or {}
    rf = latest.get("mon_rf") or {}
    hw = latest.get("mon_hw") or {}
    used = latest.get("sat_used") or {}

    fixType = _maybe_int(pvt.get("fixType"))
    numSV = _maybe_int(pvt.get("numSV"))
    hAcc = _maybe_float(pvt.get("hAcc_m"))
    lat = pvt.get("lat")
    lon = pvt.get("lon")

    jammingState = _maybe_int(sec.get("jammingState"))
    spoofingState = _maybe_int(sec.get("spoofingState"))

    jamInd = None
    for k in ("jamInd_01", "jamInd"):
        if rf.get(k) is not None:
            jamInd = _maybe_int(rf.get(k))
            break
    if jamInd is None and hw.get("jamInd") is not None:
        jamInd = _maybe_int(hw.get("jamInd"))

    # Ruch na podstawie różnic pozycji
    speed = None
    accel = None
    jump = None
    now = time.time()
    
    # Ocena wiarygodności sygnału GNSS
    if lat is not None and lon is not None and prev.get("lat") is not None and prev.get("lon") is not None:
        dt = max(1e-6, now - prev["t"])
        # oblicza dystans między poprzednią a obecną pozycją (w metrach)
        jump = haversine_m(prev["lat"], prev["lon"], lat, lon)
        # oblicz prędkość
        speed = jump / dt
        # zmiana prędkości w czasie (przyspieszenie)
        if prev.get("speed") is not None:
            accel = (speed - prev["speed"]) / dt
        prev["lat"], prev["lon"], prev["speed"], prev["t"] = lat, lon, speed, now
    else:
        if lat is not None and lon is not None:
            prev["lat"], prev["lon"], prev["speed"], prev["t"] = lat, lon, None, now

    # zapobiega fałszywym alarmom przy braku ruchu
    # GNSS ma tendencje do dryfu, gdy stoimy w miejscu
    # jak v jest bardzo mała, ustaw flagę STATIONARY
    # system wie, że drobne skoki pozycji wynikają z szumu a nie z ataku
    if speed is not None:
        if speed < cfg["STATIONARY_SPEED"]:
            prev["stationary_run"] = prev.get("stationary_run", 0) + 1
        else:
            prev["stationary_run"] = 0
    stationary = prev.get("stationary_run", 0) >= cfg["STATIONARY_N"]

    # ---- Detekcja jammingu ----
    # 1. sprawdź flage jamming zwracaną przez u-blox
    jam_detector = (jammingState is not None and jammingState >= 2)
    # 2. analiza widma. jeżeli poziom szumu w paśmie przekracza próg
    # oznacza to fizyczną obecność zakłóceń
    jam_rf = (jamInd is not None and jamInd >= cfg["JAM_JAMIND_TH"])
    # 3. skutki
    jam_impact = (
         # czy fix faktycznie spadł poniżej 3
        (fixType is not None and fixType < 3) or
        # czy liczba satelitów jest bardzo mała
        (numSV is not None and numSV <= cfg["JAM_NUMSV_TH"]) or
        # czy błąd pozycji stał się bardzo duży
        (hAcc is not None and hAcc >= cfg["JAM_HACC_TH"])
    )

    if jam_detector and jam_rf and jam_impact:
        prev["jam_bad"] = prev.get("jam_bad", 0) + 1
    else:
        prev["jam_bad"] = 0

    # ---- Detekcja SPOOFINGU ----
    # WARSTWA SPRZĘTOWA
    # inkrementacja wskaźnika podejrzenia spoofingu po sygnale z modułu
    spf_suspected = (spoofingState is not None and spoofingState >= 2)
    # inkrementacja wskaźnika spoofingu po sygnale z modułu
    spf_strong = (spoofingState is not None and spoofingState == 3)

    physics_bad = False
    phys_reasons = []
    
    # WARSTWA ANALITYCZNA
    jump_th = cfg["JUMP_MAX"] * (cfg["STATIONARY_JUMP_MULT"] if stationary else 1.0)
    # sprawdź, czy ruch raportowany przez GNSS jest zgodny z modelem pieszego
    # gdy prędkośc nienaturalnie wysoka, ustaw flagę ostrzegawczą
    # czy prędkość nie przekracza 10 m/s (36 km/h)
    if speed is not None and speed > cfg["SPEED_MAX"]:
        physics_bad = True
        phys_reasons.append(f"speed>{cfg['SPEED_MAX']}m/s")
    # czy pozycja nie zmieniła się nagle o więcej niż 40m
    if jump is not None and jump > jump_th:
        physics_bad = True
        phys_reasons.append(f"jump>{jump_th:.0f}m")
    # czy zmiana prędkości nie jest zbyt gwałtowna dla człowieka
    # np. 0 do 50 km/h w sekundę
    if accel is not None and abs(accel) > cfg["ACC_MAX"]:
        physics_bad = True
        phys_reasons.append(f"acc>{cfg['ACC_MAX']}m/s2")

    # wykrywanie niezgodności (mismatch)
    mismatch = False
    mismatch_reasons = []

    if spf_suspected:
        # zbyt dobra dokładność: jeśli hAcc (błąd) jest bardzo mały, ale odbiornik widzi tylko kilka satelitów (numSV < 6) 
        # jest to sygnał alarmowy - naturalny sygnał przy małej liczbie satelitów zawsze generuje większy błąd
        if hAcc is not None and hAcc <= cfg["HACC_TOO_GOOD"] and numSV is not None and numSV <= cfg["NUMSV_LOW"]:
            mismatch = True
            mismatch_reasons.append("hAcc_good_but_fewSV")

        if jamInd is not None and jamInd < cfg["JAMIND_LOW"]:
            # zniknięcie konstelacji Galileo jest podejrzane
            # jeśli nagle ich liczba spada do 0 a działają inne
            # tzn. że spoofer zakłóca inne konstelacje a emuluje tylko GPS
            gal_used = used.get("GAL")
            if gal_used is not None:
                if prev.get("ever_gal_used"):
                    if gal_used == 0:
                        prev["gal_zero_run"] = prev.get("gal_zero_run", 0) + 1
                    else:
                        prev["gal_zero_run"] = 0
                if gal_used > 0:
                    prev["ever_gal_used"] = True
                if prev.get("gal_zero_run", 0) >= cfg["GAL_ZERO_N"]:
                    mismatch = True
                    mismatch_reasons.append("gal_used_dropped_to_0")

    # sygnał podejrzenia spoofingu w module u-blox (warstwa sprzętowa) 
    # oraz jeden dodatkowy czynnik charakteryzujący spoofing (warstwa analityczna): 
    # - naruszenie praw fizyki ruchu
    # - nielogiczne dane
    # zwiększ wskaźnik podejrzenia spoofingu (spf_suspected)
    if spf_suspected and (mismatch or physics_bad):
        prev["spf_suspected"] = prev.get("spf_suspected", 0) + 1
    else:
        prev["spf_suspected"] = 0

    # sygnał spoofingu z modułu u-blox oraz
    # złamanie praw fizyki
    # wtedy zwiększ licznik potwierdzonego ataku (spf_bad)
    if spf_strong and physics_bad:
        prev["spf_bad"] = prev.get("spf_bad", 0) + 1
    else:
        prev["spf_bad"] = 0

    # ---- Klasyfikacja sygnałów ----
    # gdy licznik wykrytego spoofingu przekroczył wartość progową
    # zaraportuj spoofing
    if prev["spf_bad"] >= cfg["SPF_BAD_N"]:
        prev["good"] = 0
        reason = f"spfState={spoofingState} " + " ".join(phys_reasons)
        return "SPOOFED", reason.strip()
    
    # gdy licznik wykrytego jammingu przekroczył wartość progową
    # zaraportuj jamming
    if prev["jam_bad"] >= cfg["JAM_N"]:
        prev["good"] = 0
        reason = f"jamState={jammingState} jamInd={jamInd} hAcc={hAcc} numSV={numSV}"
        return "JAMMED", reason

    # gdy licznik podejrzeń spoofingu przekroczył wartość progową
    # zaraportuj podejrzenie spoofingu
    if prev["spf_suspected"] >= cfg["spf_suspected_N"]:
        prev["good"] = 0
        rr = []
        if spoofingState is not None:
            rr.append(f"spfState={spoofingState}")
        if jamInd is not None:
            rr.append(f"jamInd={jamInd}")
        rr += mismatch_reasons
        return "SPOOF_SUSPECT", " ".join(rr)

    # zaraportuj degradację sygnału
    degraded = (
        (fixType is not None and fixType < 3) or
        (hAcc is not None and hAcc >= cfg["DEG_HACC_TH"]) or
        (numSV is not None and numSV <= cfg["DEG_NUMSV_TH"])
    )
    if degraded:
        prev["good"] = 0
        return "DEGRADED", f"fixType={fixType} hAcc={hAcc} numSV={numSV}"
    
    # jeśli przez pewien czas stan jest OK, skrypt zeruje liczniki błędów.
    prev["good"] = prev.get("good", 0) + 1
    if prev["good"] >= cfg["CLEAR_M"]:
        prev["jam_bad"] = 0
        prev["spf_suspected"] = 0
        prev["spf_bad"] = 0
        prev["gal_zero_run"] = 0

    return "OK", f"jamInd={jamInd} hAcc={hAcc} numSV={numSV}"


# --------------------------------------------------
# Ekstraktuj z UBX parametry które potrzebuje skrypt
# --------------------------------------------------

def extract_sec_sig(d):
    return {
        "jammingState": _maybe_int(d.get("jammingState")),
        "spoofingState": _maybe_int(d.get("spoofingState")),
        "jamDetEnabled": _maybe_int(d.get("jamDetEnabled")),
        "spfDetEnabled": _maybe_int(d.get("spfDetEnabled")),
    }


def extract_mon_rf(d):
    return {
        "jamInd": _maybe_int(d.get("jamInd")),
        "jamInd_01": _maybe_int(d.get("jamInd_01")),
        "jamInd_02": _maybe_int(d.get("jamInd_02")),
    }

def extract_mon_hw(d):
    return {
        "jamInd": _maybe_int(d.get("jamInd")),
        "jammingState": _maybe_int(d.get("jammingState")),
        "agcCnt": _maybe_int(d.get("agcCnt")),
    }


# ----------------------------
# Main
# ----------------------------

def main():
    ap = argparse.ArgumentParser(description="GNSS monitor + jamming/spoofing switch (on-foot).")
    ap.add_argument("-p", "--port", default="/dev/ttyACM1", help="Serial port (default: /dev/ttyACM1)")
    ap.add_argument("-b", "--baud", type=int, default=38400, help="Baud rate (default: 38400)")
    ap.add_argument("-n", "--num", type=int, default=0, help="Stop after N NAV-PVT epochs (0 = run forever)")

    ap.add_argument("--no-color", action="store_true", help="Disable colored status output.")

    ap.add_argument("--speed-max", type=float, default=10.0, help="Spoof physics max speed (m/s) (default 10)")
    ap.add_argument("--jump-max", type=float, default=40.0, help="Spoof physics max 1-step jump (m) (default 40)")
    ap.add_argument("--acc-max", type=float, default=6.0, help="Spoof physics max acceleration (m/s^2) (default 6)")
    ap.add_argument("--jamind-th", type=int, default=50, help="Jamming jamInd threshold (default 50)")
    ap.add_argument("--jam-n", type=int, default=3, help="Consecutive seconds to declare JAMMED (default 3)")
    ap.add_argument("--spf-sus-n", type=int, default=5, help="Consecutive seconds to declare SPOOF_SUSPECT (default 5)")
    ap.add_argument("--spf-bad-n", type=int, default=3, help="Consecutive seconds to declare SPOOFED (default 3)")
    ap.add_argument("--clear-m", type=int, default=10, help="Consecutive good seconds to clear counters (default 10)")

    ap.add_argument("--stationary-speed", type=float, default=0.5, help="Below this speed treat as stationary (m/s) (default 0.5)")
    ap.add_argument("--stationary-n", type=int, default=5, help="Seconds below stationary-speed to consider stationary (default 5)")
    ap.add_argument("--stationary-jump-mult", type=float, default=3.0, help="Multiply jump-max when stationary (default 3.0)")

    args = ap.parse_args()

    use_color = (not args.no_color) and sys.stdout.isatty()

    cfg = {
        # progi jammingu
        "JAM_JAMIND_TH": int(args.jamind_th),
        "JAM_HACC_TH": 15.0,
        "JAM_NUMSV_TH": 5,
        "JAM_N": int(args.jam_n),

        # progi spoofingu
        "SPEED_MAX": float(args.speed_max),
        "JUMP_MAX": float(args.jump_max),
        "ACC_MAX": float(args.acc_max),
        "spf_suspected_N": int(args.spf_sus_n),
        "SPF_BAD_N": int(args.spf_bad_n),

        # mismatch
        "HACC_TOO_GOOD": 3.0,
        "NUMSV_LOW": 6,
        "JAMIND_LOW": 30,
        "GAL_ZERO_N": 5,

        # progi degradacji sygnału
        "DEG_HACC_TH": 10.0,
        "DEG_NUMSV_TH": 6,

        "CLEAR_M": int(args.clear_m),

        # czujnik się nie porusza
        "STATIONARY_SPEED": float(args.stationary_speed),
        "STATIONARY_N": int(args.stationary_n),
        "STATIONARY_JUMP_MULT": float(args.stationary_jump_mult),
    }

    try:
        ser = Serial(args.port, args.baud, timeout=1)
    except SerialException as e:
        print(f"Failed to open {args.port} @ {args.baud}: {e}", file=sys.stderr)
        sys.exit(1)

    ubr = UBXReader(ser, protfilter=UBX_PROTOCOL)

    wanted = {"NAV-PVT", "NAV-SAT", "NAV-SIG", "NAV-STATUS", "MON-RF", "MON-HW", "SEC-SIG"}

    latest = {
        "pvt": None,
        "sec": None,
        "mon_rf": None,
        "mon_hw": None,
        "sat_seen": None,
        "sat_used": None,
    }

    prev = {
        "lat": None,
        "lon": None,
        "t": time.time(),
        "speed": None,
        "stationary_run": 0,

        "jam_bad": 0,
        "spf_suspected": 0,
        "spf_bad": 0,
        "good": 0,

        "ever_gal_used": False,
        "gal_zero_run": 0,
    }

    epochs = 0

    try:
        while True:
            _, msg = ubr.read()
            if msg is None:
                continue

            ident = getattr(msg, "identity", "")
            if ident not in wanted:
                continue

            d = msg_full_dict(msg)

            if ident == "SEC-SIG":
                latest["sec"] = extract_sec_sig(d)

            elif ident == "MON-HW":
                latest["mon_hw"] = extract_mon_hw(d)

            elif ident == "MON-RF":
                latest["mon_rf"] = extract_mon_rf(d)

            elif ident == "NAV-SAT":
                seen, used = parse_nav_sat_seen_used(msg)
                latest["sat_seen"] = seen
                latest["sat_used"] = used

            elif ident == "NAV-PVT":
                pvt = pvt_extract_from_dict(d)
                latest["pvt"] = pvt

                gnss_state, reason = decide_gnss_switch(latest, prev, cfg)

                jam_state = None if latest["sec"] is None else latest["sec"].get("jammingState")
                spf_state = None if latest["sec"] is None else latest["sec"].get("spoofingState")

                jam_ind = None
                if latest["mon_rf"] is not None:
                    jam_ind = latest["mon_rf"].get("jamInd_01") if latest["mon_rf"].get("jamInd_01") is not None else latest["mon_rf"].get("jamInd")
                if jam_ind is None and latest["mon_hw"] is not None:
                    jam_ind = latest["mon_hw"].get("jamInd")

                used = latest.get("sat_used") or {}
                hacc = pvt.get("hAcc_m")

                state_txt = color_state(gnss_state, gnss_state, use_color)
                lat_s = f"{pvt.get('lat'):.6f}" if pvt.get('lat') is not None else "?"
                lon_s = f"{pvt.get('lon'):.6f}" if pvt.get('lon') is not None else "?"
                hacc_s = f"{hacc:.2f}m" if hacc is not None else "?"
                dash = (
                    f"GNSS {state_txt} | "
                    f"USED G={used.get('GPS')} E={used.get('GAL')} R={used.get('GLO')} C={used.get('BDS')} | "
                    f"lat={lat_s} lon={lon_s} hAcc={hacc_s}"
                )
                print_dashboard_line(dash, one_line=True)

                epochs += 1
                if args.num and epochs >= args.num:
                    break

    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.close()
        except Exception:
            pass
        print()  # dodaj znak nowej linii


# ═══════════════════════════════════════════════════════════════════════════════
# GNSSLocator — Wrapper gotowy do uzycia z Filtrem Kalmana
#
# Uruchamia pętlę czytelnika UBX w osobnym wątku.
# Udostępnia metodę get_position() dla Filtrów Kalmana.
#
# Mapowanie stanów bezpieczeństwa -> Pozycja:
#   OK             → Position(lat, lon, accuracy=hAcc_m,   source="gnss")
#   DEGRADED       → Position(lat, lon, accuracy=hAcc_m*3, source="gnss_degraded")
#   JAMMED         → None (Filtr polega na innych zrodlach)
#   SPOOF_SUSPECT  → None
#   SPOOFED        → None
# ═══════════════════════════════════════════════════════════════════════════════

# Ile musi minąc zanim get_position() zwróci None
_GNSS_STALE_S = 5.0

# ile przemnożyć accuracy w stanie DEGRADED
_DEGRADED_ACCURACY_MULT = 3.0


class GNSSLocator:

    _DEFAULT_CFG = {
        "JAM_JAMIND_TH": 50,
        "JAM_HACC_TH":   15.0,
        "JAM_NUMSV_TH":  5,
        "JAM_N":         3,
        "SPEED_MAX":     10.0,
        "JUMP_MAX":      40.0,
        "ACC_MAX":       6.0,
        "spf_suspected_N":     5,
        "SPF_BAD_N":     3,
        "HACC_TOO_GOOD": 3.0,
        "NUMSV_LOW":     6,
        "JAMIND_LOW":    30,
        "GAL_ZERO_N":    5,
        "DEG_HACC_TH":   10.0,
        "DEG_NUMSV_TH":  6,
        "CLEAR_M":       10,
        "STATIONARY_SPEED":      0.5,
        "STATIONARY_N":          5,
        "STATIONARY_JUMP_MULT":  3.0,
    }

    def __init__(self, port: str = "/dev/ttyACM0", baud: int = 38400):
        self._port  = port
        self._baud  = baud
        self._cfg   = dict(self._DEFAULT_CFG)

        self._position: Optional[Position] = None
        self._gnss_state: str = "UNKNOWN"
        self._lock    = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._running = False

    def start(self) -> None:
        self._running = True
        self._thread  = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        logger.info(f"GNSSLocator: started on {self._port} @ {self._baud}")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=5.0)
        logger.info("GNSSLocator: stopped")

    def get_position(self) -> Optional[Position]:
        """
        Zwraca ostatnią zaufaną pozycję lub None, jeśli:
        - brak fixa
        - fix jest nieruchomy (> _GNSS_STALE_S sekund)
        - Stan jest JAMMED / SPOOF_SUSPECT / SPOOFED
        """
        with self._lock:
            if self._position is None:
                return None
            if time.time() - self._position.timestamp > _GNSS_STALE_S:
                return None
            return self._position

    @property
    def gnss_state(self) -> str:
        """Zwraca aktualny stan bezpieczeństwa (OK / DEGRADED / itd.)."""
        with self._lock:
            return self._gnss_state


    def _run(self) -> None:
        while self._running:
            #current_port = find_ublox_port()
            #if not current_port:
            #    print("Nie znaleziono urządzenia u-blox. Ponowna próba za 5s...")
            #    time.sleep(5)
            #    continue
            
            try:
                ser = Serial(self._port, self._baud, timeout=1)
                #ser = Serial(current_port, self._baud, timeout=1)
                ubr = UBXReader(ser, protfilter=UBX_PROTOCOL)
                logger.info(f"GNSSLocator: serial port {self._port} opened")

                print_enabled_constellations_valget(ser, ubr, timeout_s=2.0, layer=0)

                wanted = {"NAV-PVT", "NAV-SAT", "NAV-SIG", "NAV-STATUS",
                          "MON-RF", "MON-HW", "SEC-SIG"}

                latest = {
                    "pvt": None, "sec": None, "mon_rf": None, "mon_hw": None,
                    "sat_seen": None, "sat_used": None,
                    "last_navsig": None, "last_navsig_ts": 0.0,
                }
                prev = {
                    "lat": None, "lon": None, "t": time.time(), "speed": None,
                    "stationary_run": 0, "jam_bad": 0, "spf_suspected": 0,
                    "spf_bad": 0, "good": 0, "ever_gal_used": False,
                    "gal_zero_run": 0,
                }

                use_color = sys.stdout.isatty()

                while self._running:
                    _, msg = ubr.read()
                    if msg is None:
                        continue

                    ident = getattr(msg, "identity", "")
                    if ident not in wanted:
                        continue

                    d = msg_full_dict(msg)

                    if ident == "SEC-SIG":
                        latest["sec"] = extract_sec_sig(d)
                        print_sec_sig_like_before(msg)

                    elif ident == "MON-HW":
                        latest["mon_hw"] = extract_mon_hw(d)
                        print_mon_hw_like_before(msg)

                    elif ident == "MON-RF":
                        latest["mon_rf"] = extract_mon_rf(d)
                        print_mon_rf_like_before(msg)

                    elif ident == "NAV-STATUS":
                        print_nav_status_summary(msg)

                    elif ident == "NAV-SIG":
                        line = nav_sig_summary(msg)
                        now_m = time.time()
                        if line != latest["last_navsig"] or (now_m - latest["last_navsig_ts"]) > 5.0:
                            print(line)
                            latest["last_navsig"] = line
                            latest["last_navsig_ts"] = now_m

                    elif ident == "NAV-SAT":
                        seen, used_sat = parse_nav_sat_seen_used(msg)
                        latest["sat_seen"] = seen
                        latest["sat_used"] = used_sat
                        print(nav_sat_oneline(seen, used_sat))

                    elif ident == "NAV-PVT":
                        pvt = pvt_extract_from_dict(d)
                        latest["pvt"] = pvt
                        print_nav_pvt_summary(pvt)

                        gnss_state, reason = decide_gnss_switch(latest, prev, self._cfg)

                        jam_state = None if latest["sec"] is None else latest["sec"].get("jammingState")
                        spf_state = None if latest["sec"] is None else latest["sec"].get("spoofingState")
                        jam_ind = None
                        if latest["mon_rf"] is not None:
                            jam_ind = latest["mon_rf"].get("jamInd_01") or latest["mon_rf"].get("jamInd")
                        if jam_ind is None and latest["mon_hw"] is not None:
                            jam_ind = latest["mon_hw"].get("jamInd")
                        used = latest.get("sat_used") or {}
                        state_field = color_state(f"state={gnss_state}", gnss_state, use_color)
                        print(
                            "GNSS_SWITCH: "
                            f"{state_field} "
                            f"secJam={jam_state} secSpf={spf_state} jamInd={jam_ind} "
                            f"usedGPS={used.get('GPS')} usedGAL={used.get('GAL')} "
                            f"usedGLO={used.get('GLO')} usedBDS={used.get('BDS')} "
                            f"reason={reason}"
                        )

                        self._update_position(pvt, gnss_state)

                ser.close()

            except SerialException as e:
                #print(f"Brak połączenia z portem {current_port}: {e}")
                logger.warning(f"Błąd portu szeregowego: {e} — ponowna proba za 3s")
                time.sleep(3)

    def _update_position(self, pvt: dict, gnss_state: str) -> None:
        """
        Konwertuje dane PVT na obiekt Position, uwzględniając stan bezpieczeństwa
        Convert PVT + gnss_state into a Position for the Kalman filter.
        Jammed/spoofed states produce None — filter ignores GNSS entirely.
        """
        lat   = pvt.get("lat")
        lon   = pvt.get("lon")
        h_acc = pvt.get("hAcc_m")

        with self._lock:
            self._gnss_state = gnss_state

            # brak fixa
            if lat is None or lon is None or h_acc is None:
                self._position = None
                return

            # sygnał intencjonalnie zakłócony, filtr ma go zignorować
            if gnss_state in ("JAMMED", "SPOOF_SUSPECT", "SPOOFED"):
                self._position = None
                return

            # sygnał zdegradowany — accuracy jest zwiększona tak, żeby filtr mniej jej ufał
            if gnss_state == "DEGRADED":
                self._position = Position(
                    lat=lat, lon=lon,
                    accuracy=max(h_acc * _DEGRADED_ACCURACY_MULT, 20.0),
                    source="gnss_degraded",
                )
                return

            # sygnał OK
            self._position = Position(
                lat=lat, lon=lon,
                accuracy=max(h_acc, 1.0),
                source="gnss",
            )


if __name__ == "__main__":
    main()
