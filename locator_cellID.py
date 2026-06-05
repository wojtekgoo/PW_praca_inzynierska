#!/usr/bin/python

"""
Lokalizacja na podstawie stacji bazowych LTE (Cell ID) z modemu Quectel.
Modem służy wyłącznie do odczytu aktualnej stacji bazowej — GNSS jest
obsługiwany przez osobną płytkę (locator_gnss.py).

Logika lokalizacji:
  Zwraca pozycję aktualnie obsługującej stacji bazowej.
  Accuracy zależy od pasma: pasma niskie (≤900 MHz) → 10 000 m, wysokie → 3 000 m.
  Pasmo częstotliwości jest kodowane w polu source, np. "cell|2600MHz",
  aby warstwa prezentacji mogła je wyświetlić na mapie.

Pozycje wież pobierane są z:
  1. btsearch.pl API  — polska baza, źródło podstawowe
  2. OpenCelliD API   — globalny fallback
"""

import re
import time
import serial
import logging
import threading
import requests
from typing import Optional

from position import Position

logger = logging.getLogger(__name__)

SERIAL_PORT        = "/dev/ttyUSB3"
BAUD_RATE          = 115200
BTSEARCH_API_KEY   = ""
OPENCELLID_API_KEY = ""

POLL_INTERVAL  = 10   # sekund między odpytaniami modemu
STALE_AFTER_S  = 15   # get_position() zwraca None jeśli fix starszy niż ta wartość

BTSEARCH_URL   = "https://btsearch.pl/api/v1/search"
OPENCELLID_URL = "https://opencellid.org/cell/get"

# Pasma LTE sub-1 GHz — duży zasięg komórki → wyższa wartość accuracy
_LOW_FREQ_BANDS = {5, 8, 12, 13, 14, 17, 18, 19, 20, 26, 27, 28, 31, 44, 68, 71, 85}

# Przybliżona częstotliwość środkowa pasma [MHz] — do wyświetlenia na mapie
_BAND_FREQ_MHZ = {
    1: 2100, 2: 1900, 3: 1800, 4: 1700, 5:  850,
    7: 2600, 8:  900, 12:  700, 13:  700, 14:  700,
    17:  700, 18:  850, 19:  850, 20:  800, 25: 1900,
    26:  850, 27:  850, 28:  700, 31:  450, 38: 2600,
    39: 1900, 40: 2300, 41: 2500, 44:  700, 66: 1700,
    68:  700, 71:  600, 85:  700,
}


def band_to_freq_str(band: int) -> str:
    """Zwraca czytelny string częstotliwości, np. '2600MHz'."""
    freq = _BAND_FREQ_MHZ.get(band)
    return f"{freq}MHz" if freq else f"band{band}"


def band_accuracy(band: int) -> int:
    """Zwraca domyślną accuracy [m]: 10 000 m dla pasm niskich, 3 000 m dla wysokich."""
    return 10_000 if band in _LOW_FREQ_BANDS else 3_000


# ── AT helpers ────────────────────────────────────────────────────────────────

def send_at(ser: serial.Serial, cmd: str, timeout: float = 3.0) -> str:
    ser.write((cmd + "\r\n").encode())
    response, start = "", time.time()
    while time.time() - start < timeout:
        if ser.in_waiting:
            response += ser.read(ser.in_waiting).decode(errors="ignore")
            if "OK" in response or "ERROR" in response:
                break
        time.sleep(0.05)
    return response.strip()


def get_serving_cell(ser: serial.Serial) -> Optional[dict]:
    resp = send_at(ser, 'AT+QENG="servingcell"')
    m = re.search(
        r'\+QENG: "servingcell","(?:NOCONN|CONNECT)","LTE","FDD",'
        r'(\d+),(\d+),([0-9A-Fa-f]+),\d+,\d+,(\d+),\d+,\d+,(\w+),([-\d]+)',
        resp
    )
    if not m:
        return None
    return {
        "mcc":     m.group(1),
        "mnc":     m.group(2),
        "cell_id": int(m.group(3), 16),
        "band":    int(m.group(4)),
        "lac":     int(m.group(5), 16),
        "rsrp":    int(m.group(6)),
    }


# ── Tower lookup ──────────────────────────────────────────────────────────────

_tower_cache: dict = {}


def lookup_tower_btsearch(ecid: int) -> Optional[tuple]:
    """Zwraca (lat, lon, opis) lub None."""
    try:
        r = requests.post(
            BTSEARCH_URL,
            headers={"Authorization": f"Bearer {BTSEARCH_API_KEY}",
                     "Content-Type": "application/json"},
            json={"query": f"ecid: {ecid}"},
            timeout=5,
        )
        r.raise_for_status()
        data = r.json().get("data", [])
        if not data:
            return None
        loc = data[0].get("location", {})
        lat = loc.get("latitude")
        lon = loc.get("longitude")
        if lat is not None and lon is not None:
            city    = loc.get("city", "")
            address = loc.get("address", "")
            desc    = f"{city} / {address}" if city and address else city or address
            logger.info(f"BTsearch: ecid={ecid} → {desc}")
            return (float(lat), float(lon), desc)
    except Exception as e:
        logger.warning(f"BTsearch: błąd przy ecid={ecid}: {e}")
    return None


def lookup_tower_opencellid(mcc: str, mnc: str, cell_id: int, lac: int) -> Optional[tuple]:
    """Zwraca (lat, lon, opis) lub None. OpenCelliD nie dostarcza opisu adresowego."""
    try:
        r = requests.get(
            OPENCELLID_URL,
            params={"key": OPENCELLID_API_KEY, "radio": "LTE",
                    "mcc": mcc, "mnc": mnc, "lac": lac,
                    "cellid": cell_id, "format": "json"},
            timeout=5,
        )
        d = r.json()
        if "lat" in d:
            logger.info(f"OpenCelliD: znaleziono cell={cell_id}")
            return (float(d["lat"]), float(d["lon"]), "")
    except Exception as e:
        logger.warning(f"OpenCelliD: błąd przy cell={cell_id}: {e}")
    return None


def lookup_tower(mcc: str, mnc: str, cell_id: int, lac: int) -> Optional[tuple]:
    """
    Zwraca (lat, lon, opis) lub None.
    Kolejność: cache → BTsearch.pl → OpenCelliD.
    Accuracy nie jest zwracana — zależy od pasma, obliczana przez band_accuracy().
    """
    cache_key = (cell_id, lac)
    if cache_key in _tower_cache:
        return _tower_cache[cache_key]

    result = lookup_tower_btsearch(cell_id) or \
             lookup_tower_opencellid(mcc, mnc, cell_id, lac)

    if result is None:
        logger.warning(f"Maszt ecid={cell_id} nie znaleziony w BTsearch.pl ani OpenCelliD")
    else:
        _tower_cache[cache_key] = result

    return result


# ── Main locator class ────────────────────────────────────────────────────────

class CellLocator:
    """
    Lokator Cell ID — pozycjonowanie na podstawie aktualnej stacji bazowej LTE.
    Modem Quectel odpytywany co POLL_INTERVAL sekund komendą AT+QENG="servingcell".
    """

    def __init__(self, port: str = SERIAL_PORT):
        self._port     = port
        self._position: Optional[Position] = None
        self._lock     = threading.Lock()
        self._thread:  Optional[threading.Thread] = None
        self._running  = False
        self._last_cell_id: Optional[int] = None
        self._last_address: str = ""

    def start(self) -> None:
        self._running = True
        self._thread  = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=10.0)
        logger.info("CellLocator: stopped")

    def get_position(self) -> Optional[Position]:
        with self._lock:
            if self._position is None:
                return None
            if time.time() - self._position.timestamp > STALE_AFTER_S:
                return None
            return self._position

    def _run(self) -> None:
        while self._running:
            try:
                with serial.Serial(self._port, BAUD_RATE, timeout=3) as ser:
                    logger.info(f"CellLocator: uruchomiony na {self._port}")
                    while self._running:
                        pos = self._update(ser)
                        with self._lock:
                            self._position = pos
                        self._print_dashboard(pos)
                        time.sleep(POLL_INTERVAL)
            except serial.SerialException as e:
                logger.warning(f"CellLocator serial error: {e} — retrying in 5s")
                time.sleep(5.0)

    def _update(self, ser: serial.Serial) -> Optional[Position]:
        cell = get_serving_cell(ser)
        if not cell:
            return self._position   # modem nie odpowiada — zachowaj ostatnią pozycję

        # Wyszukaj współrzędne tylko przy zmianie wieży (handoff)
        if cell["cell_id"] != self._last_cell_id:
            logger.info(f"Cell: handoff → {cell['cell_id']:08X}  band={cell['band']}")
            coords = lookup_tower(cell["mcc"], cell["mnc"],
                                  cell["cell_id"], cell["lac"])
            if coords is None:
                return self._position   # nieznana wieża — zachowaj ostatnią pozycję

            lat, lon, desc = coords
            acc      = band_accuracy(cell["band"])
            freq_str = band_to_freq_str(cell["band"])

            self._last_cell_id = cell["cell_id"]
            self._last_address  = desc
            return Position(
                lat=lat, lon=lon,
                accuracy=acc,
                source=f"cell|{freq_str}",   # np. "cell|2600MHz"
            )

        # Ta sama wieża — odświeżamy timestamp, żeby filtr Kalmana traktował
        # pomiar jako bieżący. Bez tego po 15 s (STALE_AFTER_S) get_position()
        # zwracałby None, a filtr widziałby Cell ID tylko raz, w momencie handoffu.
        if self._position is not None:
            return Position(
                lat=self._position.lat,
                lon=self._position.lon,
                accuracy=self._position.accuracy,
                source=self._position.source,
                timestamp=time.time(),
            )
        return self._position   # jeszcze nie było żadnego handoffu — None

    def _print_dashboard(self, pos: Optional[Position]) -> None:
        if pos is None:
            print("[CELL]  Brak dostępnej pozycji")
            print("[CELL_ADDR]  —")
            return
        freq = pos.source.split("|")[1] if "|" in pos.source else ""
        age  = time.time() - pos.timestamp
        print(
            f"[CELL]  lat={pos.lat:.6f}  lon={pos.lon:.6f}  "
            f"acc={pos.accuracy:.0f}m  pasmo={freq}  age={age:.0f}s"
        )
        print(f"[CELL_ADDR]  {self._last_address if self._last_address else '—'}")


# ── Standalone ────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s  [%(levelname)s]  %(message)s",
                        datefmt="%H:%M:%S")

    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=SERIAL_PORT)
    ap.add_argument("--test", action="store_true",
                    help="Test wyszukiwania wieży dla hardkodowanej komórki")
    args = ap.parse_args()

    if args.test:
        test_mcc, test_mnc, test_lac, test_cid, test_band = "260", "2", 52992, 32907533, 7
        print(f"Testing tower lookup: MCC={test_mcc} MNC={test_mnc} "
              f"LAC={test_lac} ECID={test_cid} band={test_band}")
        coords = lookup_tower(test_mcc, test_mnc, test_cid, test_lac)
        if coords:
            lat, lon, desc = coords
            print(f"  ✓ lat={lat}  lon={lon}")
            print(f"  opis: {desc if desc else '—'}")
            print(f"  freq={band_to_freq_str(test_band)}  accuracy={band_accuracy(test_band)}m")
        else:
            print("  ✗ Nie znaleziono wieży")
    else:
        loc = CellLocator(port=args.port)
        loc.start()
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nZatrzymywanie...")
            loc.stop()
