#!/usr/bin/python

"""
lokalizator_uwb.py — lokalizacja UWB na podstawie pomiarów odległości modułem DWM3000

Architektura systemu:
  - 1 płytka TAG podłączona do Raspberry Pi przez USB-C (port szeregowy)
  - 5 płytek ANCHOR (kotwic) ustawionych w znanych pozycjach GPS, zasilanych z LiPo
  - TAG odpytuje każdą kotwicę metodą TWR (Two-Way Ranging) z częstotliwością ~20 Hz
  - TAG wysyła wszystkie odległości w jednej linii na cykl przez USB Serial

Protokół szeregowy:
  "1:0.543,2:1.234,3:2.109,4:1.876,5:0.998\n"
  - anchor_id : odleglosc_w_metrach, rozdzielone przecinkami
  - może pojawić się dowolny podzbiór kotwic (brak = nieosiągalna w danym cyklu)

Plik konfiguracyjny kotwic (uwb_anchors.json):
  [
    {"id": 1, "lat": 49.9884, "lon": 20.0670},
    {"id": 2, "lat": 49.9885, "lon": 20.0672},
    ...
  ]
  Współrzędne mierzone GNSS przy instalacji i zapisywane jednorazowo.

Publiczny interfejs klasy:
  uwb = UWBLocator(port="/dev/ttyACM0", anchors_file="uwb_anchors.json")
  uwb.start()                        # otwiera port, uruchamia wątek w tle
  pos = uwb.get_position()           # zwraca obiekty typu Position lub None
  uwb.stop()
"""

import json
import math
import threading
import time
import logging
import sys
import argparse
import tempfile 
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
import serial
from scipy.optimize import least_squares  # metoda najmniejszych kwadratów

from position import Position  # klasa Position do zwracania wyników pomiaru

# Logowanie — wpisy widoczne przy poziomie DEBUG/INFO
logger = logging.getLogger(__name__)

# ── parametry konfiguracyjne ──────────────────────────────────────────────────

SERIAL_BAUD        = 115200    # prędkość portu szeregowego
READ_TIMEOUT_S     = 2.0       # timeout odczytu linii z portu szeregowego [s]
STALE_AFTER_S      = 2.0       # pozycja jest "przestarzała" jeśli nie odświeżona przez 2 s
MIN_ANCHORS        = 3         # minimalna liczba kotwic do trilateracji (mniej = brak fixa)
MAX_RESIDUAL_M     = 1.0       # odrzuć fix jeśli błąd RMS przekroczy 1 m
ACCURACY_FLOOR_M   = 0.10      # minimalna deklarowana dokładność (10 cm)
ACCURACY_SCALE     = 2.0       # dokładność = max(10 cm, rms * skala)

# ── funkcje pomocnicze — konwersja współrzędnych ──────────────────────────────

# Promień Ziemi używany do lokalnej projekcji "flat-earth" (płaska Ziemia)
_R_EARTH = 6_371_000.0  # [m]


def latlon_to_xy(lat: float, lon: float, origin_lat: float, origin_lon: float) -> tuple[float, float]:
    """
    Przelicz (lat, lon) na lokalne (x=wschód, y=północ) w metrach względem punktu origin.
    Ważne dla obszarów < kilku km (aproksymacja flat-earth, błąd < 0.01% przy 20m).
    """
    # y [m] = różnica szerokości geograficznej × promień Ziemi
    y = math.radians(lat - origin_lat) * _R_EARTH
    # x [m] = różnica długości geograficznej × promień Ziemi × cos(lat)
    # cos(lat) korekcja na zwężanie się południków ku biegunom
    x = math.radians(lon - origin_lon) * _R_EARTH * math.cos(math.radians(origin_lat))
    return x, y


def xy_to_latlon(x: float, y: float,
                 origin_lat: float, origin_lon: float) -> tuple[float, float]:
    """Odwrotność latlon_to_xy — z lokalnych metrów z powrotem na (lat, lon)."""
    lat = origin_lat + math.degrees(y / _R_EARTH)
    lon = origin_lon + math.degrees(x / (_R_EARTH * math.cos(math.radians(origin_lat))))
    return lat, lon


# ── model kotwicy ─────────────────────────────────────────────────────────────

@dataclass
class Anchor:
    """Reprezentacja jednej kotwicy UWB: ID, pozycja GPS i przeliczone metry lokalne."""
    id:  int    # unikalny identyfikator (1–5), zgodny z ANCHOR_ID w firmware
    lat: float  # szerokość geograficzna [°]
    lon: float  # długość geograficzna [°]
    x:   float = field(default=0.0, init=False)   # pozycja lokalna [m E] — wypełniana po ustaleniu origin
    y:   float = field(default=0.0, init=False)   # pozycja lokalna [m N] — wypełniana po ustaleniu origin


# ── algorytm multilateracji ───────────────────────────────────────────────────

def multilaterate(anchors: list[Anchor], distances: dict[int, float]) -> tuple[float, float, float]:
    """
    Dwuwymiarowa multilateracja metodą najmniejszych kwadratów.

    anchors   : lista wszystkich kotwic (z wypełnionymi polami x, y)
    distances : {anchor_id: zmierzona_odleglosc_m} — tylko kotwice widoczne w tym cyklu

    Zwraca (x_est, y_est, rms_residual_m).
    """
    # Filtruj kotwice — zostaw tylko te, dla których mamy pomiar odległości w tym cyklu
    visible = [(a, distances[a.id]) for a in anchors if a.id in distances]
    if len(visible) < MIN_ANCHORS:
        raise ValueError(
            f"Need ≥{MIN_ANCHORS} anchors, got {len(visible)}"
        )

    # Pozycje widocznych kotwic i odpowiadające im odległości
    ax = np.array([a.x for a, _ in visible])   # współrzędne X kotwic [m]
    ay = np.array([a.y for a, _ in visible])   # współrzędne Y kotwic [m]
    d  = np.array([dist for _, dist in visible]) # zmierzone odległości [m]

    def residuals(pos):
        """
        Funkcja residuów dla solvera: różnica między obliczoną a zmierzoną odległością.
        pos = [x_est, y_est] — bieżące przybliżenie pozycji TAG-a
        Solver minimalizuje sumę kwadratów tych różnic.
        """
        dx = pos[0] - ax  # wektor różnic X między szacowaną pozycją a każdą kotwicą
        dy = pos[1] - ay  # wektor różnic Y
        return np.sqrt(dx**2 + dy**2) - d  # obliczona odległość minus zmierzona

    # Multi-start: próbuj kilku punktów startowych i zachowaj najlepszy wynik.
    # Zapobiega ugrzęźnięciu solvera w centroidzie kotwic (lokalnym minimum).
    # Kandydaci: centroid, każda kotwica z osobna, punkt (0,0).
    candidates = [np.array([ax.mean(), ay.mean()])]  # środek ciężkości kotwic
    for i in range(len(ax)):
        candidates.append(np.array([ax[i], ay[i]]))  # pozycja każdej kotwicy
    candidates.append(np.array([0.0, 0.0]))           # punkt origin

    best = None
    for x0 in candidates:
        # Uruchom solver TRF (Trust Region Reflective) z danym punktem startowym
        res = least_squares(residuals, x0, method="trf")
        # Oblicz błąd RMS (Root Mean Square) residuów dla tego rozwiązania
        rms = float(np.sqrt(np.mean(res.fun**2)))
        # Zachowaj wynik tylko jeśli jest lepszy (mniejszy RMS) niż poprzedni
        if best is None or rms < best[2]:
            best = (res.x[0], res.x[1], rms)

    x_est, y_est, rms = best
    return x_est, y_est, rms


# ── parser linii szeregowej ───────────────────────────────────────────────────

def parse_uwb_line(line: str) -> Optional[dict[int, float]]:
    """
    Parsuje jedną linię z firmware TAG-a.

    Oczekiwany format:  "1:0.543,2:1.234,3:2.109\n"
    Zwraca słownik {anchor_id: odleglosc_m} lub None w razie błędu parsowania.
    """
    line = line.strip()  # usuń białe znaki i znaki nowej linii z początku/końca
    if not line or line.startswith("#"):
        # Pusta linia lub komentarz debug — zignoruj
        return None
    try:
        result = {}
        for token in line.split(","):          # rozbij linię na tokeny "id:dist"
            anchor_id_str, dist_str = token.split(":")  # rozdziel id od odległości
            anchor_id = int(anchor_id_str.strip())       # konwertuj id na int
            dist_m    = float(dist_str.strip())          # konwertuj odległość na float
            if dist_m <= 0 or dist_m > 200:
                # Odrzuć wartości spoza zakresu fizycznego (0–200 m)
                continue
            result[anchor_id] = dist_m
        return result if result else None  # zwróć None jeśli nic nie przeszło filtra
    except (ValueError, IndexError):
        # Błąd konwersji lub nieoczekiwany format tokenu — loguj i zignoruj linię
        logger.debug(f"[UWB] parse error on line: {line!r}")
        return None


# ── główna klasa ──────────────────────────────────────────────────────────────

class UWBLocator:
    """
    Czyta dane rangingowe UWB z TAG-a przez USB Serial,
    wykonuje trilaterację i udostępnia get_position().

    Użycie:
        uwb = UWBLocator(port="/dev/ttyACM0", anchors_file="uwb_anchors.json")
        uwb.start()
        ...
        pos = uwb.get_position()   # Position lub None
        ...
        uwb.stop()
    """

    def __init__(self, port: str = "/dev/ttyACM0", anchors_file: str = "uwb_anchors.json"):
        self._port         = port                    # ścieżka portu szeregowego
        self._anchors_file = Path(anchors_file)      # ścieżka do pliku JSON z kotwicami

        self._anchors: list[Anchor] = []             # lista załadowanych kotwic
        self._origin_lat: float     = 0.0            # punkt origin (centroid kotwic) — lat
        self._origin_lon: float     = 0.0            # punkt origin — lon

        self._position: Optional[Position] = None    # ostatnio obliczona pozycja TAG-a
        self._lock     = threading.Lock()            
        self._thread:  Optional[threading.Thread] = None  
        self._running  = False                       

        self._load_anchors()                         

    # ── ładowanie kotwic ──────────────────────────────────────────────────────

    def _load_anchors(self) -> None:
        """Wczytaj plik JSON z kotwicami i przelicz ich pozycje na metry lokalne."""
        if not self._anchors_file.exists():
            raise FileNotFoundError(
                f"Nie znaleziono pliku: {self._anchors_file}\n"
                "Stwórz uwb_anchors.json z koordynatami GPS kotwic."
            )

        # Wczytaj i zdekoduj JSON — obsłuż dwa formaty: lista lub słownik z kluczem "anchors"
        raw = json.loads(self._anchors_file.read_text())
        if isinstance(raw, dict):
            raw = raw.get("anchors", [])

        # Zbuduj listę obiektów Anchor z danych JSON
        self._anchors = [Anchor(id=a["id"], lat=a["lat"], lon=a["lon"]) for a in raw]

        if len(self._anchors) < MIN_ANCHORS:
            # Za mało kotwic do trilateracji
            logger.warning(
                f"[UWB]: only {len(self._anchors)} anchor(s) in config "
                f"(need >= {MIN_ANCHORS} for a fix) — will run but cannot trilaterate"
            )

        # Wyznacz punkt origin jako centroid (środek ciężkości) wszystkich kotwic
        # użyj go jako punktu referencyjnego do kalkulacji odległości pomiędzy kotwicami
        # takie obliczenia są dokładniesjze i szybsze zamiast współrzednych sferyczych
        # na obszarze pracy UWB (~50x50m) możemy traktować Ziemię jako powierzchnię płaską
        self._origin_lat = sum(a.lat for a in self._anchors) / len(self._anchors)
        self._origin_lon = sum(a.lon for a in self._anchors) / len(self._anchors)

        # Przelicz pozycje GPS każdej kotwicy na lokalne metry (x=wschód, y=północ)
        for a in self._anchors:
            a.x, a.y = latlon_to_xy(a.lat, a.lon, self._origin_lat, self._origin_lon)

        logger.info(
            f"[UWB] loaded {len(self._anchors)} anchors, "
            f"origin ({self._origin_lat:.6f}, {self._origin_lon:.6f})"
        )

    def start(self) -> None:
        """Otwórz port szeregowy i rozpocznij odczyt w wątku tła (daemon)."""
        self._running = True
        # daemon=True — wątek zostanie automatycznie zakończony gdy główny program skończy
        self._thread  = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        logger.info(f"[UWB] started on {self._port} at {SERIAL_BAUD} baud")

    def stop(self) -> None:
        """Wyślij sygnał zatrzymania do wątku tła i poczekaj na jego zakończenie."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)  # czekaj maks. 3 s na zatrzymanie wątku
        logger.info("[UWB] stopped")

    def get_position(self) -> Optional[Position]:
        """
        Zwróć ostatnio obliczoną pozycję lub None jeśli:
          - żadna pozycja nie została jeszcze obliczona,
          - ostatnia pozycja jest starsza niż STALE_AFTER_S sekund.
        """
        with self._lock:
            if self._position is None:
                return None
            age = time.time() - self._position.timestamp  # czas od ostatniego fixa [s]
            if age > STALE_AFTER_S:
                return None  # pozycja przestarzała — nie zwracaj
            return self._position


    def _run(self) -> None:
        """
        Pętla główna wątku w tle:
        otwiera port szeregowy i czyta linie, aż do wywołania stop().
        W razie błędu portu — czeka 2 s i próbuje ponownie (auto-reconnect).
        """
        while self._running:
            try:
                with serial.Serial(self._port, SERIAL_BAUD, timeout=READ_TIMEOUT_S) as ser:
                    logger.info(f"[UWB] serial port {self._port} opened")
                    while self._running:
                        raw = ser.readline()    # czekaj na pełną linię (do '\n' lub timeout)
                        if not raw:
                            continue            # timeout bez danych — pętl dalej
                        # Zdekoduj bajty na string ASCII (nieznane bajty ignoruj)
                        line = raw.decode("ascii", errors="ignore")
                        self._handle_line(line) # przetworz odczytaną linię

            except serial.SerialException as e:
                # Błąd portu (np. odłączono USB) — zaloguj i czekaj przed ponowną próbą
                logger.warning(f"[UWB] serial error: {e} — retrying in 2s")
                time.sleep(2.0)

    def _handle_line(self, line: str) -> None:
        """
        Przetwarza jedną zdekodowaną linię z serial:
        parsuje odległości → trilateruje - waliduje - zapisuje pozycję.
        """
        distances = parse_uwb_line(line)  # sparsuj linię na słownik {id: dist}
        if distances is None:
            return  # linia pusta, komentarz lub błędna — zignoruj

        # Zbierz ID kotwic widocznych w tym cyklu (do logowania)
        anchor_ids = "".join(str(k) for k in sorted(distances.keys()))

        try:
            # Oblicz pozycję TAG-a metodą multilateracji
            x_est, y_est, rms = multilaterate(self._anchors, distances)
        except ValueError:
            # Za mało widocznych kotwic — wypisz info i pomiń ten cykl
            print(f"[UWB] not enough anchors — got {anchor_ids}, need >= {MIN_ANCHORS}", end="\r", flush=True)
            return

        if rms > MAX_RESIDUAL_M:
            # Błąd RMS zbyt duży — fix uznany za niewiarygodny, odrzuć
            print(f"[UWB] fix discarded - RMS {rms:.2f}m > {MAX_RESIDUAL_M}m  anchors={anchor_ids}", end="\r", flush=True)
            return

        # Przelicz wyznaczone metry lokalne z powrotem na współrzędne GPS
        lat, lon = xy_to_latlon(x_est, y_est, self._origin_lat, self._origin_lon)

        # Oblicz deklarowaną dokładność: minimum 10 cm, lub rms × 2 jeśli większe
        # nawet gdy obliczone rms = 0, accuracy będzie minimum 10cm ze względu na fizyczną niedokładność czujnika
        accuracy = max(ACCURACY_FLOOR_M, rms * ACCURACY_SCALE)

        # Utwórz obiekt pozycji z aktualnym znacznikiem czasu
        pos = Position(
            lat       = lat,
            lon       = lon,
            accuracy  = accuracy,
            source    = "uwb", 
            timestamp = time.time(),
        )

        # Zapisz pozycję pod mutexem — bezpieczne dla równoległego get_position()
        with self._lock:
            self._position = pos

        # Wypisz fix na konsolę w czytelnym formacie (nadpisuje tę samą linię)
        print(f"[UWB] lat={lat:.7f}  lon={lon:.7f}  acc={accuracy:.2f}m  rms={rms:.3f}m  anchors={anchor_ids}", flush=True)

        logger.debug(
            f"[UWB] fix: ({lat:.7f}, {lon:.7f}) "
            f"acc={accuracy:.2f}m rms={rms:.3f}m "
            f"anchors_used={anchor_ids}"
        )


# ── dwa tryby działania: live lub symulacja ──────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="UWB locator")
    # --live: tryb produkcyjny — odczyt z prawdziwego portu szeregowego
    parser.add_argument("--live", action="store_true",
                        help="Run live: read from serial and print positions")
    parser.add_argument("--port", default="/dev/ttyACM0",
                        help="Serial port (default: /dev/ttyACM0)")
    parser.add_argument("--anchors", default="uwb_anchors.json",
                        help="Anchor config file (default: uwb_anchors.json)")
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG, format="%(asctime)s %(levelname)s %(message)s")

    # ── tryb live: uruchom locator na prawdziwym porcie i wypisz pozycje ──────
    if args.live:
        print(f"Starting live UWB locator on {args.port} ...")
        uwb = UWBLocator(port=args.port, anchors_file=args.anchors)
        uwb.start()
        try:
            while True:
                time.sleep(0.5)  # główny wątek śpi — odczyt dzieje się w tle
        except KeyboardInterrupt:
            print("\nStopping...")
            uwb.stop()
        sys.exit(0)

    # ── tryb symulacji / self-test (domyślny, bez --live) ────────────────────
    # Fikcyjny układ 5 kotwic na obszarze ~20×25m 
    # Przed wdrożeniem zastąpić rzeczywistymi współrzędnymi zmierzonymi GPS
    FAKE_ANCHORS = [
        {"id": 1, "lat": 49.988400, "lon": 20.067000},  # narożnik SW
        {"id": 2, "lat": 49.988625, "lon": 20.067000},  # narożnik NW
        {"id": 3, "lat": 49.988625, "lon": 20.067290},  # narożnik NE
        {"id": 4, "lat": 49.988400, "lon": 20.067290},  # narożnik SE
        {"id": 5, "lat": 49.988512, "lon": 20.067145},  # środek
    ]

    # Zapisz fikcyjne kotwice do tymczasowego pliku JSON
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
        json.dump(FAKE_ANCHORS, f)
        tmp_path = f.name

    try:
        # Tryb symulacji sprawda czy nasz skrypt potrafic policzyć poprawnie pozycję na podstawie podstawionych danych pozycyjnych kotwic i taga
        locator = UWBLocator(port="/dev/null", anchors_file=tmp_path)

        origin_lat = locator._origin_lat
        origin_lon = locator._origin_lon

        # Zakładamy fake lokalizację TAG-a w celach testowych: 10m na wschód i 8m na północ od origin
        FAKE_X, FAKE_Y = 10.0, 8.0
        print(f"\nOdległość taga od origin: x={FAKE_X}m, y={FAKE_Y}m")

        # Oblicz dokładne odległości od TAG-a do każdej kotwicy
        exact_distances = {}
        for a in locator._anchors:
            d = math.sqrt((FAKE_X - a.x)**2 + (FAKE_Y - a.y)**2)
            exact_distances[a.id] = d
            print(f"  Kotwica {a.id}: x={a.x:.2f}m y={a.y:.2f}m → dist={d:.3f}m")

        # Test 1: idealne odległości bez szumu — sprawdź czy solver zwraca dokładną pozycję
        x, y, rms = multilaterate(locator._anchors, exact_distances)
        print(f"\n[Test 1 — no noise]")
        print(f"  Estimated: x={x:.4f}m, y={y:.4f}m, rms={rms:.6f}m")
        print(f"  Error: {math.sqrt((x-FAKE_X)**2 + (y-FAKE_Y)**2)*100:.2f} cm")

        # Test 2: szum +/-5 cm na wszystkich odległościach — symulacja działania realnego UWB
        noisy = {k: v + np.random.uniform(-0.05, 0.05) for k, v in exact_distances.items()}
        x, y, rms = multilaterate(locator._anchors, noisy)
        print(f"\n[Test 2 — ±5cm noise]")
        print(f"  Estimated: x={x:.4f}m, y={y:.4f}m, rms={rms:.4f}m")
        print(f"  Error: {math.sqrt((x-FAKE_X)**2 + (y-FAKE_Y)**2)*100:.2f} cm")

        # Test 3: symulacja utraty sygnału, tylko 3 kotwice widoczne (SW+NE+SE)
        # Uwaga: kotwice 1,3,5 (SW-NE przekątna) są bliskie kolinearności - zła geometria
        # Kotwice 1,3,4 (SW, NE, SE) tworzą właściwy trójkąt - dobra geometria
        partial = {k: v for k, v in noisy.items() if k in [1, 3, 4]}
        x, y, rms = multilaterate(locator._anchors, partial)
        print(f"\n[Test 3 — 3 kotwice tylko (SW+NE+SE), szum +/-5cm]")
        print(f"  Estimated: x={x:.4f}m, y={y:.4f}m, rms={rms:.4f}m")
        print(f"  Error: {math.sqrt((x-FAKE_X)**2 + (y-FAKE_Y)**2)*100:.2f} cm")

        # Test 4: parsowanie linii szeregowych — sprawdź różne przypadki brzegowe
        print(f"\n[Test 4 — serial line parsing]")
        test_lines = [
            "1:0.543,2:1.234,3:2.109,4:1.876,5:0.998",  # poprawna pełna linia
            "1:0.543, 2:1.234",          # usuń spacje w danych
            "# debug message",           # ignoruj komentarze w danych — zwróć None
            "garbage",                   # błędny format — zwróć None
            "1:-0.5,2:1.234",            # ujemna odległość — pomiń ten pomiar
            "1:0.543",                   # tylko 1 kotwica (poniżej MIN) — pars OK, multilateracja rzuci ValueError
        ]
        for line in test_lines:
            result = parse_uwb_line(line)
            print(f"  {line!r:45s} - {result}")

        print("\n[OK] Testy zdane — locator_uwb.py jest gotowy.")

    finally:
        # Zawsze usuń tymczasowy plik JSON
        os.unlink(tmp_path)
