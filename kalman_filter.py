#!/usr/bin/python

"""
kalman_filter.py — fuzja pozycji z wielu źródeł dla śledzenia pieszego.

Moduł realizuje filtr Kalmana fuzjujący odczyty pozycji z następujących źródeł:
  • UWB   — Ultra-Wideband (precyzyjne pozycjonowanie wewnątrz budynków)
  • GNSS  — satelitarny system nawigacji (GPS/Galileo/GLONASS)
  • WiFi  — pozycjonowanie na podstawie sygnałów Wi-Fi
  • Cell  — pozycjonowanie na podstawie stacji bazowych sieci komórkowej
  • IMU   — jednostka pomiarowa inercyjna (akcelerometr + żyroskop + magnetometr)

Orientacja 3D urządzenia jest estymowana przez filtr AHRS Madgwicka
(madgwick_filter.py), który fuzjuje akcelerometr, żyroskop i magnetometr
w kwaternion opisujący pełną orientację w przestrzeni. Dzięki temu:
  • przyspieszenie z akcelerometru jest poprawnie obracane do układu ENU
    za pomocą pełnej macierzy rotacji 3×3 (zamiast rotacji 2D tylko po yaw),
  • grawitacja (~9.81 m/s²) jest odejmowana w układzie ENU, co eliminuje
    fałszywe przyspieszenie powstające przy przechyle urządzenia,
  • kurs (heading) pochodzi bezpośrednio z kwaterniona, nie z osobnej
    korekcji magnetometrem — upraszcza to filtr pozycyjny.

Wektor stanu filtru: [x, y, vx, vy]
  x, y   : pozycja w metrach w lokalnym układzie ENU (East-North-Up)
           względem centroidu kotwic UWB
  vx, vy : prędkość w m/s (składowa wschodnia i północna)

UWAGA: kurs (heading) NIE jest częścią wektora stanu filtru Kalmana.
  Jest estymowany niezależnie przez filtr Madgwicka i dołączany do wyniku.
  Dzięki temu model przejścia stanu jest liniowy — filtr jest standardowym
  filtrem Kalmana (LKF), a nie rozszerzonym (EKF), co poprawia stabilność
  numeryczną i upraszcza implementację.

Krok predykcji: wykonywany z częstotliwością IMU (~100 Hz).
  Przyspieszenie z IMU jest najpierw obracane do ENU przez filtr Madgwicka,
  a następnie pozbawione składowej grawitacyjnej. Czyste przyspieszenie
  liniowe w ENU jest użyte w równaniach kinematycznych.

Krok korekcji (aktualizacji): wykonywany za każdym razem, gdy którykolwiek
  z lokatorów (UWB/GNSS/WiFi/Cell) zwróci nowy pomiar pozycji.

Szum pomiarowy R = accuracy² → mała wartość accuracy oznacza duże zaufanie
  do pomiaru. Filtr automatycznie waży źródła — nie wymaga ręcznego
  ustawiania priorytetów.

Początek układu ENU: centroid (średnia arytmetyczna pozycji) kotwic UWB,
  wczytywany z pliku uwb_anchors.json.

Wymagania wobec IMULocator:
  Metoda get_reading() musi zwracać obiekt z pełnymi danymi 9-osiowymi:
    ax, ay, az  — przyspieszenie w m/s² (3 osie)
    gx, gy, gz  — prędkość kątowa w °/s (3 osie)
    mx, my, mz  — pole magnetyczne (skalibrowane, dowolne jednostki)
  Dane powinny być po kalibracji (bias gyro/accel, hard/soft iron mag)
  — zgodnie z calibration.json używanym przez madgwick_filter.py.

Interfejs publiczny:
  kf = KalmanFilter(anchors_file="uwb_anchors.json")
  kf.start()                       # uruchamia wątek filtra i wszystkie lokatory
  result = kf.get_result()         # → FusedPosition lub None
  kf.stop()                        # zatrzymuje filtr i lokatory

Uruchomienie samodzielne:
  python kalman_filter.py
  python kalman_filter.py --anchors /ścieżka/do/uwb_anchors.json
"""

import json
import math
import time
import logging
import threading
import argparse
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import re
import subprocess
import sys
import numpy as np

# Import klas lokatorów — każdy odpowiada za komunikację z jednym typem czujnika
# i udostępnia metodę get_position() zwracającą obiekt Position (lat, lon, accuracy)
from position import Position
from locator_uwb  import UWBLocator       # lokator UWB (Ultra-Wideband)
from locator_wifi import WiFiLocator       # lokator Wi-Fi (fingerprinting/trilateration)
from locator_cellID import CellLocator     # lokator Cell ID (stacje bazowe GSM/LTE)
from locator_imu  import IMULocator        # lokator IMU (akcelerometr + żyroskop + magnetometr)

# Filtr AHRS Madgwicka — estymuje orientację 3D urządzenia jako kwaternion,
# fuzjując dane z akcelerometru, żyroskopu i magnetometru.
# Kwaternion jest używany do:
#   1) obrotu przyspieszenia z układu ciała do ENU (macierz rotacji 3×3),
#   2) odjęcia grawitacji w układzie ENU,
#   3) wyznaczenia kursu (heading) do wyświetlania.
from madgwick_filter import MadgwickFilter

logger = logging.getLogger(__name__)

# ── Stałe fizyczne ───────────────────────────────────────────────────────────

GRAVITY = 9.81   # przyspieszenie grawitacyjne w m/s²

# ── Konfiguracja ─────────────────────────────────────────────────────────────

# Ile próbek IMU podać filtrowi Madgwicka na starcie, aby kwaternion
# skonwergował do poprawnej orientacji. Przy ~100 Hz, 300 próbek ≈ 3 s.
# W tym czasie urządzenie powinno być w miarę nieruchome.
AHRS_WARMUP_SAMPLES = 300

# Szum procesowy (process noise) — określa, jak bardzo filtr „ufa" modelowi
# ruchu (predykcji z IMU) w porównaniu do pomiarów z lokatorów.
#   Wyższe wartości → filtr szybciej reaguje na zmiany pozycji (mniej wygładzania)
#   Niższe wartości  → filtr bardziej wygładza trajektorię (większa inercja)
Q_POS = 0.01   # m² na sekundę  — szum procesowy pozycji (x, y)
Q_VEL = 0.10   # (m/s)² na sekundę — szum procesowy prędkości (vx, vy)

# Maksymalny wiek pomiaru (w sekundach), powyżej którego odczyt jest odrzucany.
# Każde źródło ma inny dopuszczalny wiek, ponieważ różnią się częstotliwością
# aktualizacji i stabilnością sygnału.
MAX_FIX_AGE = {
    "uwb":            0.5,    # UWB — bardzo szybki, pomiar powinien być świeży
    "gnss":           2.0,    # GNSS — aktualizacja co ~1s
    "gnss_degraded":  2.0,    # GNSS zdegradowany (np. słaba widoczność satelitów)
    "wifi":          30.0,    # WiFi — skanowanie trwa kilkanaście sekund
    "cell":          15.0,    # Cell ID — odpytywanie modemu co 10s
}

# Częstotliwość pętli predykcji, gdy IMU jest niedostępne (Hz).
# Filtr nadal działa, ale zakłada ruch ze stałą prędkością (bez przyspieszenia).
FALLBACKpredict_step_HZ = 10.0

# ── Klasa wyjściowa ──────────────────────────────────────────────────────────

@dataclass
class FusedPosition:
    """
    Wynik działania filtru Kalmana — sfuzjowana (połączona) pozycja.

    Pola:
      lat, lon   : sfuzjowana pozycja w stopniach dziesiętnych (WGS84)
      accuracy   : szacowany błąd horyzontalny 1-sigma w metrach
                   (68% pomiarów mieści się w tym promieniu)
      vx, vy     : prędkość w m/s (składowa wschodnia i północna)
      speed      : prędkość skalarna w m/s (moduł wektora [vx, vy])
      heading    : kurs w stopniach kompasu: 0°=Północ, 90°=Wschód, zgodnie
                   z ruchem wskazówek zegara — pochodzi z filtra Madgwicka
      source     : które źródło(-a) napędzały ostatnią korekcję,
                   np. "uwb+imu", "gnss+imu", "imu_only"
      timestamp  : czas tej estymaty (czas systemowy Unix)
    """
    lat:       float
    lon:       float
    accuracy:  float
    vx:        float
    vy:        float
    speed:     float
    heading:   float          # kurs kompasowy: 0°=Północ, 90°=Wschód, CW
    source:    str
    timestamp: float = field(default_factory=time.time)

    def __str__(self) -> str:
        """Reprezentacja tekstowa — używana do logowania i debugowania."""
        age = time.time() - self.timestamp
        return (
            f"[FUSED {self.source:>12s}]  "
            f"lat={self.lat:.7f}  lon={self.lon:.7f}  "
            f"acc=~{self.accuracy:.1f}m  "
            f"speed={self.speed:.2f}m/s  "
            f"hdg={self.heading:.1f}°  "
            f"age={age:.1f}s"
        )

# ── Funkcje pomocnicze — konwersja współrzędnych ─────────────────────────────
# transformacja współrzędnych z układu sferycznego na płaski
def latlon_to_xy(lat: float, lon: float, origin_lat: float, origin_lon: float) -> tuple:
    """
    Konwersja współrzędnych geograficznych (lat/lon) na lokalne metry
    w układzie ENU (East-North-Up) względem punktu odniesienia (origin).

    Stosuje uproszczony model kulistej Ziemi o promieniu R = 6 371 000 m.
    Dokładność jest wystarczająca dla odległości do kilku kilometrów.

    Zwraca:
      x — odległość w kierunku wschodnim (East) w metrach
      y — odległość w kierunku północnym (North) w metrach
    """
    R = 6_371_000.0   # promień Ziemi w metrach
    # Składowa x (East): różnica długości geograficznej × promień × cos(szer. geogr.)
    # Mnożenie przez cos(φ) kompensuje zbieżność południków ku biegunom.
    x = math.radians(lon - origin_lon) * R * math.cos(math.radians(origin_lat))
    # Składowa y (North): różnica szerokości geograficznej × promień
    y = math.radians(lat - origin_lat) * R
    return x, y

def xy_to_latlon(x: float, y: float, origin_lat: float, origin_lon: float) -> tuple:
    """
    Odwrotna konwersja: lokalne metry ENU → współrzędne geograficzne (lat/lon).
    Operacja odwrotna do latlon_to_xy().
    """
    R = 6_371_000.0
    lat = origin_lat + math.degrees(y / R)
    lon = origin_lon + math.degrees(x / (R * math.cos(math.radians(origin_lat))))
    return lat, lon

def load_anchor_centroid(anchors_file: str) -> tuple:
    """
    Wczytuje plik JSON z pozycjami kotwic UWB i oblicza ich centroid
    (średnią arytmetyczną współrzędnych).

    Centroid służy jako początek (origin) lokalnego układu ENU — dzięki temu
    wartości x, y w filtrze są bliskie zeru i wyrażone w metrach,
    co zapewnia dobrą stabilność numeryczną obliczeń macierzowych.

    Zwraca: (lat, lon) — centroid kotwic w stopniach dziesiętnych.
    """
    path = Path(anchors_file)
    if not path.exists():
        raise FileNotFoundError(f"Nie znaleziono pliku kotwic: {anchors_file}")
    with open(path) as f:
        data = json.load(f)
    # Obsługa dwóch formatów: lista kotwic lub obiekt z kluczem "anchors"
    anchors = data if isinstance(data, list) else data.get("anchors", [])
    if not anchors:
        raise ValueError("Brak kotwic w pliku")
    lat = sum(a["lat"] for a in anchors) / len(anchors)
    lon = sum(a["lon"] for a in anchors) / len(anchors)
    logger.info(f"Kalman: początek ENU ustawiony na centroid kotwic ({lat:.7f}, {lon:.7f})")
    return lat, lon

def kwaternion_na_dcm(q: np.ndarray) -> np.ndarray:
    """
    https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    Konwersja kwaterniona na macierz kosinusów kierunkowych (DCM),
    czyli macierz rotacji 3×3 przekształcającą wektor z układu odniesienia
    czujnika (ang. body frame) do układu ENU (odniesionego względem Ziemi):
    a_enu = R * a_body
    """
    w, x, y, z = q

    R = np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - w*z),       2*(x*z + w*y)],
        [2*(x*y + w*z),       1 - 2*(x*x + z*z),   2*(y*z - w*x)],
        [2*(x*z - w*y),       2*(y*z + w*x),       1 - 2*(x*x + y*y)],
    ])
    return R

class KalmanFilter:
    """
    Filtr Kalmana fuzjujący dane z UWB, GNSS, WiFi, Cell i IMU
    z pełną obsługą orientacji 3D (filtr AHRS Madgwicka).

    W porównaniu do wersji bez AHRS, filtr ten:
      • poprawnie obsługuje przechylone urządzenie (pitch/roll),
      • odejmuje grawitację w układzie ENU za pomocą macierzy rotacji 3×3,
      • ma prostszy model liniowy (LKF zamiast EKF) — θ nie jest
        w wektorze stanu, bo kurs pochodzi z kwaterniona Madgwicka.

    Filtr działa w wątku tła i realizuje następujący cykl:
      1. Rozgrzewka AHRS — filtr Madgwicka konwerguje do poprawnej orientacji
      2. Krok predykcji — IMU → Madgwick → rotacja do ENU → odjęcie grawitacji
         → równania kinematyczne
      3. Krok korekcji — wykonywany przy każdym nowym pomiarze pozycji
         z dowolnego lokatora (UWB/GNSS/WiFi/Cell)
      4. Publikacja wyniku — dostępnego przez get_result()
    """

    def __init__(self, anchors_file: str = "uwb_anchors.json",
                 uwb_port:   str = "/dev/ttyACM0",
                 gnss_port:  str = "/dev/ttyACM1",
                 gnss_script: str = "/home/wojtek/locator_gnss.py",
                 cell_port:  str = "/dev/ttyUSB3"):

        # Wczytanie centroidu kotwic UWB — punkt odniesienia układu ENU
        self._origin_lat, self._origin_lon = load_anchor_centroid(anchors_file)

        # ── Lokatory — obiekty odpowiedzialne za odczyt pozycji z czujników ──
        self._uwb  = UWBLocator(port=uwb_port, anchors_file=anchors_file)
        self._wifi = WiFiLocator()
        self._cell = CellLocator(port=cell_port)
        self._imu  = IMULocator()

        # ── GNSS — uruchamiany jako oddzielny proces (subprocess) ────────────
        # Skrypt locator_gnss.py komunikuje się z odbiornikiem GNSS przez port
        # szeregowy i wypisuje pozycję w formacie jednoliniowym na stdout.
        self._gnss_script = gnss_script
        self._gnss_port   = gnss_port
        self._gnss_pos:   Optional[Position] = None   # ostatni odczyt GNSS
        self._gnss_lock   = threading.Lock()           # ochrona dostępu wielowątkowego
        self._gnss_proc   = None                       # uchwyt do procesu potomnego

        # ── Filtr AHRS Madgwicka — orientacja 3D urządzenia ─────────────────
        # Kwaternion z Madgwicka jest aktualizowany co 10 ms (100 Hz) i używany
        # do budowy macierzy rotacji body→ENU oraz do odczytu kursu.
        self.f_madgwick = MadgwickFilter(beta=0.1, sample_rate=100)
        self.madgwick_ready = False   # flaga: czy kwaternion skonwergował

        # ── Wektor stanu: [x, y, vx, vy] ─────────────────────────────────────
        # ZMIANA vs. poprzednia wersja: kurs θ usunięty z wektora stanu.
        # Orientacja jest estymowana przez filtr Madgwicka niezależnie od
        # filtru Kalmana. Dzięki temu model przejścia stanu jest liniowy
        # (brak rotacji zależnej od θ) i filtr upraszcza się z EKF do LKF.
        self._x = np.zeros(4)           # wektor stanu — inicjalizacja zerami
                                        # self._x[0]: x (position East in meters)
                                        # self._x[1]: y (position North in meters)
                                        # self._x[2]: vx (velocity East in m/s)
                                        # self._x[3]: vy (velocity North in m/s)
        # Macierz kowariancji — duża niepewność początkowa                           
        self._P = np.eye(4) * 100.0     

        # ── Zmienne pomocnicze ───────────────────────────────────────────────
        self._last_imu_t:    float = 0.0   # czas ostatniego odczytu IMU
        self._last_sources:  list  = []    # źródła użyte w ostatniej korekcji
        self.last_pos_used: dict  = {}    # słownik: źródło → timestamp ostatnio użytego pomiaru
        self._last_fix_src:  str   = ""    # etykieta ostatniego źródła (nie-IMU)
        self._last_fix_t:    float = 0.0   # czas ostatniego pomiaru pozycyjnego
        self._FIX_LABEL_TTL: float = 15.0  # jak długo (s) wyświetlać etykietę źródła

        # ── ZUPT (Zero Velocity Update) ──────────────────────────────────────
        # Technika korekcji prędkości, gdy urządzenie jest nieruchome.
        # Przechowujemy okno ostatnich wartości |a| (moduł przyspieszenia
        # liniowego w ENU, po odjęciu grawitacji), aby wykrywać bezruch.
        # Jeśli stoisz w miejscu, algorytm wykryje to przez IMU i "siłowo" wyzeruje prędkość, 
        # nawet jeśli GNSS (ze względu na swój naturalny szum) twierdziłby, że minimalnie się przesuwasz.
        self._accel_window: list  = []     # ostatnie wartości |a_linear_enu|
        self._ZUPT_WINDOW   = 20           # rozmiar okna (~0.2s przy 100 Hz)
        self._ZUPT_THRESH   = 0.08         # próg odch. std. (m/s²) — poniżej = bezruch

        # ── Wynik i synchronizacja wątków ────────────────────────────────────
        self._result: Optional[FusedPosition] = None  # ostatni wynik filtru
        self._lock   = threading.Lock()               # mutex chroniący _result
        self._thread: Optional[threading.Thread] = None
        self._running = False

    # ── Interfejs publiczny ──────────────────────────────────────────────────

    def start(self) -> None:
        """
        Uruchamia wszystkie lokatory oraz główny wątek filtru Kalmana.
        Lokatory działają we własnych wątkach i zbierają pomiary niezależnie.
        """
        self._uwb.start()
        self._wifi.start()
        self._cell.start()
        self._imu.start()

        # GNSS działa jako osobny subprocess — uruchom wątek czytający jego stdout
        threading.Thread(target=self._run_gnss, daemon=True).start()

        # Główny wątek filtru
        self._running = True
        self._thread  = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        logger.info("KalmanFilter: uruchomiony")

    def stop(self) -> None:
        """
        Zatrzymuje filtr i wszystkie lokatory.
        Czeka do 5 sekund na zakończenie wątku filtra.
        """
        self._running = False
        if self._thread:
            self._thread.join(timeout=5.0)
        if self._gnss_proc:
            self._gnss_proc.terminate()
        self._uwb.stop()
        self._wifi.stop()
        self._cell.stop()
        self._imu.stop()
        logger.info("KalmanFilter: zatrzymany")

    def get_result(self) -> Optional[FusedPosition]:
        """
        Zwraca najnowszą sfuzjowaną pozycję.
        Bezpieczna do wywołania z dowolnego wątku (chroniona mutexem).
        Zwraca None, jeśli filtr nie wyznaczył jeszcze żadnej pozycji.
        """
        with self._lock:
            return self._result

    # ── Główna pętla wątku tła ────────────────────────────────────────────────

    def _run(self) -> None:
        """
        Główna pętla filtru Kalmana, działająca w osobnym wątku.

        Sekwencja w każdej iteracji:
          1. Odczytaj pełne dane 9-osiowe z IMU
          2. Zaktualizuj filtr Madgwicka (kwaternion orientacji)
          3. Zbuduj macierz rotacji body→ENU z kwaterniona
          4. Obróć przyspieszenie do ENU i odejmij grawitację
          5. Krok predykcji — równania kinematyczne z czystym przyspieszeniem
          6. Detekcja bezruchu (ZUPT) — wyzeruj prędkość, jeśli urządzenie stoi
          7. Krok korekcji pozycji — dla każdego lokatora z nowym pomiarem
          8. Przelicz stan na lat/lon i opublikuj wynik
        """
        # Na początku rozgrzewka filtra AHRS (kwaternion musi skonwergować)
        self.init_madgwick()

        self._last_imu_t = time.time()

        while self._running:
            # ── Odczyt IMU i aktualizacja orientacji ─────────────────────────
            reading = self._imu.get_reading()    # odczyt 9-osiowy z IMU
            now     = time.time()
            dt      = now - self._last_imu_t     # czas od ostatniej iteracji
            self._last_imu_t = now

            if reading and self.madgwick_ready and 0 < dt < 0.5:
                # ── Krok 1: Aktualizacja filtra Madgwicka ────────────────────
                # Do filtra M. trafiają odczyty z 3 sensorów.
                # Madgwick wyznacza kwaternion
                # reprezentujący orientację urządzenia w przestrzeni 3D.
                self.f_madgwick.update(
                    reading.ax, reading.ay, reading.az, # Przyspieszenie liniowe + grawitacja (z akcelerometru)
                    reading.gx, reading.gy, reading.gz, # Prędkość kątowa (z żyroskopu)
                    reading.mx, reading.my, reading.mz, # Pole magnetyczne (z magnetometru)
                )

                # ── Krok 2: Rotacja przyspieszenia do ENU ────────────────────
                # Macierz rotacji R (3×3) przekształca wektor z układu odniesienia
                # czujnika do układu ENU. Dzięki temu niezaleznie jak trzymamy urządzenie
                # oś Z wektora przyspieszenia skierowana jest w górę
                R = kwaternion_na_dcm(self.f_madgwick.q)

                # Wektor przyspieszenia w układzie odniesienia czujnika (body frame) [m/s²]
                a_body = np.array([reading.ax, reading.ay, reading.az])

                # Obrót do ENU i odjęcie grawitacji.
                # Akcelerometr mierzy siłę właściwą (specific force), czyli
                # sumę przyspieszenia liniowego i reakcji na grawitację.
                # W bezruchu na poziomej powierzchni odczyt = [0, 0, +9.81],
                # bo czujnik „czuje" siłę podtrzymującą go przed spadkiem.
                # Po obrocie do ENU odejmujemy wektor grawitacji [0, 0, 9.81],
                # pozostawiając czyste przyspieszenie liniowe ruchu.
                a_enu = R @ a_body - np.array([0.0, 0.0, GRAVITY])

                # Składowe East i North — wejście do filtru Kalmana_accel_window
                ax_enu = float(a_enu[0])   # przyspieszenie na wschód (m/s²)
                ay_enu = float(a_enu[1])   # przyspieszenie na północ (m/s²)

                # ── Krok 3: Predykcja filtru Kalmana ─────────────────────────
                # Przyspieszenie jest już w układzie ENU i bez grawitacji,
                # więc predict_step() nie musi nic obracać — prosty model liniowy.
                self.predict_step(ax_enu, ay_enu, dt)

                # ── Krok 4: ZUPT — detekcja bezruchu i zerowanie prędkości ──
                # Moduł przyspieszenia liniowego w ENU (po odjęciu grawitacji).
                # Gdy urządzenie stoi, ten moduł oscyluje blisko zera.
                a_linear_mag = math.sqrt(ax_enu**2 + ay_enu**2)
                self._accel_window.append(a_linear_mag)
                if len(self._accel_window) > self._ZUPT_WINDOW:
                    self._accel_window.pop(0)   # utrzymuj stały rozmiar okna

                # Jeśli odchylenie standardowe w oknie jest poniżej progu,
                # urządzenie jest nieruchome — zastosuj ZUPT
                if len(self._accel_window) == self._ZUPT_WINDOW:
                    mean = sum(self._accel_window) / self._ZUPT_WINDOW
                    std  = math.sqrt(sum((v - mean)**2 for v in self._accel_window)
                                     / self._ZUPT_WINDOW)
                    if std < self._ZUPT_THRESH:
                        self.apply_zupt()

                # Utrzymuj częstotliwość pętli na ~100 Hz
                sleep_dt = max(0.0, (1.0 / 100.0) - (time.time() - now))
            else:
                # Brak IMU — „dryfuj" ze stałą prędkością (brak przyspieszenia)
                self.predict_step(0.0, 0.0, dt)
                sleep_dt = 1.0 / FALLBACKpredict_step_HZ

            # ── Krok 5: korekcja pozycji (dane ze wszystkich lokatorów) ──────────
            sources_used = []

            for name, pos in self.fresh_fixes():
                # Przelicz pozycję z lat/lon na lokalne metry ENU
                x_m, y_m = latlon_to_xy(pos.lat, pos.lon,
                                        self._origin_lat, self._origin_lon)
                # Krok korekcji — aktualizuj stan na podstawie pomiaru
                self.update_step(x_m, y_m, pos.accuracy)
                sources_used.append(name)
                self.last_pos_used[name] = pos.timestamp

            # ── Publikacja wyniku ─────────────────────────────────────────────
            # Przelicz estymowaną pozycję z ENU z powrotem na lat/lon
            lat, lon = xy_to_latlon(self._x[0], self._x[1],
                                    self._origin_lat, self._origin_lon)

            # Szacowanie dokładności z diagonali macierzy kowariancji.
            # P[0,0] to wariancja x, P[1,1] to wariancja y.
            # Średnia daje izotropowy (kołowy) estymator błędu.
            accuracy = math.sqrt((self._P[0, 0] + self._P[1, 1]) / 2.0)
            accuracy = max(accuracy, 0.05)   # minimum 5 cm — zabezpieczenie

            # Prędkość: składowe i moduł (prędkość skalarna)
            vx, vy = float(self._x[2]), float(self._x[3])
            speed  = math.sqrt(vx ** 2 + vy ** 2)

            # Kurs z filtra Madgwicka — poprawnie obsługuje przechył urządzenia.
            # euler_angles() zwraca (heading, pitch, roll) w stopniach,
            # gdzie heading jest kursem kompasowym (0°=N, 90°=E, CW).
            if self.madgwick_ready:
                heading_deg, _, _ = self.f_madgwick.euler_angles()
            else:
                heading_deg = 0.0

            # Ustalenie etykiety źródła do wyświetlania
            if sources_used:
                # Właśnie wykonano korekcję — pokaż, które lokatory wzięły udział
                base = "+".join(sources_used)
                source = base + "+imu" if reading else base
                self._last_fix_src = source
                self._last_fix_t   = time.time()
            elif time.time() - self._last_fix_t < self._FIX_LABEL_TTL and self._last_fix_src:
                # Brak nowej korekcji, ale ostatnia jest jeszcze „świeża"
                source = self._last_fix_src + " (imu)" if reading else self._last_fix_src
            elif reading and self.madgwick_ready:
                # Tylko IMU — brak żadnych pomiarów pozycyjnych
                source = "imu_only"
            else:
                # Brak IMU i brak pomiarów — filtr „dryfuje" ze stałą prędkością
                source = "coasting"

            # Utwórz obiekt wyniku i opublikuj go (thread-safe)
            result = FusedPosition(
                lat=lat, lon=lon, accuracy=accuracy,
                vx=vx, vy=vy, speed=speed,
                heading=heading_deg,
                source=source,
            )

            with self._lock:
                self._result = result

            time.sleep(sleep_dt)

    # ── Odczyt GNSS z procesu potomnego ───────────────────────────────────────

    def _run_gnss(self) -> None:
        """
        Uruchamia locator_gnss.py jako subprocess i czyta jego wyjście
        znak po znaku. Parsuje linie w formacie:
          GNSS OK | USED G=8 E=4 R=0 C=0 | lat=50.123456 lon=20.654321 hAcc=2.50m

        W razie błędu procesu — restart po 3 sekundach (pętla nieskończona).
        """
        cmd = ["python3", "-u", self._gnss_script,
               "--port", self._gnss_port]
        while True:
            try:
                self._gnss_proc = subprocess.Popen(
                    cmd, stdout=subprocess.PIPE, stderr=None,
                    encoding="utf-8", errors="replace",
                )
                for raw_line in self._gnss_proc.stdout:
                    # Usunięcie kodów kolorów ANSI
                    line = re.sub(r'\033\[[0-9;]*[mK]', '', raw_line).strip()
                    # Parsuj tylko linie GNSS zawierające współrzędne
                    if line.startswith("GNSS ") and "lat=" in line:
                        print(line, flush=True)   # przekaż do print_display.py
                        self._parse_gnss_line(line)
            except Exception as e:
                logger.warning(f"Błąd procesu GNSS: {e} — ponowna próba za 3s")
            time.sleep(3.0)

    def _parse_gnss_line(self, line: str) -> None:
        """
        Parsuje jednoliniowe wyjście GNSS i tworzy obiekt Position.

        Obsługa stanów bezpieczeństwa GNSS:
          JAMMED / SPOOF_SUSPECT / SPOOFED → odrzucenie (filtr ignoruje GNSS)
          DEGRADED → accuracy pomnożone ×3 (mniejsze zaufanie do pomiaru)
          OK → normalna dokładność (minimum 1.0 m)
        """
        # Wyodrębnienie stanu: GNSS OK / GNSS DEGRADED / GNSS JAMMED itp.
        state_match = re.match(r'GNSS (\w+)', line)
        state = state_match.group(1) if state_match else "UNKNOWN"

        # Stany niebezpieczne — GNSS może być zakłócany lub sfałszowany
        if state in ("JAMMED", "SPOOF_SUSPECT", "SPOOFED"):
            with self._gnss_lock:
                self._gnss_pos = None   # filtr nie użyje danych GNSS
            return

        # Wyodrębnienie współrzędnych i dokładności z tekstu linii
        try:
            lat  = float(re.search(r'lat=([\d.]+)',  line).group(1))
            lon  = float(re.search(r'lon=([\d.]+)',  line).group(1))
            hacc = float(re.search(r'hAcc=([\d.]+)', line).group(1))
        except (AttributeError, ValueError):
            return   # nieprawidłowy format — ignoruj linię

        # Dla stanu DEGRADED zwiększamy szum pomiarowy (×3), aby filtr
        # mniej ufał temu pomiarowi. Dla normalnego stanu — minimum 1.0 m.
        # max(hacc, 1.0) - blad GNSS bedzie zawsze minimum 1 metr
        accuracy = hacc * 3.0 if state == "DEGRADED" else max(hacc, 1.0)
        source   = "gnss_degraded" if state == "DEGRADED" else "gnss"

        with self._gnss_lock:
            self._gnss_pos = Position(
                lat=lat, lon=lon, accuracy=accuracy,
                source=source, timestamp=time.time()
            )

    # ── Rozgrzewka AHRS (zastępuje kalibrację biasu IMU) ─────────────────────

    def init_madgwick(self) -> None:
        """
        Rozgrzewka filtra Madgwicka — podaje mu AHRS_WARMUP_SAMPLES odczytów
        IMU, aby kwaternion orientacji skonwergował do poprawnej wartości.

        Zastępuje wcześniejszą kalibrację biasu (ax_bias, ay_bias, gz_bias).
        Kalibracja samych czujników (bias, hard/soft iron) jest teraz
        wykonywana osobno — przez calibration.json i locator_imu.

        Po rozgrzewce filtr Madgwicka zna orientację urządzenia w 3D
        i może poprawnie obracać przyspieszenie do układu ENU.

        Urządzenie powinno być w miarę nieruchome podczas rozgrzewki,
        aby filtr mógł ustalić kierunek grawitacji i pola magnetycznego.
        """
        logger.info(f"Kalman: rozgrzewka AHRS — {AHRS_WARMUP_SAMPLES} próbek, "
                    f"nie ruszaj urządzenia...")
        print(f"[KALMAN] Rozgrzewka AHRS (~{AHRS_WARMUP_SAMPLES // 100}s) "
              f"— nie ruszaj urządzenia...")

        count = 0
        while count < AHRS_WARMUP_SAMPLES and self._running:
            reading = self._imu.get_reading()
            if reading:
                self.f_madgwick.update(
                    reading.ax, reading.ay, reading.az,
                    reading.gx, reading.gy, reading.gz,
                    reading.mx, reading.my, reading.mz,
                )
                count += 1
            time.sleep(0.01)   # ~100 Hz

        if count >= AHRS_WARMUP_SAMPLES:
            self.madgwick_ready = True
            heading, pitch, roll = self.f_madgwick.euler_angles()
            logger.info(f"Kalman: AHRS gotowy — heading={heading:.1f}° "
                        f"pitch={pitch:.1f}° roll={roll:.1f}°")
            print(f"[KALMAN] AHRS gotowy: heading={heading:.1f}°  "
                  f"pitch={pitch:.1f}°  roll={roll:.1f}° — start filtru.")
        else:
            logger.warning("Kalman: IMU niedostępne — AHRS nie uruchomiony")
            print("[KALMAN] IMU niedostępne — AHRS nie uruchomiony.")

    # ── Krok predykcji filtru Kalmana ─────────────────────────────────────────

    def predict_step(self, ax_enu: float, ay_enu: float, dt: float) -> None:
        """
        state vector is updated using the kinematic equations to project where the pedestrian will be based on time and IMU acceleration.
        X_k+1 = A_k * X_k + B_k * u_k + eps_k
        Krok predykcji — propaguje wektor stanu [x, y, vx, vy]
        do przodu o dt sekund.

        Parametry:
          ax_enu : przyspieszenie liniowe w kierunku wschodnim (East), m/s²
                   — już w układzie ENU, po odjęciu grawitacji
          ay_enu : przyspieszenie liniowe w kierunku północnym (North), m/s²
                   — już w układzie ENU, po odjęciu grawitacji
          dt     : krok czasowy w sekundach

        ZMIANA vs. poprzednia wersja:
          Przyspieszenie wchodzi już obrócone do ENU i bez grawitacji
          (dzięki filtrowi Madgwicka), więc:
            • nie ma rotacji body→ENU wewnątrz predict_step(),
            • nie ma zależności od kąta θ w wektorze stanu,
            • jakobian F jest stały (nie zależy od stanu) → filtr jest liniowy,
            • macierz F nie ma piątej kolumny z pochodnymi po θ.

        Model ruchu (równania kinematyczne):
            x  ← x  + vx·dt + ½·ax_enu·dt²
            y  ← y  + vy·dt + ½·ay_enu·dt²
            vx ← vx + ax_enu·dt
            vy ← vy + ay_enu·dt

        Propagacja kowariancji:
          P ← F · P · Fᵀ + Q
        """
        dt2 = dt * dt
        
        # Macierz przejścia stanu A (4×4) — model liniowy
        # Wynika z równań kinematyki punktu materialnego dla ruchu jednostajnie zmiennego.
        # Przyspieszenie jest nam dane z IMU, filtr nie musi go estymowac i ufa, co IMU powie
        A = np.array([
            [1, 0, dt,  0],    # x  zależy od vx
            [0, 1,  0, dt],    # y  zależy od vy
            [0, 0,  1,  0],    # vx — stałe (korekta przez Q)
            [0, 0,  0,  1],    # vy — stałe (korekta przez Q)
        ])

        # do przewidywanej pozycji i prędkości ręcznie dodawane jest zmierzone przyspieszenie z akcelerometru (po obrocie do układu ENU)
        # Propagacja wektora stanu (równania kinematyczne)
        # X_k+1 = A_k * X_k + B_k * u_k + error
        # A_k * X_k = [x + vx·dt, y + vy·dt, vx, vy]
        # B_k * u_k = [½·ax·dt², ½·ay·dt², ax·dt, ay·dt]
        
        # Poniżej: X_k+1 = A_k * X_k + B_k * u_k
        # macierz X_k+1 (where we are)
        self._x[0] += self._x[2] * dt + 0.5 * ax_enu * dt2      # x  += vx·dt + ½·ax·dt²
        self._x[1] += self._x[3] * dt + 0.5 * ay_enu * dt2      # y  += vy·dt + ½·ay·dt²
        self._x[2] += ax_enu * dt                               # vx += ax·dt
        self._x[3] += ay_enu * dt                               # vy += ay·dt

        # Macierz szumu procesowego Q (diagonalna)
        # Skalowana przez dt — dłuższy krok = większa niepewność
        Q = np.diag([
            Q_POS * dt2,     # szum pozycji x
            Q_POS * dt2,     # szum pozycji y
            Q_VEL * dt,      # szum prędkości vx
            Q_VEL * dt,      # szum prędkości vy
        ])

        # Propagacja macierzy kowariancji: P = A · P · Aᵀ + Q
        self._P = A @ self._P @ A.T + Q

    # ── Krok korekcji pozycji (measurement update) ────────────────────────────

    def update_step(self, x_m: float, y_m: float, accuracy: float) -> None:
        """
        is corrected when a new measurement arrives from UWB, GNSS, WiFi, or Cell, shifting the values toward the observed position.
        Korekcja stanu na podstawie pomiaru pozycji (x_m, y_m) w metrach ENU.

        Jest to standardowy krok aktualizacji filtru Kalmana:
          1. Oblicz innowację (resztę): y = z − H·x  (różnica pomiaru od predykcji)
          2. Oblicz macierz innowacji:  S = H·P·Hᵀ + R
          3. Oblicz wzmocnienie Kalmana: K = P·Hᵀ·S⁻¹
          4. Skoryguj stan:             x ← x + K·y
          5. Zaktualizuj kowariancję:   P ← (I − K·H)·P

        Macierz obserwacji H:
          Pomiar dostarcza tylko pozycję (x, y), nie prędkość.
          H = [[1, 0, 0, 0],    — obserwujemy x
               [0, 1, 0, 0]]    — obserwujemy y

        Macierz szumu pomiarowego R:
          R = accuracy² · I₂  — im mniejsza accuracy, tym większe zaufanie
          do pomiaru. Dzięki temu filtr automatycznie waży źródła:
          UWB (acc ~0.3m) dostaje dużą wagę, WiFi (acc ~30m) — małą.
        """
        # Macierz obserwacji — obserwujemy tylko pozycję (x, y)
        # Macierz obserwacji H pełni rolę łącznika między wektorami stanu systemu a wektorami pomiaru. 
        # Ma ona tyle wierszy, ile jest mierzonych wartości przez czujniki ( [x,y]  ) 
        # i tyle kolumn, ile jest elementów w stanie, które filtr stara się śledzić ( [x,y,vx,vy] ):
        # Jedynki w pierwszej i drugiej kolumnie mówią: „Interesuje nas pozycja x i y”
        # Zera w trzeciej i czwartej kolumnie mówią: „Zignoruj prędkości vx i vy, bo ten sensor ich nie mierzy”.
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ])

        # Macierz szumu pomiarowego — accuracy² na diagonali
        # macierz kowariancji szumu pomiarowego
        # matematyczny sposób na powiedzenie filtrowi, jak bardzo ma (nie) ufać konkretnemu czujnikowi w danej chwili
        """
        Filtr Kalmana musi zdecydować, czy bardziej wierzyć swoim własnym przewidywaniom (modelowi ruchu), czy nowym danym z czujnika (np. UWB lub GNSS). 
        Macierz $R$ dostarcza informacji o błędach tych czujników:
        - Małe wartości w $R$: Oznaczają, że czujnik jest bardzo precyzyjny. Filtr "uwierzy" mu i mocno skoryguje pozycję w stronę odczytu.
        - Duże wartości w $R$: Oznaczają, że czujnik jest niepewny (zaszumiony). Filtr potraktuje taki odczyt z rezerwą, stawiając wyżej płynność ruchu wynikającą z predykcji.
        """
        R = np.eye(2) * (accuracy ** 2)

        # Wektor pomiaru
        z = np.array([x_m, y_m])

        # Innowacja (residuum): różnica między pomiarem a predykcją
        y = z - H @ self._x

        # Macierz kowariancji innowacji
        S = H @ self._P @ H.T + R
        
        # Kalman Gain
        # Wzmocnienie Kalmana — określa, jak bardzo pomiar zmieni stan
        # Duże K = filtr mocno koryguje (duże zaufanie do pomiaru)
        # Małe K = filtr mało koryguje (duże zaufanie do predykcji)
        K = self._P @ H.T @ np.linalg.inv(S)

        # Korekcja wektora stanu
        #self._x = self._x + K @ y
        self._x = self._x + K @ (z - H @ self._x)

        # Korekcja macierzy kowariancji — niepewność maleje po korekcji
        self._P = (np.eye(4) - K @ H) @ self._P

    # ── ZUPT (Zero Velocity Update) ──────────────────────────────────────────

    def apply_zupt(self) -> None:
        """
        Korekta zerowej prędkości (ZUPT) — stosowana, gdy algorytm
        wykryje, że urządzenie jest nieruchome.

        Traktujemy vx=0 i vy=0 jako pomiar z bardzo małym szumem (R=0.001).
        Filtr Kalmana koryguje prędkość w stronę zera, eliminując
        akumulowany dryf z całkowania przyspieszenia.

        Jest to kluczowa technika w nawigacji inercyjnej — bez ZUPT
        błąd pozycji rośnie kwadratowo w czasie (podwójne całkowanie
        szumu akcelerometru).

        Uwaga: ZUPT nie zmienia pozycji (x, y) — koryguje wyłącznie
        składowe prędkości (vx, vy).
        """
        # Macierz obserwacji - obserwujemy prędkości vx i vy
        H = np.array([
            [0, 0, 1, 0],   # obserwacja vx
            [0, 0, 0, 1],   # obserwacja vy
        ])
        # Macierz szumu pomiarowego
        # Bardzo mały szum — duże zaufanie, że prędkość = 0
        R = np.eye(2) * 0.001

        # „Pomiar": prędkość = [0, 0]
        z = np.array([0.0, 0.0])
        y = z - H @ self._x         # innowacja

        # Standardowy krok korekcji Kalmana
        S = H @ self._P @ H.T + R
        K = self._P @ H.T @ np.linalg.inv(S)

        self._x = self._x + K @ y
        self._P = (np.eye(4) - K @ H) @ self._P

    # ── Pobranie świeżych pomiarów z lokatorów ────────────────────────────────

    def fresh_fixes(self) -> list:
        """
        Zwraca listę krotek (nazwa, Position) dla wszystkich lokalizatorów,
        które mają nowy pomiar (nieużyty jeszcze w korekcji)

        Mechanizm filtrowania:
          1. Sprawdź, czy timestamp pomiaru jest nowszy niż ostatnio użyty
          2. Sprawdź, czy pomiar nie jest zbyt stary (MAX_FIX_AGE)

        Dzięki temu każdy pomiar jest użyty dokładnie raz, a przeterminowane
        odczyty (np. stare WiFi) nie zaburzają filtra.
        """
        fixes = []
        # Lista kandydatów — odpytaj każdy lokator o aktualną pozycję
        candidates = [
            ("uwb",  self._uwb.get_position()),
            ("gnss", self._gnss_pos),
            ("wifi", self._wifi.get_position()),
            ("cell", self._cell.get_position()),
        ]
        for name, pos in candidates:
            if pos is None:
                continue   # lokator nie ma pomiaru — pomiń
            last_used = self.last_pos_used.get(name, 0.0)
            pos_ts = pos.timestamp if isinstance(pos.timestamp, float) \
                     else pos.timestamp.timestamp()
            # Użyj tylko jeśli pomiar jest nowy i nie za stary.
            # Etykieta źródła może zawierać sufiks (np. "cell|2600MHz") — bierzemy
            # człon przed znakiem '|', aby trafić w klucz słownika ("cell").
            source_key = pos.source.split("|")[0]
            max_age = MAX_FIX_AGE.get(source_key, 10.0)
            if pos_ts > last_used and (time.time() - pos_ts) < max_age:
                fixes.append((name, pos))
        return fixes


# ── Uruchomienie samodzielne ─────────────────────────────────────────────────

if __name__ == "__main__":
    import signal

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s  [%(levelname)s]  %(message)s",
                        datefmt="%H:%M:%S")

    # Parsowanie argumentów wiersza poleceń
    ap = argparse.ArgumentParser(description="Wieloźródłowy filtr Kalmana")
    ap.add_argument("--anchors",     default="uwb_anchors.json")
    ap.add_argument("--uwb-port",    default="/dev/ttyACM0")
    ap.add_argument("--gnss-port",   default="/dev/ttyACM1")
    ap.add_argument("--gnss-script", default="/home/wojtek/locator_gnss.py")
    ap.add_argument("--cell-port",   default="/dev/ttyUSB2")
    args = ap.parse_args()

    # Utworzenie i uruchomienie filtru
    kf = KalmanFilter(
        anchors_file=args.anchors,
        uwb_port=args.uwb_port,
        gnss_port=args.gnss_port,
        gnss_script=args.gnss_script,
        cell_port=args.cell_port,
    )
    kf.start()

    # Obsługa sygnału SIGTERM — wysyłanego przez print_display.py przy zamykaniu.
    # Zapewnia poprawne zamknięcie procesu GNSS i wątków lokatorów.
    def _handle_sigterm(signum, frame):
        kf.stop()
        sys.exit(0)
    signal.signal(signal.SIGTERM, _handle_sigterm)

    print("\n[KALMAN] Działa — Ctrl+C aby zatrzymać\n")
    print(f"{'Źródło':>20}  {'Lat':>12}  {'Lon':>12}  {'Dok.':>8}  {'Prędk.':>8}")
    print("-" * 70)

    try:
        # Pętla główna — odczytuj wynik filtru co 0.1s i wyświetlaj na konsoli
        while True:
            result = kf.get_result()
            if result:
                print(
                    f"[KALMAN] {result.source:<16} "
                    f"lat={result.lat:.7f}  "
                    f"lon={result.lon:.7f}  "
                    f"acc={result.accuracy:.2f}m  "
                    f"speed={result.speed:.2f}m/s  "
                    f"hdg={result.heading:.1f}deg",
                    end="\r", flush=True
                )
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\nZatrzymywanie...")
        kf.stop()