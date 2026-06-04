#!/usr/bin/python

"""
Geolokacja za pomocą WiFi używając Google Geolocation API.

Skanuje najbliższe access pointy WiFi i odpytuje bazę Google
z ich BSSID w celu zmapowania na szacunkową pozycję.

Requirements:
    pip install requests
    sudo setcap cap_net_raw,cap_net_admin+eip /sbin/iwlist
"""

import os
import re
import json
import time
import logging
import threading
import subprocess
import requests
from dataclasses import dataclass, field
from typing import Optional

from position import Position

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  [%(levelname)s]  %(message)s",
    datefmt="%H:%M:%S"
)

# ── Konfiguracja ─────────────────────────────────────────────────────────────

GOOGLE_API_KEY   = ""
WIFI_INTERFACE   = "wlan0"
MIN_NETWORKS     = 2      # min ilość APs potrzebnych do fixu
ACCURACY_LIMIT   = 200    # odrzuć rezultaty gorsze niż ilość metrów
REQUEST_TIMEOUT  = 5      

# ── Zablokuj pozycjonowanie WiFi ──────-──────────────────────────────────────

DISABLE_FLAG = "/tmp/wifi.off"

# ── Skanowanie WiFi ─────────────────────────────────────────────────────────────

def scan_wifi(interface: str = WIFI_INTERFACE) -> list[dict]:
    """
    Skanuj najbliższe sieci WiFi używając iwlist.
    Zwraca listę {"macAddress": "AA:BB:CC:DD:EE:FF", "signalStrength": -65}

    Skrypt potrzebuje jednego z dwóch:
      - sudo, lub
      - sudo setcap cap_net_raw,cap_net_admin+eip /sbin/iwlist
    """
    try:
        result = subprocess.run(
            ["iwlist", interface, "scan"],
            capture_output=True,
            text=True,
            timeout=10,
        )
        if result.returncode != 0:
            logging.warning(f"iwlist error: {result.stderr.strip()}")
            return []

        networks = []
        current: dict = {}

        for line in result.stdout.splitlines():
            line = line.strip()

            if "Address:" in line:
                if current:
                    networks.append(current)
                mac = line.split("Address:")[-1].strip()
                current = {"macAddress": mac}

            elif "Signal level=" in line:
                m = re.search(r"Signal level=([-\d]+)", line)
                if m:
                    current["signalStrength"] = int(m.group(1))

            elif "ESSID:" in line:
                # Optional — not sent to API but useful for logging
                ssid = line.split("ESSID:")[-1].strip().strip('"')
                current["ssid"] = ssid

        if current.get("macAddress"):
            networks.append(current)

        # zostaw tylko te wpisy, które mają signal strength
        networks = [n for n in networks if "signalStrength" in n]

        logging.info(f"Skan WiFi znalazł {len(networks)} access points")
        for n in networks:
            logging.debug(f"  {n.get('ssid', '?'):30s}  {n['macAddress']}  {n['signalStrength']} dBm")

        return networks

    except FileNotFoundError:
        logging.error("Nie znaleziono iwlist — zainstaluj: sudo apt install wireless-tools")
        return []
    except subprocess.TimeoutExpired:
        logging.warning("WiFi scan timed out")
        return []
    except Exception as e:
        logging.warning(f"WiFi scan failed: {e}")
        return []

# ── Google Geolocation API ────────────────────────────────────────────────────

def query_google(networks: list[dict]) -> Optional[Position]:
    """
    POST wifi access points to Google Geolocation API.
    Returns Position or None if failed/insufficient data.
    """
    # wyślij tylko macAddress i signalStrength — pomiń ssid
    payload = {
        "wifiAccessPoints": [
            {"macAddress": n["macAddress"], "signalStrength": n["signalStrength"]}
            for n in networks
        ]
    }

    try:
        response = requests.post(
            f"https://www.googleapis.com/geolocation/v1/geolocate?key={GOOGLE_API_KEY}",
            json=payload,
            timeout=REQUEST_TIMEOUT,
        )

        if response.status_code != 200:
            logging.warning(f"Google API HTTP {response.status_code}: {response.text}")
            return None

        data = response.json()

        if "error" in data:
            logging.warning(f"Błąd Google API: {data['error'].get('message', data['error'])}")
            return None

        if "location" not in data:
            logging.warning("Odpowiedź Google API nie zawiera pola location")
            return None

        return Position(
            lat=data["location"]["lat"],
            lon=data["location"]["lng"],
            accuracy=float(data.get("accuracy", 999)),
            source="wifi",
        )

    except requests.exceptions.Timeout:
        logging.warning("Google Geolocation API timeout")
    except requests.exceptions.ConnectionError:
        logging.warning("Brak połączenia z internetem")
    except Exception as e:
        logging.warning(f"Google Geolocation request failed: {e}")

    return None

def get_position() -> Optional[Position]:
    """
    Main entry point. Scan WiFi and return position estimate, or None.

    Returns None if:
    - Fewer than MIN_NETWORKS access points found
    - Google API call fails
    - Accuracy is worse than ACCURACY_LIMIT
    """
    networks = scan_wifi()

    if len(networks) < MIN_NETWORKS:
        logging.info(f"WiFi: znaleziono tylko {len(networks)} APs, potrzeba {MIN_NETWORKS} — pomijam")
        return None

    pos = query_google(networks)

    if pos is None:
        return None

    if pos.accuracy > ACCURACY_LIMIT:
        logging.info(f"WiFi: accuracy {pos.accuracy:.0f}m przekracza limit {ACCURACY_LIMIT}m — discarding")
        return None

    logging.info(f"WiFi fix: {pos}")
    return pos

# ── Klasa WiFiLocator ───────────────────────────────────

SCAN_INTERVAL_S = 15
STALE_AFTER_S   = 30

class WiFiLocator:
    """
    Wraps get_position() in a background thread.
    Prints a dashboard line after every scan AND exposes get_position()
    for the Kalman filter — both work simultaneously.

    Usage:
        loc = WiFiLocator()
        loc.start()
        pos = loc.get_position()   # → Position or None
        loc.stop()
    """

    def __init__(self, interface: str = WIFI_INTERFACE,
                 scan_interval: float = SCAN_INTERVAL_S):
        self._interface     = interface
        self._scan_interval = scan_interval

        self._position: Optional[Position] = None
        self._lock    = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._running = False

    def start(self) -> None:
        self._running = True
        self._thread  = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        logging.info("WiFiLocator: uruchomiony")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=15.0)
        logging.info("WiFiLocator: stopped")

    def get_position(self) -> Optional[Position]:
        """Zwróć ostatnią pozycję, lub None jeżeli brak pozycji"""
        with self._lock:
            if self._position is None:
                return None
            if time.time() - self._position.timestamp > STALE_AFTER_S:
                return None
            return self._position

    def _run(self) -> None:
        while self._running:
            if os.path.exists(DISABLE_FLAG):
                with self._lock:
                    self._position = None
                self._print_dashboard(None)
                logging.info("[WIFI] Wyłączone (flag file present)")
            else:
                pos = get_position()
                with self._lock:
                    self._position = pos
                self._print_dashboard(pos)
            time.sleep(self._scan_interval)

    def _print_dashboard(self, pos: Optional[Position]) -> None:
        print("---", flush=True)
        if pos:
            import datetime
            ts = pos.timestamp.timestamp() if isinstance(pos.timestamp, datetime.datetime) else pos.timestamp
            age = time.time() - ts
            print(
                f"[WIFI]  "
                f"lat={pos.lat:.7f}  lon={pos.lon:.7f}  "
                f"accuracy=~{pos.accuracy:.0f}m  "
                f"age={age:.0f}s"
            )
            #print(f"Google Maps: https://maps.google.com/?q={pos.lat},{pos.lon}")
        else:
            print("[WIFI]  Brak dostępnej pozycji")


# ── Standalone ────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("Skanowanie WiFi i odpytywanie Google Geolocation...\n", flush=True)
    loc = WiFiLocator()
    loc.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nZatrzymywanie...")
        loc.stop()
