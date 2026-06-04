#!/usr/bin/python

import re
import sys
import math
import os
import signal
import atexit
import pygame
import subprocess
import threading

pygame.init()
screen = pygame.display.set_mode((800, 480))

# TTF font z pelnym Latin-Extended (polskie znaki) -- fallback do SysFont
def _load_font(size):
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
        "/usr/share/fonts/truetype/freefont/FreeMono.ttf",
    ]
    for path in candidates:
        if os.path.exists(path):
            return pygame.font.Font(path, size)
    return pygame.font.SysFont("monospace", size)

font = _load_font(20)

# -- Scrolling text ------------------------------------------------------------

class ScrollText:
    """Przewija tekst poziomo gdy jest szerszy niz dostepna przestrzen."""
    PAUSE_FRAMES = 20   # ile klatek stac w miejscu przed i po przewinieciu
    SPEED        = 2    # piksele na klatke

    def __init__(self):
        self._offset     = 0
        self._pause      = self.PAUSE_FRAMES
        self._last_text  = ""
        self._at_end     = False

    def draw(self, surface, font, text, color, x, y, max_width):
        if text != self._last_text:          # reset przy zmianie tekstu
            self._offset    = 0
            self._pause     = self.PAUSE_FRAMES
            self._last_text = text
            self._at_end    = False

        text_surf = font.render(text, True, color)
        text_w    = text_surf.get_width()

        if text_w <= max_width:              # miesci sie -- bez przewijania
            self._offset = 0
            surface.blit(text_surf, (x, y))
            return

        clip = pygame.Rect(x, y, max_width, text_surf.get_height())
        surface.set_clip(clip)
        surface.blit(text_surf, (x - self._offset, y))
        surface.set_clip(None)

        if self._pause > 0:
            self._pause -= 1
        elif not self._at_end:
            self._offset += self.SPEED
            if self._offset >= text_w - max_width:
                self._offset = text_w - max_width
                self._at_end = True
                self._pause  = self.PAUSE_FRAMES * 2
        else:
            self._offset = 0
            self._at_end = False
            self._pause  = self.PAUSE_FRAMES

_cell_addr_scroll = ScrollText()

# ── Output panels ─────────────────────────────────────────────────────────────
output1 = []   # GNSS
output2 = []   # WiFi
output3 = []   # UWB
output4 = []   # IMU
output5 = []   # Kalman fused
output6 = []   # Cell ID pozycja
output7 = []   # Cell ID opis lokalizacji

# Znaczniki czasu ostatniej aktualizacji paneli
import time as _time

_last_update = {'gnss': 0.0, 'wifi': 0.0, 'uwb': 0.0, 'imu': 0.0, 'kalman': 0.0, 'cell': 0.0}

_STALE = {'gnss': 10.0, 'wifi': 35.0, 'uwb': 5.0, 'imu': 5.0, 'kalman': 10.0, 'cell': 25.0}

def _fresh(key): return (_time.time() - _last_update[key]) < _STALE[key]

# ── Porty — jedyne miejsce gdzie trzeba je zmieniać ──────────────────────────
CELL_PORT = "/dev/ttyUSB3"   # Quectel modem (port AT commands)

# Auto-detekcja portow ttyACM* przez udevadm. Dzieki temu GNSS i UWB
# moga byc podlaczone do dowolnych portow USB w dowolnej kolejnosci.
def detect_acm_ports(default_gnss="/dev/ttyACM0", default_uwb="/dev/ttyACM1"):
    """
    Skanuje wszystkie /dev/ttyACM* i przypisuje port wedlug ID_MODEL z udevadm:
      - "Challenger_2040_UWB"   -> UWB
      - "u-blox_GNSS_receiver"  -> GNSS
    Jesli ktoregos urzadzenia nie da sie znalezc, wraca do wartosci domyslnej.
    """
    gnss = uwb = None

    try:
        candidates = sorted(p for p in os.listdir("/dev") if p.startswith("ttyACM"))
    except OSError:
        candidates = []

    for name in candidates:
        port = f"/dev/{name}"
        try:
            out = subprocess.check_output(
                ["udevadm", "info", port],
                stderr=subprocess.DEVNULL,
                encoding="utf-8",
                errors="replace",
            )
        except (subprocess.CalledProcessError, FileNotFoundError):
            continue

        # Wyciagnij wartosc ID_MODEL=... (ale nie ID_MODEL_ENC ani ID_MODEL_ID)
        model = ""
        for line in out.splitlines():
            m = re.search(r"ID_MODEL=(\S+)", line)
            if m:
                model = m.group(1)
                break

        if "Challenger_2040_UWB" in model:
            uwb = port
        elif "u-blox_GNSS_receiver" in model:
            gnss = port

    if gnss is None:
        print(f"[WARN] Nie wykryto u-blox GNSS, uzywam domyslnego {default_gnss}",
              file=sys.stderr)
        gnss = default_gnss
    if uwb is None:
        print(f"[WARN] Nie wykryto Challenger 2040 UWB, uzywam domyslnego {default_uwb}",
              file=sys.stderr)
        uwb = default_uwb

    print(f"[INFO] GNSS port: {gnss}")
    print(f"[INFO] UWB  port: {uwb}")
    return gnss, uwb

GNSS_PORT, UWB_PORT = detect_acm_ports()

# ── Map state ─────────────────────────────────────────────────────────────────
current_lat  = [None]
current_lon  = [None]
VIEW_DASHBOARD = 0
VIEW_MAP       = 1
current_view   = VIEW_DASHBOARD

TILE_SIZE = 256
TILE_DIR  = "/home/wojtek/tiles"
ZOOM      = 17

# ── ANSI stripper ─────────────────────────────────────────────────────────────

def strip_ansi(text):
    return re.sub(r'\033\[[0-9;]*[mK]', '', text)

# ── Line router ───────────────────────────────────────────────────────────────

def append_line(output_list, line, max_lines=10):
    output_list.append(line)
    if len(output_list) > max_lines:
        output_list.pop(0)

def route_line(line: str):
    """Route a stdout line from kalman_filter.py to the correct panel."""
    now = _time.time()

    if line.startswith("[KALMAN]"):
        if output5:
            output5[-1] = line
        else:
            output5.append(line)
        _last_update["kalman"] = now

    elif line.startswith("[UWB]"):
        if output3:
            output3[-1] = line
        else:
            output3.append(line)
        _last_update["uwb"] = now

    elif line.startswith("GNSS ") or (("lat=" in line or "lon=" in line) and "hAcc" in line):
        if output1:
            output1[-1] = line
        else:
            output1.append(line)
        _last_update["gnss"] = now

    elif line.startswith("[WIFI"):
        append_line(output2, line, max_lines=2)
        _last_update["wifi"] = now

    elif line.startswith("[CELL_ADDR]"):
        if output7:
            output7[-1] = line
        else:
            output7.append(line)
        _last_update["cell"] = now

    elif line.startswith("[CELL]"):
        if output6:
            output6[-1] = line
        else:
            output6.append(line)

    elif len(line) > 1 and (line[0] in "+-") and line[1].isdigit():
        append_line(output4, line, max_lines=5)
        _last_update["imu"] = now

# ── Kalman subprocess — launch and read ──────────────────────────────────────

_kalman_proc = None   # module-level so _cleanup() can always reach it


def _cleanup():
    """Terminate the kalman_filter.py subprocess tree on any exit."""
    if _kalman_proc is not None and _kalman_proc.poll() is None:
        _kalman_proc.terminate()
        try:
            _kalman_proc.wait(timeout=3)
        except Exception:
            _kalman_proc.kill()


atexit.register(_cleanup)

# SIGTERM (e.g. `kill <pid>`) triggers sys.exit() which fires atexit handlers.
signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))


def run_kalman(path, args):
    global _kalman_proc
    _kalman_proc = subprocess.Popen(
        ["python3", "-u", path] + args,
        stdout=subprocess.PIPE,
        stderr=None,
        encoding="utf-8",   # dekoduj cale linie jako UTF-8, nie bajt po bajcie
        errors="replace",
    )
    for raw_line in _kalman_proc.stdout:
        line = strip_ansi(raw_line).strip()
        if line:
            route_line(line)

# ── Map helpers ───────────────────────────────────────────────────────────────

def parse_position_kalman():
    for line in reversed(output5):
        try:
            lat_val = lon_val = None
            for p in line.split():
                if p.startswith("lat="):
                    lat_val = float(p.split("=")[1])
                if p.startswith("lon="):
                    lon_val = float(p.split("=")[1])
            if lat_val and lon_val:
                current_lat[0] = lat_val
                current_lon[0] = lon_val
                return
        except (ValueError, IndexError):
            pass

def deg2tile_float(lat, lon, zoom):
    n = 2 ** zoom
    x = (lon + 180) / 360 * n
    y = (1 - math.log(math.tan(math.radians(lat)) + 1 / math.cos(math.radians(lat))) / math.pi) / 2 * n
    return x, y

tile_cache = {}

def load_tile(zoom, x, y):
    key = (zoom, x, y)
    if key in tile_cache:
        return tile_cache[key]
    path = f"{TILE_DIR}/{zoom}/{x}/{y}.png"
    if os.path.exists(path):
        try:
            tile = pygame.image.load(path)
            tile_cache[key] = tile
            return tile
        except Exception:
            return None
    return None

def draw_map(surface, lat, lon, zoom, rect):
    cx, cy = deg2tile_float(lat, lon, zoom)
    center_x = rect.x + rect.width  // 2
    center_y = rect.y + rect.height // 2
    tx_min = int(cx - rect.width  / (2 * TILE_SIZE)) - 1
    tx_max = int(cx + rect.width  / (2 * TILE_SIZE)) + 1
    ty_min = int(cy - rect.height / (2 * TILE_SIZE)) - 1
    ty_max = int(cy + rect.height / (2 * TILE_SIZE)) + 1
    for tx in range(tx_min, tx_max + 1):
        for ty in range(ty_min, ty_max + 1):
            px = center_x + int((tx - cx) * TILE_SIZE)
            py = center_y + int((ty - cy) * TILE_SIZE)
            if px >= rect.right or px + TILE_SIZE <= rect.left:
                continue
            if py >= rect.bottom or py + TILE_SIZE <= rect.top:
                continue
            tile = load_tile(zoom, tx, ty)
            if tile:
                surface.blit(tile, (px, py))
            else:
                pygame.draw.rect(surface, (60, 60, 60), (px, py, TILE_SIZE, TILE_SIZE))
                pygame.draw.rect(surface, (80, 80, 80), (px, py, TILE_SIZE, TILE_SIZE), 1)
    pygame.draw.circle(surface, (255, 255, 255), (center_x, center_y), 10)
    pygame.draw.circle(surface, (255, 0,   0),   (center_x, center_y),  8)

# ── Start single Kalman subprocess ────────────────────────────────────────────

threading.Thread(
    target=run_kalman,
    args=(
        "/home/wojtek/kalman_filter.py",
        ["--uwb-port",  UWB_PORT,
         "--gnss-port", GNSS_PORT,
         "--cell-port", CELL_PORT,
         "--anchors",   "/home/wojtek/uwb_anchors.json"],
    ),
    daemon=True,
).start()

# ── Colors and constants ──────────────────────────────────────────────────────

switch_btn   = pygame.Rect(690, 182, 110, 24)
COLOR_GNSS   = (  0, 255,   0)
COLOR_WIFI   = (255, 255,   0)
COLOR_UWB    = (255, 128,   0)
COLOR_CELL   = (255,  80,  80)
COLOR_KALMAN = (255,   0, 255)
COLOR_IMU    = (  0, 200, 255)
COLOR_DIM    = (100, 100, 100)
COLOR_DIV    = (100, 100, 100)
IMU_HEADER   = "Accel-X    Accel-Y    Accel-Z      Gyro-X     Gyro-Y     Gyro-Z"

# ── Main loop ─────────────────────────────────────────────────────────────────

try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()   # triggers atexit → _cleanup()
            if event.type == pygame.MOUSEBUTTONDOWN:
                if switch_btn.collidepoint(event.pos):
                    current_view = VIEW_MAP if current_view == VIEW_DASHBOARD else VIEW_DASHBOARD

        screen.fill((0, 0, 0))

        if current_view == VIEW_DASHBOARD:

            # Panel 1 — GNSS (green)                              y=10
            gnss_ok = output1 and _fresh("gnss")
            line = output1[-1] if gnss_ok else "Brak GNSS..."
            screen.blit(font.render(line, True, COLOR_GNSS if gnss_ok else COLOR_DIM), (10, 10))
            pygame.draw.line(screen, COLOR_DIV, (0, 35), (800, 35), 1)

            # Panel 2 — WiFi (yellow) — jedna linia               y=45
            wifi_ok = output2 and _fresh("wifi")
            line = output2[-1] if wifi_ok else "Brak WiFi..."
            screen.blit(font.render(line, True, COLOR_WIFI if wifi_ok else COLOR_DIM), (10, 45))
            pygame.draw.line(screen, COLOR_DIV, (0, 68), (800, 68), 1)

            # Panel 3 — UWB (orange)                              y=78
            uwb_ok = output3 and _fresh("uwb")
            line = output3[-1] if uwb_ok else "Brak UWB..."
            screen.blit(font.render(line, True, COLOR_UWB if uwb_ok else COLOR_DIM), (10, 78))
            pygame.draw.line(screen, COLOR_DIV, (0, 100), (800, 100), 1)

            # Panel 6 — Cell ID pozycja (czerwony)                y=110
            cell_ok = output6 and _fresh("cell")
            line = output6[-1] if cell_ok else "Brak Cell ID..."
            screen.blit(font.render(line, True, COLOR_CELL if cell_ok else COLOR_DIM), (10, 110))

            # Panel 7 — Cell ID opis lokalizacji (bez prefiksu, przesuwny) y=132
            raw_addr = output7[-1] if (cell_ok and output7) else ""
            if raw_addr:
                addr_text = raw_addr.split("]", 1)[-1].lstrip()
                _cell_addr_scroll.draw(
                    screen, font, addr_text, COLOR_CELL,
                    x=10, y=132, max_width=790
                )
            pygame.draw.line(screen, COLOR_DIV, (0, 154), (800, 154), 1)

            # Panel 5 — Kalman fused (magenta)                    y=164
            kalman_ok = output5 and _fresh("kalman")
            line = output5[-1] if kalman_ok else "Brak fixu z filtra Kalmana..."
            screen.blit(font.render(line, True, COLOR_KALMAN if kalman_ok else COLOR_DIM), (10, 164))

            # Heading indicator (parsed from [KALMAN] line)       y=186
            if kalman_ok:
                for part in output5[-1].split():
                    if part.startswith("hdg="):
                        try:
                            hdg_val = float(part[4:].replace("°", "").replace("deg", "").strip())
                            dirs = ["N","NE","E","SE","S","SW","W","NW"]
                            compass = dirs[int((hdg_val + 22.5) / 45) % 8]
                            hdg_line = f"  Heading: {hdg_val:.1f}  {compass}"
                            screen.blit(font.render(hdg_line, True, COLOR_KALMAN), (10, 186))
                        except ValueError:
                            pass
                        break
            pygame.draw.line(screen, COLOR_DIV, (0, 208), (800, 208), 1)

            # IMU header                                           y=218
            screen.blit(font.render(IMU_HEADER, True, (150, 150, 150)), (10, 218))
            pygame.draw.line(screen, (60, 60, 60), (0, 240), (800, 240), 1)

            # Panel 4 — IMU rows (cyan, newest at top)            y=250+
            if _fresh("imu"):
                for i, line in enumerate(reversed(output4)):
                    screen.blit(font.render(line, True, COLOR_IMU), (10, 250 + i * 22))
            else:
                screen.blit(font.render("Brak IMU...", True, COLOR_DIM), (10, 250))

            # Switch button (gorny prawy rog, przy linii GNSS)
            pygame.draw.rect(screen, (60, 60, 60), switch_btn)
            screen.blit(font.render("MAP",  True, (255, 255, 255)), (switch_btn.x + 25, switch_btn.y + 4))

        elif current_view == VIEW_MAP:
            parse_position_kalman()

            if current_lat[0] and current_lon[0]:
                map_rect = pygame.Rect(0, 40, 800, 440)
                screen.set_clip(map_rect)
                draw_map(screen, current_lat[0], current_lon[0], ZOOM, map_rect)
                screen.set_clip(None)

                # Parse source and heading from latest [KALMAN] line
                src_str = ""
                hdg_str = ""
                if output5:
                    for part in output5[-1].split():
                        if part.startswith("hdg="):
                            try:
                                hdg_val = float(part[4:].replace("deg","").replace("°","").strip())
                                dirs = ["N","NE","E","SE","S","SW","W","NW"]
                                hdg_str = f"  {hdg_val:.1f}° {dirs[int((hdg_val+22.5)/45)%8]}"
                            except ValueError:
                                pass
                    # source is the token right after [KALMAN]
                    parts = output5[-1].split()
                    if len(parts) >= 2:
                        src_str = parts[1]

                coord_txt = font.render(
                    f"[KALMAN] {src_str}  Lat: {current_lat[0]:.7f}  Lon: {current_lon[0]:.7f}{hdg_str}",
                    True, (255, 255, 255)
                )
                screen.blit(coord_txt, (10, 10))
            else:
                screen.blit(font.render("Brak fixu z filtra Kalmana...", True, (200, 200, 200)), (250, 220))

            # Switch button
            pygame.draw.rect(screen, (60, 60, 60), switch_btn)
            screen.blit(font.render("DASH", True, (255, 255, 255)), (switch_btn.x + 15, switch_btn.y + 4))

        pygame.display.flip()
        pygame.time.wait(100)

except KeyboardInterrupt:
    pygame.quit()
    sys.exit()   # triggers atexit → _cleanup()