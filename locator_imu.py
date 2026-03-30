#!/usr/bin/python

"""
https://github.com/sparkfun/qwiic_ism330dhcx_py
https://robu.in/wp-content/uploads/2024/01/1848625-ISM330DHCXMMC5983MA-1.pdf

IMU reader — ISM330DHCX accelerometer/gyroscope + MMC5983MA magnetometer via I2C.

Dostarcza skalibrowane dane 9-osiowe (ax, ay, az, gx, gy, gz, mx, my, mz)
dla filtru Kalmana i filtra AHRS Madgwicka.

Kalibracja czujników jest wczytywana z pliku calibration.json i stosowana
do każdego odczytu:
  • akcelerometr — odjęcie biasu i skalowanie (jednostki: m/s²)
  • żyroskop     — odjęcie biasu (jednostki: °/s)
  • magnetometr  — kompensacja hard iron i soft iron (jednostki umowne)

Magnetometr jest próbkowany co MAG_SAMPLE_DIV-ty odczyt (~20 Hz przy 100 Hz IMU).
Między odczytami magnetometru, ostatnia wartość mx/my/mz jest powtarzana
w IMUReading — zmienia się wystarczająco wolno, by to nie wpływało na jakość.
"""

import json
import math
import time
import logging
import threading
import argparse
from dataclasses import dataclass, field
from typing import Optional

import smbus2

logger = logging.getLogger(__name__)

# rejestry ISM330DHCX

ISM_ADDR      = 0x6B
REG_CTRL1_XL  = 0x10
REG_CTRL2_G   = 0x11
REG_OUTX_L_G  = 0x22   # gyro X/Y/Z then accel X/Y/Z (12 bytes)

# accel +/-4g z datasheetu daje czułość (sensitivity) 0.000122 g/LSB
# https://www.st.com/resource/en/datasheet/ism330dhcx.pdf
# sensitivity konwertuje wynik z ADC na m/s^2
ACCEL_SENS_MS2 = 0.000122 * 9.80665
# ten sam współczynnik w jednostkach g
# bo madgwick_filter.py operuje w jednostkach g
ACCEL_SENS_G   = 0.000122

# czułość dla zakresu żyroskopu +/- 250 dps
# datasheet podaje +/- 2000dps, ale wtedy odczyty się nie zgadzały 
# z rzeczywistym położeniem urządzenia
GYRO_SENS_DPS  = 0.00875

# rejestry MMC5983MA 

MMC_ADDR       = 0x30
MMC_REG_XOUT0  = 0x00
MMC_REG_XOUT1  = 0x01
MMC_REG_YOUT0  = 0x02
MMC_REG_YOUT1  = 0x03
MMC_REG_ZOUT0  = 0x04  
MMC_REG_ZOUT1  = 0x05
MMC_REG_XYZ2   = 0x06
MMC_REG_STATUS = 0x08 
MMC_REG_CTRL0  = 0x09
MMC_REG_WHOAMI = 0x2F

MMC_NULL_FIELD = 131072

# deklinacja magnetyczna dla Krakowa
# do zmiany jeżeli czujnik będzie używany w innych rejonach
# https://www.magnetic-declination.com
MAG_DECLINATION_DEG = 5.5

# Plik kalibracyjny — ten sam, który używa madgwick_filter.py.
# Zawiera bias gyro/accel (w g i °/s) oraz hard/soft iron magnetometru.
CALIBRATION_FILE = "calibration.json"

@dataclass
class IMUReading:
    ax: float       
    ay: float      
    az: float      
    gx: float     
    gy: float     
    gz: float       
    mx: float       
    my: float      
    mz: float
    timestamp: float = field(default_factory=time.time)

@dataclass
class MagReading:
    mx: float 
    my: float
    mz: float
    heading_deg: float
    timestamp: float = field(default_factory=time.time)

# konwertuje unsigned 16bit int na signed 16bit int
def u16_to_s16(v: int) -> int:
    return v - 65536 if v > 32767 else v

# odczyt 12tu bajtów z rejestrów ISM330DHCX
def read_imu_raw(bus: smbus2.SMBus) -> tuple:
    """Zwraca (gx, gy, gz, ax, ay, az) jako 16-bitowy signed integer"""
    data = bus.read_i2c_block_data(ISM_ADDR, REG_OUTX_L_G, 12)
    gx = u16_to_s16(data[1] << 8 | data[0])  # żyroskop X
    gy = u16_to_s16(data[3] << 8 | data[2])
    gz = u16_to_s16(data[5] << 8 | data[4])
    ax = u16_to_s16(data[7] << 8 | data[6])
    ay = u16_to_s16(data[9] << 8 | data[8])
    az = u16_to_s16(data[11] << 8 | data[10])
    return gx, gy, gz, ax, ay, az

def raw_to_imu(gx, gy, gz, ax, ay, az, mx=0.0, my=0.0, mz=0.0) -> IMUReading:
    """Konwersja integera z czujnika na jednostki fizyczne"""
    return IMUReading(
        ax=ax * ACCEL_SENS_MS2,
        ay=ay * ACCEL_SENS_MS2,
        az=az * ACCEL_SENS_MS2,
        gx=gx * GYRO_SENS_DPS,
        gy=gy * GYRO_SENS_DPS,
        gz=gz * GYRO_SENS_DPS,
        mx=mx, my=my, mz=mz,
    )

def load_calibration(filename: str = CALIBRATION_FILE) -> Optional[dict]:
    """
    Wczytuje plik kalibracyjny. Zwraca None, jeśli plik nie istnieje.
    Format pliku jest taki sam jak w madgwick_filter.py:
      { "gyro":  { "bias_x", "bias_y", "bias_z" },
        "accel": { "bias_x", "bias_y", "bias_z", "scale_x", "scale_y", "scale_z" },
        "mag":   { "hard_iron_x", "hard_iron_y", "hard_iron_z",
                   "soft_iron_scale_x", "soft_iron_scale_y", "soft_iron_scale_z" } }
    """
    from pathlib import Path
    if not Path(filename).exists():
        logger.warning(f"IMULocator: brak pliku kalibracyjnego {filename}")
        return None
    with open(filename) as f:
        cal = json.load(f)
    logger.info(f"IMULocator: wczytano kalibrację z {filename}")
    return cal

def apply_cal(reading: IMUReading, cal: dict) -> IMUReading:
    """
    Stosuje kalibrację do odczytu IMU.

    Żyroskop: odjęcie biasu (°/s).
    Akcelerometr: odjęcie biasu i skalowanie.
      calibration.json przechowuje bias w jednostkach g (bo madgwick_filter.py
      operuje w g). Przeliczamy bias na m/s² przed odjęciem.
    Magnetometr: kompensacja hard iron (odjęcie) i soft iron (mnożenie).
    """
    g = cal["gyro"]
    a = cal["accel"]
    m = cal["mag"]

    G = 9.80665   # stała przeliczeniowa g -> m/s²

    return IMUReading(
        # konwersja g na m/s^2, odjęcie biasu, skalowanie.
        # konkretna wartość stałej skalowania pochodzi z kalibracji czujnika
        # przeprowadzonej w osobnym skrypcie imu_calibration.py
        ax=(reading.ax / G - a["bias_x"]) / a["scale_x"] * G,
        ay=(reading.ay / G - a["bias_y"]) / a["scale_y"] * G,
        az=(reading.az / G - a["bias_z"]) / a["scale_z"] * G,
        # żyroskop w spoczynku nie pokazuje idealnie 0 °/s - ma systematyczny dryf (bias)
        # kalibracja polega na uśrednieniu wielu odczytów w bezruchu i zapisaniu średniej jako biasu
        # potem bias jest odejmowany od każdego odczytu.
        gx=reading.gx - g["bias_x"],
        gy=reading.gy - g["bias_y"],
        gz=reading.gz - g["bias_z"],
        # https://atadiat.com/en/e-magnetometer-soft-iron-and-hard-iron-calibration-why-how/
        # kokretne wartości zostały znalezione przez kalibrację urządzenia
        # przeprowadzonej w skrypcie imu_calibration.py
        mx=(reading.mx - m["hard_iron_x"]) * m["soft_iron_scale_x"],
        my=(reading.my - m["hard_iron_y"]) * m["soft_iron_scale_y"],
        mz=(reading.mz - m["hard_iron_z"]) * m["soft_iron_scale_z"],
        timestamp=reading.timestamp,
    )

def init_mag(bus: smbus2.SMBus) -> bool:
    """
    Inicjalizacja MMC5983MA.
    Aktywuje funkcję Auto SET/RESET.
    """
    try:
        who = bus.read_byte_data(MMC_ADDR, MMC_REG_WHOAMI)
    except Exception as e:
        logger.error(f"MMC5983MA: odczyt I2C nie powiódł się: {e}")
        return False

    if who == 0x00:
        logger.error(f"MMC5983MA WHO_AM_I = 0x00 — chip nie odpowiada")
        return False

    if who != 0xFF:
        logger.warning(f"MMC5983MA WHO_AM_I = 0x{who:02X}")

    # Aktywuj automatyczne SET/RESET
    bus.write_byte_data(MMC_ADDR, MMC_REG_CTRL0, 0x20)
    time.sleep(0.01)
    # Wyślij komende SET w celu początkowego resetu czujnika
    bus.write_byte_data(MMC_ADDR, MMC_REG_CTRL0, 0x08)
    time.sleep(0.01)
    logger.info("IMULocator: MMC5983MA magnetometr zainicjalizowany")
    return True

def read_mag_raw(bus: smbus2.SMBus) -> Optional[tuple]:
    """
    Zwraca pojedynczy pomiar magmnetometru (mx, my, mz)
    """
    # wysyła polecenie wykonania pomiaru
    bus.write_byte_data(MMC_ADDR, MMC_REG_CTRL0, 0x21)
    # skrypt czeka na fizyczny pomiar pola
    for _ in range(50):
        time.sleep(0.001)
        # co 1ms sprawdź czy dane są gotowe
        if bus.read_byte_data(MMC_ADDR, MMC_REG_STATUS) & 0x10:
            break
    else:
        logger.warning("MMC5983MA upłynał czas pomiaru")
        return None

    # dane z mag rozproszone w 7 rejestrach
    data = bus.read_i2c_block_data(MMC_ADDR, MMC_REG_XOUT0, 7)
    # złóż rozproszone bity z pamięci czujnika w jedną pełną liczbę dla każdej osi X, Y, Z
    mx = (data[0] << 10) | (data[1] << 2) | ((data[6] >> 6) & 0x03)
    my = (data[2] << 10) | (data[3] << 2) | ((data[6] >> 4) & 0x03)
    mz = (data[4] << 10) | (data[5] << 2) | ((data[6] >> 2) & 0x03)

    # uzyskaj wartośc ze znakiem (signed)
    mx -= MMC_NULL_FIELD
    my -= MMC_NULL_FIELD
    mz -= MMC_NULL_FIELD

    return mx, my, mz

def compute_heading(mx: float, my: float, mz: float,
                     ax: float, ay: float, az: float) -> float:
    """
    Kompensacja przechyłu, jeżeli trzymamy kompas krzywo
    """
    # kąty roll (przechył boczny) i pitch (przód-tył) z akcelerometru
    roll  = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

    # kompensacja przechyłu
    # składowe pola magnetycznego są transformowane do płaszczyzny poziomej z uwzględnieniem tych kątów
    xh = mx * math.cos(pitch) + mz * math.sin(pitch)
    yh = (mx * math.sin(roll) * math.sin(pitch)
          + my * math.cos(roll)
          - mz * math.sin(roll) * math.cos(pitch))

    heading = math.degrees(math.atan2(-yh, xh))

    # zastosuj korektę deklinacji magnetycznej
    heading += MAG_DECLINATION_DEG

    # znormalizuj na 0–360
    return heading % 360.0


STALE_AFTER_S  = 0.5
MAG_SAMPLE_DIV = 5      # odczytaj magnetometr co n-ty odczyt IMU (20Hz przy 100Hz IMU)


class IMULocator:
    """
    Odczytuje ISM330DHCX (accel+gyro) i MMC5983MA (magnetometr) co ~100Hz / ~20Hz.

    Odczyty są kalibrowane na podstawie calibration.json (bias, hard/soft iron).
    IMUReading zawiera pełne 9 osi (ax, ay, az, gx, gy, gz, mx, my, mz).
    Między odczytami magnetometru (~20 Hz) ostatnia wartość mx/my/mz jest
    powtarzana, bo pole magnetyczne zmienia się znacznie wolniej niż 100 Hz.
    """

    def __init__(self, i2c_bus: int = 1, sample_rate_hz: float = 100.0,
                 print_rate_hz: float = 10.0, print_raw: bool = False,
                 print_mag: bool = False):
        self._bus_num      = i2c_bus
        self._sample_dt    = 1.0 / sample_rate_hz
        self._print_every  = max(1, int(sample_rate_hz / print_rate_hz))
        self._print_raw    = print_raw
        self._print_mag    = print_mag

        self._reading:     Optional[IMUReading] = None
        self._mag:         Optional[MagReading] = None
        self._lock         = threading.Lock()
        self._mag_lock     = threading.Lock()
        self._thread:      Optional[threading.Thread] = None
        self._running      = False
        self._mag_ok       = False

        # Kalibracja — wczytywana z calibration.json (ten sam plik co madgwick_filter.py)
        self._cal = load_calibration()

        # Cache ostatnich wartości magnetometru — powtarzane między odczytami.
        # Magnetometr jest próbkowany ~20 Hz, ale IMUReading jest tworzony ~100 Hz.
        # Między próbkami używamy ostatnich znanych wartości mx/my/mz.
        self._last_mx: float = 0.0
        self._last_my: float = 0.0
        self._last_mz: float = 0.0

    # ── public ────────────────────────────────────────────────────────────────

    def start(self) -> None:
        self._running = True
        self._thread  = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        logger.info("IMULocator: start")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        logger.info("IMULocator: stop")

    def get_reading(self) -> Optional[IMUReading]:
        with self._lock:
            if self._reading is None:
                return None
            if time.time() - self._reading.timestamp > STALE_AFTER_S:
                return None
            return self._reading

    def get_mag_heading(self) -> Optional[float]:
        """
        Zwraca odczyt kompasu
        """
        with self._mag_lock:
            if self._mag is None:
                return None
            if time.time() - self._mag.timestamp > 2.0:
                return None
            return self._mag.heading_deg

    def _run(self) -> None:
        try:
            bus = smbus2.SMBus(self._bus_num)
            bus.write_byte_data(ISM_ADDR, REG_CTRL1_XL, 0xA8)  # accel 6.66kHz ±4g
            bus.write_byte_data(ISM_ADDR, REG_CTRL2_G,  0xAC)  # gyro  6.66kHz ±2000dps
            time.sleep(0.1)
            logger.info(f"IMULocator: ISM330DHCX zainicjalizowany na I2C {self._bus_num}")

            self._mag_ok = init_mag(bus)

            self.print_header()

        except Exception as e:
            logger.error(f"IMULocator: inicjalizacja nie powiodła się: {e}")
            return

        sample_count = 0
        while self._running:
            try:
                gx_r, gy_r, gz_r, ax_r, ay_r, az_r = read_imu_raw(bus)

                # Odczyt magnetometru co MAG_SAMPLE_DIV-tą próbkę (~20 Hz).
                # Między odczytami używamy ostatnich znanych wartości
                mag_reading = None
                if self._mag_ok and sample_count % MAG_SAMPLE_DIV == 0:
                    raw = read_mag_raw(bus)
                    if raw is not None:
                        self._last_mx, self._last_my, self._last_mz = raw

                # Budowa IMUReading z 9 osiami (mx/my/mz z cache)
                reading = raw_to_imu(gx_r, gy_r, gz_r, ax_r, ay_r, az_r, self._last_mx, self._last_my, self._last_mz)

                # Kalibracja, jeśli plik kalibracyjny został wczytany
                if self._cal is not None:
                    reading = apply_cal(reading, self._cal)

                with self._lock:
                    self._reading = reading

                if self._mag_ok and sample_count % MAG_SAMPLE_DIV == 0:
                    hdg = compute_heading(reading.mx, reading.my, reading.mz,
                                           reading.ax, reading.ay, reading.az)
                    mag_reading = MagReading(mx=reading.mx, my=reading.my,
                                             mz=reading.mz, heading_deg=hdg)
                    with self._mag_lock:
                        self._mag = mag_reading

                sample_count += 1
                if sample_count % self._print_every == 0:
                    with self._mag_lock:
                        mag_for_print = self._mag
                    self.print_dashboard(reading, gx_r, gy_r, gz_r,
                                          ax_r, ay_r, az_r, mag_for_print)

                time.sleep(self._sample_dt)

            except Exception as e:
                logger.warning(f"IMULocator: błąd odczytu: {e}")
                time.sleep(0.5)

        bus.close()

    def print_header(self) -> None:
        if self._print_mag and self._mag_ok:
            print(
                f"{'Accel-X':>10} {'Accel-Y':>10} {'Accel-Z':>10}   "
                f"{'Gyro-X':>10} {'Gyro-Y':>10} {'Gyro-Z':>10}   "
                f"{'Heading':>9}"
            )
            print(
                f"{'(m/s²)':>10} {'(m/s²)':>10} {'(m/s²)':>10}   "
                f"{'(°/s)':>10} {'(°/s)':>10} {'(°/s)':>10}   "
                f"{'(°True)':>9}"
            )
            print("-" * 83)
        else:
            print(
                f"{'Accel-X':>10} {'Accel-Y':>10} {'Accel-Z':>10}   "
                f"{'Gyro-X':>10} {'Gyro-Y':>10} {'Gyro-Z':>10}"
            )
            print(
                f"{'(m/s²)':>10} {'(m/s²)':>10} {'(m/s²)':>10}   "
                f"{'(°/s)':>10} {'(°/s)':>10} {'(°/s)':>10}"
            )
            print("-" * 69)

    def print_dashboard(self, r: IMUReading,
                         gx_r, gy_r, gz_r, ax_r, ay_r, az_r,
                         mag: Optional[MagReading]) -> None:
        line = (
            f"{r.ax:+10.3f} {r.ay:+10.3f} {r.az:+10.3f}   "
            f"{r.gx:+10.3f} {r.gy:+10.3f} {r.gz:+10.3f}"
        )
        if self._print_mag and mag is not None:
            line += f"   {mag.heading_deg:>8.1f}°"
        print(line, flush=True)

        if self._print_raw:
            print(
                f"  raw: ax={ax_r:6d} ay={ay_r:6d} az={az_r:6d}   "
                f"gx={gx_r:6d} gy={gy_r:6d} gz={gz_r:6d}",
                flush=True
            )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s  [%(levelname)s]  %(message)s",
                        datefmt="%H:%M:%S")

    ap = argparse.ArgumentParser(description="Odczytuje IMU i magnetometr")
    ap.add_argument("--rate", type=float, default=10.0,
                    help="Częstotliwość wyświetlania (10 Hz)")
    ap.add_argument("--raw",  action="store_true",
                    help="Wypisz surowe wartości z czujnika")
    ap.add_argument("--mag",  action="store_true",
                    help="Pokaż kierunek")
    args = ap.parse_args()

    imu = IMULocator(print_rate_hz=args.rate, print_raw=args.raw,
                     print_mag=args.mag)
    imu.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\Zatrzymuję...")
        imu.stop()
