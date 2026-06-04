#!/usr/bin/python


"""
IMU Calibration Script for ISM330DHCX (accel + gyro) and MMC5983MA (magnetometer)

Performs:
  1. Gyroscope bias calibration    — leave the sensor still
  2. Accelerometer bias calibration — place on 6 faces
  3. Magnetometer calibration       — rotate in figure-8

Saves results to: calibration.json

Usage:
    pip install smbus2 numpy
    python imu_calibration.py
"""

import smbus2
import time
import json
import numpy as np

# ── I2C addresses ─────────────────────────────────────────────────────────────

ISM_ADDR = 0x6b   # ISM330DHCX
MMC_ADDR = 0x30   # MMC5983MA

# ── Scaling factors ───────────────────────────────────────────────────────────
# These match the register config set below:
#   CTRL1_XL = 0xA0 → ±4g    → 0.122 mg/LSB
#   CTRL2_G  = 0xA0 → ±2000dps → 70 mdps/LSB

ACCEL_SCALE = 0.061 / 1000.0   # LSB → g
GYRO_SCALE  = 70.0  / 1000.0   # LSB → dps

# ── Low-level I2C helpers ─────────────────────────────────────────────────────

bus = smbus2.SMBus(1)

def write(addr, reg, val):
    bus.write_byte_data(addr, reg, val)

def read_block(addr, reg, length):
    return bus.read_i2c_block_data(addr, reg, length)

def s16(val):
    """Convert unsigned 16-bit to signed."""
    return val - 65536 if val > 32767 else val

# ── ISM330DHCX init ───────────────────────────────────────────────────────────

def init_ism330():
    write(ISM_ADDR, 0x10, 0xA0)   # CTRL1_XL: 6.66kHz, ±4g
    write(ISM_ADDR, 0x11, 0xA0)   # CTRL2_G:  6.66kHz, ±2000dps
    time.sleep(0.1)

def read_ism330():
    """Returns (ax, ay, az, gx, gy, gz) in raw LSB units."""
    data = read_block(ISM_ADDR, 0x22, 12)
    gx = s16(data[1] << 8 | data[0])
    gy = s16(data[3] << 8 | data[2])
    gz = s16(data[5] << 8 | data[4])
    ax = s16(data[7] << 8 | data[6])
    ay = s16(data[9] << 8 | data[8])
    az = s16(data[11] << 8 | data[10])
    return ax, ay, az, gx, gy, gz

def read_ism330_scaled():
    """Returns (ax, ay, az) in g and (gx, gy, gz) in dps."""
    ax, ay, az, gx, gy, gz = read_ism330()
    return (
        ax * ACCEL_SCALE, ay * ACCEL_SCALE, az * ACCEL_SCALE,
        gx * GYRO_SCALE,  gy * GYRO_SCALE,  gz * GYRO_SCALE
    )

# ── MMC5983MA init ────────────────────────────────────────────────────────────

def init_mmc5983():
    # Control register 0: take measurement
    write(MMC_ADDR, 0x09, 0x01)
    time.sleep(0.01)

def read_mmc5983():
    """Returns (mx, my, mz) as raw 18-bit values, centered at 2^17."""
    data = read_block(MMC_ADDR, 0x00, 7)
    mx = (data[0] << 10) | (data[1] << 2) | ((data[6] >> 6) & 0x03)
    my = (data[2] << 10) | (data[3] << 2) | ((data[6] >> 4) & 0x03)
    mz = (data[4] << 10) | (data[5] << 2) | ((data[6] >> 2) & 0x03)
    # Center around 0 (remove 2^17 offset)
    mx -= 131072
    my -= 131072
    mz -= 131072
    mz = -mz
    # Trigger next measurement
    write(MMC_ADDR, 0x09, 0x01)
    return mx, my, mz

# ── Step 1: Gyroscope calibration ─────────────────────────────────────────────

def calibrate_gyro(samples=2000):
    """
    Collect samples while sensor is still.
    Average = bias offset to subtract from all future readings.
    """
    print("\n" + "="*50)
    print("STEP 1: Gyroscope Calibration")
    print("="*50)
    print("Place the sensor on a flat, stable surface.")
    print("DO NOT move it during calibration.")
    input("Press Enter when ready...")

    print(f"Collecting {samples} samples", end="", flush=True)
    readings = []
    for i in range(samples):
        _, _, _, gx, gy, gz = read_ism330_scaled()
        readings.append([gx, gy, gz])
        if i % 50 == 0:
            print(".", end="", flush=True)
        time.sleep(0.002)

    readings = np.array(readings)
    bias = readings.mean(axis=0)
    noise = readings.std(axis=0)

    print(f"\n\nGyro bias  (dps): x={bias[0]:+.4f}  y={bias[1]:+.4f}  z={bias[2]:+.4f}")
    print(f"Gyro noise (dps): x={noise[0]:.4f}   y={noise[1]:.4f}   z={noise[2]:.4f}")
    print("✓ Gyroscope calibration complete")

    return {"bias_x": bias[0], "bias_y": bias[1], "bias_z": bias[2]}

# ── Step 2: Accelerometer calibration ─────────────────────────────────────────

FACES = [
    ("X up   (+X = +1g)", 0,  1),
    ("X down (-X = +1g)", 0, -1),
    ("Y up   (+Y = +1g)", 1,  1),
    ("Y down (-Y = +1g)", 1, -1),
    ("Z up   (+Z = +1g)", 2,  1),
    ("Z down (-Z = +1g)", 2, -1),
]

def calibrate_accel(samples=200):
    """
    6-position calibration.
    Place sensor on each of 6 faces — one axis should read ±1g each time.
    Compute offset and scale factor per axis.
    """
    print("\n" + "="*50)
    print("STEP 2: Accelerometer Calibration (6 faces)")
    print("="*50)
    print("You will place the sensor in 6 orientations.")
    print("Keep it still during each measurement.\n")

    face_means = []

    for desc, axis, sign in FACES:
        input(f"Place sensor: {desc}  → then press Enter...")
        readings = []
        for _ in range(samples):
            ax, ay, az, _, _, _ = read_ism330_scaled()
            readings.append([ax, ay, az])
            time.sleep(0.005)
        mean = np.mean(readings, axis=0)
        face_means.append(mean)
        print(f"  Read: x={mean[0]:+.4f}g  y={mean[1]:+.4f}g  z={mean[2]:+.4f}g\n")

    # Each axis has two readings: +1g face and -1g face
    # bias  = (positive_reading + negative_reading) / 2
    # scale = (positive_reading - negative_reading) / 2   (should = 1.0)
    axes = ['x', 'y', 'z']
    result = {}

    print("\nAccelerometer calibration results:")
    for i, axis in enumerate(axes):
        pos = face_means[i*2][i]    # reading when this axis points up
        neg = face_means[i*2+1][i]  # reading when this axis points down
        bias  = (pos + neg) / 2.0
        scale = (pos - neg) / 2.0
        print(f"  {axis}: bias={bias:+.5f}g  scale={scale:.5f}  (ideal: 0, 1)")
        result[f"bias_{axis}"]  = bias
        result[f"scale_{axis}"] = scale

    print("✓ Accelerometer calibration complete")
    return result

# ── Step 3: Magnetometer calibration ──────────────────────────────────────────

def calibrate_mag(duration=30):
    """
    Hard iron + soft iron calibration.

    Hard iron: fixed offset caused by permanent magnets near sensor
    Soft iron: axis scaling caused by ferromagnetic materials

    Method: rotate sensor slowly in all directions for `duration` seconds.
    Fit a sphere to the collected points → find center (hard iron offset)
    and scale factors (soft iron correction).
    """
    print("\n" + "="*50)
    print("STEP 3: Magnetometer Calibration")
    print("="*50)
    print(f"Slowly rotate the sensor in ALL directions for {duration} seconds.")
    print("Try to cover all orientations — figure-8 motion works well.")
    input("Press Enter to start...")

    print(f"Collecting data for {duration}s — ROTATE NOW...")
    readings = []
    start = time.time()

    while time.time() - start < duration:
        mx, my, mz = read_mmc5983()
        readings.append([mx, my, mz])
        elapsed = time.time() - start
        remaining = duration - elapsed
        print(f"\r  {remaining:.0f}s remaining  samples={len(readings)}", end="", flush=True)
        time.sleep(0.05)

    print(f"\n\nCollected {len(readings)} samples")
    readings = np.array(readings, dtype=float)

    # Hard iron correction: center of the min/max bounding box
    hard_iron = (readings.max(axis=0) + readings.min(axis=0)) / 2.0

    # Soft iron correction: scale each axis to unit sphere
    ranges = (readings.max(axis=0) - readings.min(axis=0)) / 2.0
    avg_range = ranges.mean()
    soft_iron_scale = avg_range / ranges   # per-axis scale factor

    print(f"\nHard iron offset: x={hard_iron[0]:.1f}  y={hard_iron[1]:.1f}  z={hard_iron[2]:.1f}")
    print(f"Soft iron scale:  x={soft_iron_scale[0]:.4f}  y={soft_iron_scale[1]:.4f}  z={soft_iron_scale[2]:.4f}")
    print("✓ Magnetometer calibration complete")

    return {
        "hard_iron_x": hard_iron[0],
        "hard_iron_y": hard_iron[1],
        "hard_iron_z": hard_iron[2],
        "soft_iron_scale_x": soft_iron_scale[0],
        "soft_iron_scale_y": soft_iron_scale[1],
        "soft_iron_scale_z": soft_iron_scale[2],
    }

# ── Save / load calibration ───────────────────────────────────────────────────

def save_calibration(gyro, accel, mag, filename="calibration.json"):
    data = {
        "gyro":  gyro,
        "accel": accel,
        "mag":   mag,
    }
    with open(filename, "w") as f:
        json.dump(data, f, indent=2)
    print(f"\n✓ Calibration saved to {filename}")

def load_calibration(filename="calibration.json"):
    with open(filename) as f:
        return json.load(f)

# ── Verify calibration ────────────────────────────────────────────────────────

def verify_calibration(cal, samples=100):
    """
    Quick check — apply calibration and show corrected readings.
    Gyro should be ~0 when still.
    Accel magnitude should be ~1g when still.
    """
    print("\n" + "="*50)
    print("VERIFICATION — leave sensor still")
    print("="*50)
    time.sleep(1)

    g_cal  = cal["gyro"]
    a_cal  = cal["accel"]

    accel_mags = []
    gyro_mags  = []

    for _ in range(samples):
        ax, ay, az, gx, gy, gz = read_ism330_scaled()

        # Apply corrections
        gx -= g_cal["bias_x"]
        gy -= g_cal["bias_y"]
        gz -= g_cal["bias_z"]

        ax = (ax - a_cal["bias_x"]) / a_cal["scale_x"]
        ay = (ay - a_cal["bias_y"]) / a_cal["scale_y"]
        az = (az - a_cal["bias_z"]) / a_cal["scale_z"]

        accel_mags.append(np.sqrt(ax**2 + ay**2 + az**2))
        gyro_mags.append(np.sqrt(gx**2 + gy**2 + gz**2))
        time.sleep(0.01)

    print(f"Accel magnitude: {np.mean(accel_mags):.4f}g  (ideal: 1.0000g)")
    print(f"Gyro  magnitude: {np.mean(gyro_mags):.4f} dps (ideal: 0.0000 dps)")

    accel_error = abs(np.mean(accel_mags) - 1.0)
    if accel_error < 0.02:
        print("✓ Accelerometer looks good")
    else:
        print(f"⚠ Accelerometer error {accel_error:.4f}g — consider redoing calibration")

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("╔══════════════════════════════════════╗")
    print("║    ISM330DHCX + MMC5983MA            ║")
    print("║    IMU Calibration Tool              ║")
    print("╚══════════════════════════════════════╝")

    init_ism330()
    init_mmc5983()

    gyro_cal  = calibrate_gyro()
    accel_cal = calibrate_accel()
    mag_cal   = calibrate_mag()

    save_calibration(gyro_cal, accel_cal, mag_cal)
    verify_calibration({"gyro": gyro_cal, "accel": accel_cal, "mag": mag_cal})

    print("\n✓ All done! calibration.json is ready to use in your other scripts.")

if __name__ == "__main__":
    main()