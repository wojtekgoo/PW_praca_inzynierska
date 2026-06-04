#!/usr/bin/python

"""
Madgwick Orientation Filter for ISM330DHCX + MMC5983MA

Fuses accelerometer + gyroscope + magnetometer to produce
stable heading, pitch and roll — without drift.

Theory:
    Gyroscope alone drifts over time (integrating small errors).
    Accelerometer gives tilt but not heading, and is noisy.
    Magnetometer gives heading but is sensitive to interference.
    Madgwick filter fuses all three optimally using gradient descent.

Reference:
    Madgwick, S. (2010). "An efficient orientation filter for
    inertial and inertial/magnetic sensor arrays."

Usage:
    python madgwick_filter.py

Requirements:
    pip install smbus2 numpy
"""

import smbus2
import time
import math
import json
import numpy as np

# ── Configuration ─────────────────────────────────────────────────────────────

CALIBRATION_FILE = "calibration.json"
SAMPLE_RATE      = 100    # Hz — how often we read the sensor
BETA             = 0.1    # Madgwick filter gain
                          # Higher = faster response but noisier
                          # Lower  = smoother but slower to correct drift
                          # 0.033 is Madgwick's original recommendation
                          # 0.1 works well in practice

# ── I2C addresses ─────────────────────────────────────────────────────────────

ISM_ADDR = 0x6b
MMC_ADDR = 0x30

# ── Scale factors (must match register config below) ──────────────────────────

ACCEL_SCALE = 0.061  / 1000.0   # ±2g   → 0.061 mg/LSB → g
GYRO_SCALE  = 70.0   / 1000.0   # ±2000dps → 70 mdps/LSB → dps

# ── Hardware init ─────────────────────────────────────────────────────────────

bus = smbus2.SMBus(1)

def init_sensors():
    bus.write_byte_data(ISM_ADDR, 0x10, 0xA0)   # CTRL1_XL: 6.66kHz, ±2g
    bus.write_byte_data(ISM_ADDR, 0x11, 0xA0)   # CTRL2_G:  6.66kHz, ±2000dps
    bus.write_byte_data(MMC_ADDR, 0x09, 0x01)   # MMC: take measurement
    time.sleep(0.1)

def s16(v):
    return v - 65536 if v > 32767 else v

def read_imu_raw():
    data = bus.read_i2c_block_data(ISM_ADDR, 0x22, 12)
    gx = s16(data[1] << 8 | data[0]) * GYRO_SCALE
    gy = s16(data[3] << 8 | data[2]) * GYRO_SCALE
    gz = s16(data[5] << 8 | data[4]) * GYRO_SCALE
    ax = s16(data[7] << 8 | data[6]) * ACCEL_SCALE
    ay = s16(data[9] << 8 | data[8]) * ACCEL_SCALE
    az = s16(data[11]<< 8 | data[10])* ACCEL_SCALE
    return ax, ay, az, gx, gy, gz

def read_mag_raw():
    bus.write_byte_data(MMC_ADDR, 0x09, 0x01)
    time.sleep(0.01)
    data = bus.read_i2c_block_data(MMC_ADDR, 0x00, 7)
    mx = (data[0] << 10) | (data[1] << 2) | ((data[6] >> 6) & 0x03)
    my = (data[2] << 10) | (data[3] << 2) | ((data[6] >> 4) & 0x03)
    mz = (data[4] << 10) | (data[5] << 2) | ((data[6] >> 2) & 0x03)
    mx -= 131072
    my -= 131072
    mz -= 131072
    mz  = -mz    # flip Z to match ISM330DHCX coordinate system
    return mx, my, mz

# ── Calibration ───────────────────────────────────────────────────────────────

def load_calibration(filename=CALIBRATION_FILE):
    with open(filename) as f:
        return json.load(f)

def apply_calibration(ax, ay, az, gx, gy, gz, mx, my, mz, cal):
    g = cal["gyro"]
    a = cal["accel"]
    m = cal["mag"]

    # Gyro: subtract bias
    gx -= g["bias_x"]
    gy -= g["bias_y"]
    gz -= g["bias_z"]

    # Accel: subtract bias, divide by scale
    ax = (ax - a["bias_x"]) / a["scale_x"]
    ay = (ay - a["bias_y"]) / a["scale_y"]
    az = (az - a["bias_z"]) / a["scale_z"]

    # Mag: subtract hard iron, multiply by soft iron scale
    mx = (mx - m["hard_iron_x"]) * m["soft_iron_scale_x"]
    my = (my - m["hard_iron_y"]) * m["soft_iron_scale_y"]
    mz = (mz - m["hard_iron_z"]) * m["soft_iron_scale_z"]

    return ax, ay, az, gx, gy, gz, mx, my, mz

# ── Madgwick Filter ───────────────────────────────────────────────────────────

class MadgwickFilter:
    """
    Madgwick AHRS (Attitude and Heading Reference System) filter.

    Maintains orientation as a quaternion q = [w, x, y, z].
    A quaternion is a 4D number that represents rotation in 3D space
    without gimbal lock problems that affect Euler angles.

    We convert to Euler angles (heading/pitch/roll) only at the end
    for human-readable output.
    """

    def __init__(self, beta=BETA, sample_rate=SAMPLE_RATE):
        self.beta        = beta
        self.dt          = 1.0 / sample_rate
        # Initial quaternion — identity rotation (no rotation)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

    def update(self, ax, ay, az, gx, gy, gz, mx, my, mz):
        """
        Update quaternion with new sensor readings.

        Inputs:
            ax, ay, az  — accelerometer in g
            gx, gy, gz  — gyroscope in degrees/second
            mx, my, mz  — magnetometer (calibrated, any unit)
        """
        q = self.q
        dt = self.dt

        # Convert gyroscope to radians/second
        gx = math.radians(gx)
        gy = math.radians(gy)
        gz = math.radians(gz)

        # ── Step 1: Normalize accelerometer ──────────────────────────────────
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm == 0:
            return   # bad reading, skip
        ax /= norm; ay /= norm; az /= norm

        # ── Step 2: Normalize magnetometer ───────────────────────────────────
        norm = math.sqrt(mx*mx + my*my + mz*mz)
        if norm == 0:
            return
        mx /= norm; my /= norm; mz /= norm

        # ── Step 3: Reference direction of Earth's magnetic field ─────────────
        # Rotate magnetometer reading into Earth frame using current quaternion
        # This gives us the expected direction of magnetic north in Earth frame
        q0, q1, q2, q3 = q
        hx = (2*mx*(0.5 - q2*q2 - q3*q3) +
              2*my*(q1*q2 - q0*q3) +
              2*mz*(q1*q3 + q0*q2))
        hy = (2*mx*(q1*q2 + q0*q3) +
              2*my*(0.5 - q1*q1 - q3*q3) +
              2*mz*(q2*q3 - q0*q1))
        hz = (2*mx*(q1*q3 - q0*q2) +
              2*my*(q2*q3 + q0*q1) +
              2*mz*(0.5 - q1*q1 - q2*q2))

        # Project onto horizontal plane (bx = north component, bz = vertical)
        bx = math.sqrt(hx*hx + hy*hy)
        bz = hz

        # ── Step 4: Gradient descent — find correction direction ──────────────
        # These are the partial derivatives of the objective function
        # They point in the direction that reduces error between
        # measured and expected sensor values

        # Objective function terms (accel + mag)
        f1 = 2*(q1*q3 - q0*q2)                         - ax
        f2 = 2*(q0*q1 + q2*q3)                         - ay
        f3 = 2*(0.5 - q1*q1 - q2*q2)                   - az
        f4 = 2*bx*(0.5 - q2*q2 - q3*q3) + 2*bz*(q1*q3 - q0*q2) - mx
        f5 = 2*bx*(q1*q2 - q0*q3)       + 2*bz*(q0*q1 + q2*q3) - my
        f6 = 2*bx*(q0*q2 + q1*q3)       + 2*bz*(0.5 - q1*q1 - q2*q2) - mz

        # Jacobian matrix — gradient of objective function
        j11 = -2*q2;       j12 =  2*q3;       j13 = -2*q0;       j14 =  2*q1
        j21 =  2*q1;       j22 =  2*q0;       j23 =  2*q3;       j24 =  2*q2
        j31 =  0;          j32 = -4*q1;       j33 = -4*q2;       j34 =  0
        j41 = -2*bz*q2;    j42 =  2*bz*q3;    j43 = -4*bx*q2 - 2*bz*q0; j44 = -4*bx*q3 + 2*bz*q1
        j51 = -2*bx*q3 + 2*bz*q1; j52 = 2*bx*q2 + 2*bz*q0; j53 = 2*bx*q1 + 2*bz*q3; j54 = -2*bx*q0 + 2*bz*q2
        j61 =  2*bx*q2;    j62 =  2*bx*q3 - 4*bz*q1; j63 = 2*bx*q0 - 4*bz*q2; j64 = 2*bx*q1

        # Gradient = Jᵀ × f
        step0 = j11*f1 + j21*f2 + j31*f3 + j41*f4 + j51*f5 + j61*f6
        step1 = j12*f1 + j22*f2 + j32*f3 + j42*f4 + j52*f5 + j62*f6
        step2 = j13*f1 + j23*f2 + j33*f3 + j43*f4 + j53*f5 + j63*f6
        step3 = j14*f1 + j24*f2 + j34*f3 + j44*f4 + j54*f5 + j64*f6

        # Normalize gradient
        norm = math.sqrt(step0**2 + step1**2 + step2**2 + step3**2)
        if norm > 0:
            step0 /= norm; step1 /= norm; step2 /= norm; step3 /= norm

        # ── Step 5: Gyroscope integration + gradient correction ───────────────
        # Rate of change of quaternion from gyroscope
        qdot0 = 0.5 * (-q1*gx - q2*gy - q3*gz)
        qdot1 = 0.5 * ( q0*gx + q2*gz - q3*gy)
        qdot2 = 0.5 * ( q0*gy - q1*gz + q3*gx)
        qdot3 = 0.5 * ( q0*gz + q1*gy - q2*gx)

        # Apply gradient correction (beta controls how much we trust accel/mag)
        qdot0 -= self.beta * step0
        qdot1 -= self.beta * step1
        qdot2 -= self.beta * step2
        qdot3 -= self.beta * step3

        # Integrate to get new quaternion
        q0 += qdot0 * dt
        q1 += qdot1 * dt
        q2 += qdot2 * dt
        q3 += qdot3 * dt

        # Normalize quaternion (keep it unit length)
        norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        self.q = np.array([q0/norm, q1/norm, q2/norm, q3/norm])

    def euler_angles(self):
        """
        Convert quaternion to Euler angles.

        Returns:
            heading  — compass bearing 0-360° (0=North, 90=East)
            pitch    — nose up/down angle (-90 to +90°)
            roll     — left/right tilt (-180 to +180°)
        """
        q0, q1, q2, q3 = self.q

        # Roll (rotation around X axis)
        roll = math.degrees(math.atan2(
            2*(q0*q1 + q2*q3),
            1 - 2*(q1*q1 + q2*q2)
        ))

        # Pitch (rotation around Y axis)
        sinp = 2*(q0*q2 - q3*q1)
        sinp = max(-1.0, min(1.0, sinp))   # clamp to [-1, 1]
        pitch = math.degrees(math.asin(sinp))

        # Yaw / Heading (rotation around Z axis)
        heading = math.degrees(math.atan2(
            2*(q0*q3 + q1*q2),
            1 - 2*(q2*q2 + q3*q3)
        ))

        # Convert heading from (-180, 180) to (0, 360)
        if heading < 0:
            heading += 360

        return heading, pitch, roll

# ── Main loop ─────────────────────────────────────────────────────────────────

def main():
    print("Loading calibration...")
    cal = load_calibration()

    print("Initializing sensors...")
    init_sensors()

    print("Starting Madgwick filter — let the sensor settle for a few seconds.\n")
    print(f"{'Heading':>10} {'Pitch':>8} {'Roll':>8}   Quaternion")
    print("-" * 65)

    filt = MadgwickFilter(beta=BETA, sample_rate=SAMPLE_RATE)

    dt = 1.0 / SAMPLE_RATE
    iteration = 0

    while True:
        t_start = time.time()

        # Read sensors
        ax, ay, az, gx, gy, gz = read_imu_raw()
        mx, my, mz             = read_mag_raw()

        # Apply calibration
        ax, ay, az, gx, gy, gz, mx, my, mz = apply_calibration(
            ax, ay, az, gx, gy, gz, mx, my, mz, cal
        )

        # Update filter
        filt.update(ax, ay, az, gx, gy, gz, mx, my, mz)

        # Get orientation
        heading, pitch, roll = filt.euler_angles()
        q = filt.q

        # Print every 10 iterations (10Hz display, 100Hz filter)
        if iteration % 10 == 0:
            print(f"{heading:10.2f}°  {pitch:+7.2f}°  {roll:+7.2f}°   "
                  f"[{q[0]:+.3f} {q[1]:+.3f} {q[2]:+.3f} {q[3]:+.3f}]")

        iteration += 1

        # Maintain sample rate
        elapsed = time.time() - t_start
        sleep_time = dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")