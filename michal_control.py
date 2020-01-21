import time
import math


from adafruit_motorkit import MotorKit

import board
import busio
import adafruit_lsm9ds1


class ComplementaryFilter:
    def __init__(self, K):
        self.K = K
        self.lastSin = None
        self.lastCos = None
        self.lastGyroTime = None
        self.lastGyro = 0

    def update_accel(self,accel_y, accel_z):
        # heading is in degrees
        rad = math.atan2(accel_z, accel_y)

        if self.lastSin == None or self.lastSin == None:
            self.lastSin = math.sin(rad)
            self.lastCos = math.cos(rad)
        else:
            self.lastSin = self.K * self.lastSin + (1-self.K) * math.sin(rad)
            self.lastCos = self.K * self.lastCos + (1-self.K) * math.cos(rad)

    def update_gyro(self,gyro_z):
        omega = -gyro_z
        # omega is in deg/s
        if self.lastGyroTime == None:
            self.lastGyroTime = time.time()
            return
        if self.lastSin == None or self.lastSin == None:
            return

        omega_rad = math.radians(omega)
        delta_t = time.time() - self.lastGyroTime
        self.lastGyroTime = time.time()

        rad = math.atan2(self.lastSin, self.lastCos) + omega_rad * delta_t
        self.lastSin = math.sin(rad)
        self.lastCos = math.cos(rad)

        self.lastGyro = (self.lastGyro + omega * delta_t) % 360

    def get_angle(self):
        rad = math.atan2(self.lastSin, self.lastCos)
        return math.degrees(rad) % 360


i2c = busio.I2C(board.SCL, board.SDA)
imu = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

# kit = MotorKit()
# kit.motor1.throttle = 1.0
# time.sleep(0.5)
# kit.motor1.throttle = 0


filt = ComplementaryFilter(0.99)

lastGyro = 0
lastAccel = 0
lastPrint = 0

while True:
    if time.time() - lastAccel > 0.10:
        accel_x, accel_y, accel_z = imu.acceleration
        filt.update_accel(accel_z, accel_y)
        lastAccel = time.time()

    if time.time() - lastGyro > 0.01:
        gyro_x, gyro_y, gyro_z = imu.gyro
        filt.update_gyro(-gyro_z)
        lastGyro = time.time()

    if time.time() - lastPrint > 0.05:
        print(filt.get_angle())
        lastPrint = time.time()
