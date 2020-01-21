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
        rad = math.atan2(accel_z, -accel_y)
        # print("accel angle", math.degrees(rad))

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
        # print("gyro only", self.lastGyro)

    def get_angle(self):
        rad = math.atan2(self.lastSin, self.lastCos)
        return math.degrees(rad)

class PID:

    def __init__(self, Kp, Ki, Kd, setpoint = 0, limits = (-1,1) ):
        self.setpoint = setpoint
        self.limits = limits

        self.intergral = 0
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def measure(self, value):
        dt = time.time() - lastMeasure
        lastMeasure = time.time()

        error = value - self.setpoint

        self.intergral   += error * dt
        self.derivative   = error / dt
        self.proportional = error

        #prevents windup
        self.intergral = self.clamp(self.intergral)

    def get_output(self):
        out = 0

        out += self.Kp * self.proportional
        out += self.Ki * self.intergral
        out += self.Kd * self.derivative

        return self.clamp(out)

    def clamp(self,value):
        if value > self.limits[1]:
            return self.limits[1]
        if value < self.limits[0]:
            return self.limits[0]
        return value


i2c = busio.I2C(board.SCL, board.SDA)
imu = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

# kit = MotorKit()
# kit.motor1.throttle = 1.0
# time.sleep(0.5)
# kit.motor1.throttle = 0

filt = ComplementaryFilter(0.95)
pid = PID(1, 0, 0)

lastGyro = 0
lastAccel = 0
lastPrint = 0

while True:
    if time.time() - lastAccel > 0.10:
        accel_x, accel_y, accel_z = imu.acceleration
        filt.update_accel(accel_y, accel_z)
        lastAccel = time.time()

    if time.time() - lastGyro > 0.01:
        gyro_x, gyro_y, gyro_z = imu.gyro
        gyro_x -= 1.9 # super hack calibration
        # print('gyro x', gyro_x) # move calibration untill
        # readings have mean zero when stataionary
        filt.update_gyro(gyro_x)
        lastGyro = time.time()

    if time.time() - lastPrint > 0.05:
        print(filt.get_angle())
        lastPrint = time.time()
