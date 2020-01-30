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

    def update_accel(self,accel_y, accel_z, fudge =0):
        rad = math.atan2(accel_z, -accel_y)
        # print("accel angle", math.degrees(rad))
        rad -= math.radians(fudge)

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
        self.output = 0
        self.lastMeasure = None

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def measure(self, value):
        if self.lastMeasure == None:
            self.lastMeasure = time.time()
            return 

        dt = time.time() - self.lastMeasure
        self.lastMeasure = time.time()

        error = self.setpoint - value

        self.intergral   += error * dt
        self.derivative   = error / dt
        self.proportional = error

        #prevents windup
        self.intergral = self.clamp(self.intergral)

        out = 0
        out += self.Kp * self.proportional
        out += self.Ki * self.intergral
        out += self.Kd * self.derivative

        self.output = self.clamp(out)

    def get_output(self):
        return self.output

    def clamp(self,value):
        if value > self.limits[1]:
            return self.limits[1]
        if value < self.limits[0]:
            return self.limits[0]
        return value


i2c = busio.I2C(board.SCL, board.SDA)
imu = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)


kit = MotorKit()
def drive(value):
    kit.motor1.throttle = -value
    kit.motor2.throttle =  value

filt = ComplementaryFilter(0.97)
pid = PID(0.12, 0.0001101001, 0.00, limits=(-0.7, 0.7) )

lastGyro = 0
lastAccel = 0
lastPrint = 0

def calibrate_gyro():
    su = 0
    T = 1000
    for _ in range(T):
        gyro_x, gyro_y, gyro_z = imu.gyro
        su += gyro_x
        time.sleep(0.01)
    print('gyro calibration value:',su / T)
    exit()

# calibrate_gyro()
gyro_x_cailb = 1.23

out_of_bound_count = 0

for _ in range(100):
    accel_x, accel_y, accel_z = imu.acceleration
    filt.update_accel(accel_y, accel_z)

try:
    while True:
        if time.time() - lastAccel > 0.03:
            # print("accel time", time.time() - lastAccel)
            accel_x, accel_y, accel_z = imu.acceleration
            filt.update_accel(accel_y, accel_z, fudge = -1.0)
            lastAccel = time.time()

        if time.time() - lastGyro > 0.005:
            # print("gyro time", time.time() - lastGyro)
            gyro_x, gyro_y, gyro_z = imu.gyro
            gyro_x -= gyro_x_cailb # super hack calibration
            # print('gyro x', gyro_x) # move calibration untill
            # readings have mean zero when stataionary
            filt.update_gyro(gyro_x)
            lastGyro = time.time()

        if time.time() - lastPrint > 0.005:
            # print("print time", time.time() - lastPrint)
            lastPrint = time.time()
            angle = filt.get_angle()
            output = pid.get_output()

            if math.fabs(angle) > 20:
                print("angle is", angle)
                out_of_bound_count +=1
                drive(0)
                if out_of_bound_count > 10:
                    exit()
            else:
                pid.measure(filt.get_angle())
                print("angle", angle, output)
                drive(output)
                out_of_bound_count = 0
finally:
    print('ending')
    drive(0)
