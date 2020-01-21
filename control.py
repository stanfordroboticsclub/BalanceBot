import time



# Motor examples
# from adafruit_motorkit import MotorKit
# kit = MotorKit()
# kit.motor1.throttle = 1.0
# time.sleep(0.5)
# kit.motor1.throttle = 0

# IMU examples
import board
import busio
import adafruit_lsm9ds1

i2c = busio.I2C(board.SCL, board.SDA)
imu = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

while 1:
    gyro_x, gyro_y, gyro_z = imu.gyro
    accel_x, accel_y, accel_z = imu.acceleration
    mag_x, mag_y, mag_z = imu.magnetic

    print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(accel_x, accel_y, accel_z))
    print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(mag_x, mag_y, mag_z))
    print('Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(gyro_x, gyro_y, gyro_z))

    time.sleep(0.5)

