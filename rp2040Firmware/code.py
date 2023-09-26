import board
import busio
import digitalio
import bitbangio
from adafruit_bus_device.i2c_device import I2CDevice

from adafruit_bno08x.i2c import BNO08X_I2C
#from adafruit_bno08x import BNO_REPORT_ACCELEROMETER
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from ulab import numpy as np
import time

def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

def rotationMatrixToEulerAngles(R) :
    sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
 
    singular = sy < 1e-6
 
    if  not singular :
        x = np.arctan2(R[2,1] , R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else :
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
 
    return np.array([x*180/3.14, y*180/3.14, z*180/3.14])

class Quaternion:
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

def quaternion_conjugate(q):
    return Quaternion(q.w, -q.x, -q.y, -q.z)

def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1.w, q1.x, q1.y, q1.z
    w2, x2, y2, z2 = q2.w, q2.x, q2.y, q2.z

    # Calculate the resulting quaternion components
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return Quaternion(w, x, y, z)

def getCurl(q):
    return np.arctan2(2 * (q.x * q.w - q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))

# cs = digitalio.DigitalInOut(board.GP13)
# cs.direction = digitalio.Direction.OUTPUT
# cs.value = True
# 
# spi = busio.SPI(board.GP14, MISO=board.GP12, MOSI=board.GP15)
# 
# while not spi.try_lock():
#     pass
# try:
#     spi.configure(baudrate=8000000, polarity=1, phase=1, bits=8)
#     cs.value = True
#     spi.write(bytes([0x01, 0xFF]))
#     cs.value = False
# finally:
#     spi.unlock()

# cs = digitalio.DigitalInOut(board.GP13)
# cs.direction = digitalio.Direction.INPUT
# spi = busio.SPI(board.GP14, MISO=board.GP12, MOSI=board.GP15)
# while not spi.try_lock():
#     pass
# spi.configure(baudrate=8000000, phase=1, polarity=1, bits=8)

# i2c_0 = busio.I2C(board.GP5, board.GP4)
i2c_0 = bitbangio.I2C(board.GP5, board.GP4, timeout = 1000)
i2c_1 = bitbangio.I2C(board.GP1, board.GP0, timeout = 1000)
# i2c = bitbangio.I2C(board.GP5, board.GP4, frequency=10000)

bnoRef = BNO08X_I2C(i2c_0, None, 0x4B)
bnoIndex = BNO08X_I2C(i2c_1, None, 0x4A)
# bnoMiddle = BNO08X_I2C(i2c_1, None, 0x4A)
# bnoRing = BNO08X_I2C(i2c_1, None, 0x4B)

bnoRef.enable_feature(BNO_REPORT_ROTATION_VECTOR)
bnoIndex.enable_feature(BNO_REPORT_ROTATION_VECTOR)
# bnoMiddle.enable_feature(BNO_REPORT_ROTATION_VECTOR)
# bnoRing.enable_feature(BNO_REPORT_ROTATION_VECTOR)

#bnoRef.enable_feature(BNO_REPORT_ACCELEROMETER)
#bnoIndex.enable_feature(BNO_REPORT_ACCELEROMETER)

# print("waiting cs low")
# 
# while cs.value:
#     pass
# 
# print("detected cs low")

while True:
#     while !cs:
#         pass
    
    handQuaternion = Quaternion(bnoRef.quaternion[3],bnoRef.quaternion[0],bnoRef.quaternion[1],bnoRef.quaternion[2])
    indexQuaternion = Quaternion(bnoIndex.quaternion[3],bnoIndex.quaternion[0],bnoIndex.quaternion[1],bnoIndex.quaternion[2])
#     middleQuaternion = Quaternion(bnoMiddle.quaternion[3],bnoMiddle.quaternion[0],bnoMiddle.quaternion[1],bnoMiddle.quaternion[2])
#     ringQuaternion = Quaternion(bnoRing.quaternion[3],bnoRing.quaternion[0],bnoRing.quaternion[1],bnoRing.quaternion[2])
    
    indexToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(indexQuaternion))
#     middleToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(middleQuaternion))
#     ringToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(ringQuaternion))
    
    indexCurlAmount = getCurl(indexToHandQuaternion)
#     middleCurlAmount = getCurl(middleToHandQuaternion)
#     ringCurlAmount = getCurl(ringToHandQuaternion)
#     curlAmount = getCurl(handQuaternion)
    print(indexCurlAmount*180/3.14)
    time.sleep(.1)
    
    #accel_x1, accel_y1, accel_z1 = bno1.acceleration  # pylint:disable=no-member
    #print("1 X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x1, accel_y1, accel_z1))
    #quat_i1, quat_j1, quat_k1, quat_real1 = bno1.quaternion
    #print(
    #  "1 I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i1, quat_j1, quat_k1, quat_real1)
    #)
    #rotationMatrixRef = quaternion_rotation_matrix(bnoRef.quaternion)
    #print(rotationMatrixRef)
    #eulerAngles = rotationMatrixToEulerAngles(rotationMatrixRef)
    #print(eulerAngles)
    
    #accel_x2, accel_y2, accel_z2 = bno2.acceleration  # pylint:disable=no-member
    #print("2 X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x2, accel_y2, accel_z2))
    #quat_i2, quat_j2, quat_k2, quat_real2 = bno2.quaternion
    #print(
    #  "2 I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i2, quat_j2, quat_k2, quat_real2)
    #)
    #rotationMatrixIndex = quaternion_rotation_matrix(bnoIndex.quaternion)
    #print(rotationMatrixIndex)
    
    #rotationMatrixIndexLocal = rotationMatrixIndex - rotationMatrixRef
    #print(rotationMatrixIndexLocal)
    
    #eulerAngles = rotationMatrixToEulerAngles(rotationMatrixIndexLocal)
    #print(eulerAngles)