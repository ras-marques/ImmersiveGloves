import array
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
#from adafruit_bno08x import BNO_REPORT_ACCELEROMETER
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from ulab import numpy as np
import time
import rp2pio
import adafruit_pioasm

assembled = adafruit_pioasm.assemble("""
.program spi_slave
.wrap_target:
    set x, 31
    pull
loop:
    wait 0 pin 0
    wait 0 pin 1
    out pins 1
    wait 1 pin 1
    jmp x-- loop
.wrap
""")

sm = rp2pio.StateMachine(
    assembled,
    frequency=0,
    first_out_pin=board.GP12,
    out_pin_count=1,
    first_in_pin=board.GP13,
    in_pin_count=3,
)

print(sm.frequency)

frame_id = 0
first_header_32bits  = (1 << 24) + (frame_id << 16) + (3 << 8) + (0 << 0)
second_header_32bits = (10 << 24) + (0 << 16) + (0 << 8) + (0 << 0)
fullframe = array.array('L',[first_header_32bits, second_header_32bits, 0, 0, 255])

while True:
    first_header_32bits  = (1 << 24) + (frame_id << 16) + (3 << 8) + (0 << 0)
    fullframe = array.array('L',[first_header_32bits, second_header_32bits, 0, 0, 255])
    #fullframe = array.array('L',[first_header_32bits])
    print(fullframe)
    sm.write(fullframe)
    time.sleep(0.01)
    frame_id = frame_id + 1
    if frame_id > 255:
        frame_id = 0

# def quaternion_rotation_matrix(Q):
#     """
#     Covert a quaternion into a full three-dimensional rotation matrix.
#  
#     Input
#     :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
#  
#     Output
#     :return: A 3x3 element matrix representing the full 3D rotation matrix. 
#              This rotation matrix converts a point in the local reference 
#              frame to a point in the global reference frame.
#     """
#     # Extract the values from Q
#     q0 = Q[0]
#     q1 = Q[1]
#     q2 = Q[2]
#     q3 = Q[3]
#      
#     # First row of the rotation matrix
#     r00 = 2 * (q0 * q0 + q1 * q1) - 1
#     r01 = 2 * (q1 * q2 - q0 * q3)
#     r02 = 2 * (q1 * q3 + q0 * q2)
#      
#     # Second row of the rotation matrix
#     r10 = 2 * (q1 * q2 + q0 * q3)
#     r11 = 2 * (q0 * q0 + q2 * q2) - 1
#     r12 = 2 * (q2 * q3 - q0 * q1)
#      
#     # Third row of the rotation matrix
#     r20 = 2 * (q1 * q3 - q0 * q2)
#     r21 = 2 * (q2 * q3 + q0 * q1)
#     r22 = 2 * (q0 * q0 + q3 * q3) - 1
#      
#     # 3x3 rotation matrix
#     rot_matrix = np.array([[r00, r01, r02],
#                            [r10, r11, r12],
#                            [r20, r21, r22]])
#                             
#     return rot_matrix
# 
# def rotationMatrixToEulerAngles(R) :
#     sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
#  
#     singular = sy < 1e-6
#  
#     if  not singular :
#         x = np.arctan2(R[2,1] , R[2,2])
#         y = np.arctan2(-R[2,0], sy)
#         z = np.arctan2(R[1,0], R[0,0])
#     else :
#         x = np.arctan2(-R[1,2], R[1,1])
#         y = np.arctan2(-R[2,0], sy)
#         z = 0
#  
#     return np.array([x*180/3.14, y*180/3.14, z*180/3.14])
# 
# i2c = busio.I2C(board.GP5, board.GP4)
# bno1 = BNO08X_I2C(i2c, None, 0x4A)
# #bno1.enable_feature(BNO_REPORT_ACCELEROMETER)
# bno1.enable_feature(BNO_REPORT_ROTATION_VECTOR)
# bno2 = BNO08X_I2C(i2c, None, 0x4B)
# #bno2.enable_feature(BNO_REPORT_ACCELEROMETER)
# bno2.enable_feature(BNO_REPORT_ROTATION_VECTOR)
# 
# while True:
#     #accel_x1, accel_y1, accel_z1 = bno1.acceleration  # pylint:disable=no-member
#     #print("1 X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x1, accel_y1, accel_z1))
#     #quat_i1, quat_j1, quat_k1, quat_real1 = bno1.quaternion
#     #print(
#     #  "1 I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i1, quat_j1, quat_k1, quat_real1)
#     #)
#     rotationMatrixRef = quaternion_rotation_matrix(bno1.quaternion)
#     #print(rotationMatrixRef)
#     #eulerAngles = rotationMatrixToEulerAngles(rotationMatrixRef)
#     #print(eulerAngles)
#     
#     #accel_x2, accel_y2, accel_z2 = bno2.acceleration  # pylint:disable=no-member
#     #print("2 X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x2, accel_y2, accel_z2))
#     #quat_i2, quat_j2, quat_k2, quat_real2 = bno2.quaternion
#     #print(
#     #  "2 I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i2, quat_j2, quat_k2, quat_real2)
#     #)
#     rotationMatrixIndex = quaternion_rotation_matrix(bno2.quaternion)
#     #print(rotationMatrixIndex)
#     
#     rotationMatrixIndexLocal = rotationMatrixIndex - rotationMatrixRef
#     #print(rotationMatrixIndexLocal)
#     
#     eulerAngles = rotationMatrixToEulerAngles(rotationMatrixIndexLocal)
#     print(eulerAngles)
#     
#     time.sleep(.5)