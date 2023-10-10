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
import bitbangio

# probepin = adafruit_pioasm.assemble("""
# .program spidebug
# .wrap_target:
#     wait 0 pin 0
#     set pins 0
#     wait 1 pin 0
#     set pins 1
# .wrap
# """)
# 
# sm12 = rp2pio.StateMachine(
#     probepin,
#     frequency=0,
#     first_in_pin=board.GP12,
#     in_pin_count=1,
#     first_set_pin=board.GP9,
#     set_pin_count=1
# )
# 
# sm13 = rp2pio.StateMachine(
#     probepin,
#     frequency=0,
#     first_in_pin=board.GP13,
#     in_pin_count=1,
#     first_set_pin=board.GP10,
#     set_pin_count=1
# )
# 
# sm14 = rp2pio.StateMachine(
#     probepin,
#     frequency=0,
#     first_in_pin=board.GP14,
#     in_pin_count=1,
#     first_set_pin=board.GP11,
#     set_pin_count=1
# )
# 
# sm15 = rp2pio.StateMachine(
#     probepin,
#     frequency=0,
#     first_in_pin=board.GP15,
#     in_pin_count=1,
#     first_set_pin=board.GP16,
#     set_pin_count=1
# )
# 
# while True:
#     pass

# THIS IS THE NOT THE GOOD ONE
# assembled = adafruit_pioasm.assemble("""
# .program spi_slave
# .wrap_target:
#     set x, 31
#     pull
# loop:
#     wait 0 pin 0
#     wait 0 pin 1
#     out pins 1
#     wait 1 pin 1
#     jmp x-- loop
# .wrap
# """)

# sm = rp2pio.StateMachine(
#     assembled,
#     frequency=0,
#     first_out_pin=board.GP12,
#     out_pin_count=1,
#     first_in_pin=board.GP13,
#     in_pin_count=3
# )
# sm = rp2pio.StateMachine(
#     assembled,
#     frequency=0,
#     first_out_pin=board.GP11,
#     out_pin_count=1,
#     first_in_pin=board.GP13,
#     in_pin_count=3,
#     out_shift_right=True
# )

assembled = adafruit_pioasm.assemble("""
.program spi_slave
.wrap_target:
    set x, 31
    pull
loop:
    wait 0 pin 1
    wait 0 pin 2
    out pins 1
    wait 1 pin 2
    jmp x-- loop
.wrap
""")

sm = rp2pio.StateMachine(
    assembled,
    frequency=0,
    first_out_pin=board.GP15,
    out_pin_count=1,
    first_in_pin=board.GP12,
    in_pin_count=3,
    out_shift_right=False
)

print(sm.frequency)

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


# i2c_0 = bitbangio.I2C(board.GP5, board.GP4, frequency = 100000, timeout = 2000)
# i2c_1 = bitbangio.I2C(board.GP7, board.GP6, frequency = 100000, timeout = 2000)
# 
# bnoRef = BNO08X_I2C(i2c_0, None, 0x4B)
# bnoIndex = BNO08X_I2C(i2c_1, None, 0x4A)
# # bnoMiddle = BNO08X_I2C(i2c_2, None, 0x4B)
# # bnoRing = BNO08X_I2C(i2c_3, None, 0x4B)
# # bnoPinky = BNO08X_I2C(i2c_4, None, 0x4B)
# 
# bnoRef.enable_feature(BNO_REPORT_ROTATION_VECTOR)
# bnoIndex.enable_feature(BNO_REPORT_ROTATION_VECTOR)
# bnoMiddle.enable_feature(BNO_REPORT_ROTATION_VECTOR)
# bnoRing.enable_feature(BNO_REPORT_ROTATION_VECTOR)
# bnoPinky.enable_feature(BNO_REPORT_ROTATION_VECTOR)

frame_id = 0
first_header_32bits  = (1 << 24) + (frame_id << 16) + (3 << 8) + (0 << 0)
second_header_32bits = (10 << 24) + (0 << 16) + (0 << 8) + (0 << 0)

system_button = 1        # 1   0
a_button = 1             # 1   1
b_button = 1             # 1   2
trigger_button = 1       # 1   3
grip_button = 0          # 1   4
thumbstick_button = 0    # 1   5
menu_button = 0          # 1   6
thumbstick_enable = 0    # 1   7
thumbstick_x_axis = 0    # 10  8
thumbstick_y_axis = 0    # 10  18
trigger_axis = 90         # 10  28
index_axis = 127           # 10  38
middle_axis = 0          # 10  48
ring_axis = 0            # 10  58
pinky_axis = 0           # 10  68

while True:
#     handQuaternion = Quaternion(bnoRef.quaternion[3],bnoRef.quaternion[0],bnoRef.quaternion[1],bnoRef.quaternion[2])
#     indexQuaternion = Quaternion(bnoIndex.quaternion[3],bnoIndex.quaternion[0],bnoIndex.quaternion[1],bnoIndex.quaternion[2])
# #     middleQuaternion = Quaternion(bnoMiddle.quaternion[3],bnoMiddle.quaternion[0],bnoMiddle.quaternion[1],bnoMiddle.quaternion[2])
# #     ringQuaternion = Quaternion(bnoRing.quaternion[3],bnoRing.quaternion[0],bnoRing.quaternion[1],bnoRing.quaternion[2])
# #     pinkyQuaternion = Quaternion(bnoPinky.quaternion[3],bnoPinky.quaternion[0],bnoPinky.quaternion[1],bnoPinky.quaternion[2])
# 
#     indexToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(indexQuaternion))
# #     middleToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(middleQuaternion))
# #     ringToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(ringQuaternion))
# #     pinkyToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(pinkyQuaternion))
# 
#     indexCurlAmount = getCurl(indexToHandQuaternion)
# #     middleCurlAmount = getCurl(middleToHandQuaternion)
# #     ringCurlAmount = getCurl(ringToHandQuaternion)
# #     pinkyCurlAmount = getCurl(pinkyQuaternion)
#     indexAngle = int(indexCurlAmount*180/3.14)
#     print(indexAngle)
#     
#     if indexAngle <= 0 and indexAngle >= -180:
#         index_axis = int(682 * -indexAngle / 180)
#     elif indexAngle <= 180 and indexAngle >= 90:
#         index_axis = int(682 - (341 / 90) * (indexAngle - 180))
#     
#     print(index_axis)
# #     print(middleCurlAmount*180/3.14)
# #     print(ringCurlAmount*180/3.14)
# #     print(pinkyCurlAmount*180/3.14)
    
    first_header_32bits  = (1 << 24) + (frame_id << 16) + (3 << 8) + (0 << 0)
    first_data_32_bits = (system_button << 31) + (a_button << 30) + (b_button << 29) + (trigger_button << 28) + (grip_button << 27) + (thumbstick_button << 26) + (menu_button << 25) + (thumbstick_enable << 24) + (thumbstick_x_axis << 14) + (thumbstick_y_axis << 4)
    second_data_32_bits = (trigger_axis << 24) + (index_axis << 16)# + (middle_axis << 4) + (ring_axis >> 6)
    print(f"{second_data_32_bits:032b}")
    third_data_32_bits = 0 #((ring_axis << 26) & 0xFFFF) + (pinky_axis << 16)
    
    fullframe = array.array('L',[first_header_32bits, second_header_32bits, first_data_32_bits, second_data_32_bits, third_data_32_bits])
    print(fullframe)
    
    sm.write(fullframe)
    time.sleep(0.01)
    frame_id = frame_id + 1
    if frame_id > 255:
        frame_id = 0