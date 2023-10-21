import array
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
#from adafruit_bno08x import BNO_REPORT_ACCELEROMETER
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
# import pyquaternion
from ulab import numpy as np
import time
import rp2pio
import adafruit_pioasm
import bitbangio
import usb_cdc

serial = usb_cdc.data

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
    q0 = Q.w
    q1 = Q.x
    q2 = Q.y
    q3 = Q.z
     
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

indexActive = True
middleActive = False
ringActive = False
pinkyActive = False

i2c_0 = 0
i2c_1 = 0
i2c_2 = 0
i2c_3 = 0
i2c_4 = 0
i2c_5 = 0
bnoRef = 0
# bnoThumb = BNO08X_I2C(i2c_1, None, 0x4B)
bnoIndex = 0
bnoMiddle = 0
bnoRing = 0
bnoPinky = 0

i2c_0 = bitbangio.I2C(board.GP16, board.GP17, frequency = 400000, timeout = 8000)
bnoRef = BNO08X_I2C(i2c_0, None, 0x4B)
bnoRef.enable_feature(BNO_REPORT_ROTATION_VECTOR)

# i2c_1 = bitbangio.I2C(board.GP10, board.GP11, frequency = 100000, timeout = 2000)
# bnoThumb = BNO08X_I2C(i2c_1, None, 0x4B)
# bnoThumb.enable_feature(BNO_REPORT_ROTATION_VECTOR)

if indexActive:
    i2c_2 = bitbangio.I2C(board.GP8, board.GP9, frequency = 400000, timeout = 8000)
    bnoIndex = BNO08X_I2C(i2c_2, None, 0x4B)
    bnoIndex.enable_feature(BNO_REPORT_ROTATION_VECTOR)
if middleActive:
    i2c_3 = bitbangio.I2C(board.GP6, board.GP7, frequency = 400000, timeout = 8000)
    bnoMiddle = BNO08X_I2C(i2c_3, None, 0x4B)
    bnoMiddle.enable_feature(BNO_REPORT_ROTATION_VECTOR)
if ringActive:
    i2c_4 = bitbangio.I2C(board.GP4, board.GP5, frequency = 400000, timeout = 8000)
    bnoRing = BNO08X_I2C(i2c_4, None, 0x4B)
    bnoRing.enable_feature(BNO_REPORT_ROTATION_VECTOR)
if pinkyActive:
    i2c_5 = bitbangio.I2C(board.GP0, board.GP1, frequency = 400000, timeout = 8000)
    bnoPinky = BNO08X_I2C(i2c_5, None, 0x4B)
    bnoPinky.enable_feature(BNO_REPORT_ROTATION_VECTOR)

frame_id = 0
first_header_32bits  = (1 << 24) + (frame_id << 16) + (3 << 8) + (0 << 0)
second_header_32bits = (10 << 24) + (0 << 16) + (0 << 8) + (0 << 0)

system_button = 0        # 1   0
a_button = 0             # 1   1
b_button = 0             # 1   2
trigger_button = 0       # 1   3
grip_button = 0          # 1   4
thumbstick_button = 0    # 1   5
menu_button = 0          # 1   6
thumbstick_enable = 0    # 1   7
thumbstick_x_axis = 0    # 10  8
thumbstick_y_axis = 0    # 10  18
trigger_axis = 0         # 10  28
index_axis = 200           # 10  38
middle_axis = 400          # 10  48
ring_axis = 600            # 10  58
pinky_axis = 800           # 10  68

thumb_axis = 0

while True:
    handQuaternion = Quaternion(bnoRef.quaternion[3],bnoRef.quaternion[0],bnoRef.quaternion[1],bnoRef.quaternion[2])
    string = "{:10.4f}".format(handQuaternion.w) + "{:10.4f}".format(handQuaternion.x) + "{:10.4f}".format(handQuaternion.y) + "{:10.4f}".format(handQuaternion.z)
    print(string)
    serial.write(string.encode())
    serial.write(b'\n\r')
    
    handRotationMatrix = quaternion_rotation_matrix(handQuaternion)
#     euler_angles = rotationMatrixToEulerAngles(rotation_matrix)
#     print(euler_angles)
    
#     thumbQuaternion = Quaternion(bnoThumb.quaternion[3],bnoThumb.quaternion[0],bnoThumb.quaternion[1],bnoThumb.quaternion[2])
#     thumbToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(thumbQuaternion))
#     thumbCurlAmount = getCurl(thumbToHandQuaternion)
#     thumbAngle = int(thumbCurlAmount*180/3.14)
#     if thumbAngle <= 0 and thumbAngle >= -180:
#         thumb_axis = int(682 * -thumbAngle / 180)
#     elif thumbAngle <= 180 and thumbAngle >= 90:
#         thumb_axis = int(682 - (341 / 90) * (thumbAngle - 180))

    if indexActive:
        indexQuaternion = Quaternion(bnoIndex.quaternion[3],bnoIndex.quaternion[0],bnoIndex.quaternion[1],bnoIndex.quaternion[2])
#         indexRotationMatrix = quaternion_rotation_matrix(indexQuaternion)
#         relative_rot_matrix = np.dot(indexRotationMatrix, np.linalg.inv(handRotationMatrix))
#         euler_angles = rotationMatrixToEulerAngles(relative_rot_matrix)
#         print(euler_angles)

#         w_bh, x_bh, y_bh, z_bh = bnoRef.quaternion[3],bnoRef.quaternion[0],bnoRef.quaternion[1],bnoRef.quaternion[2]
#         w_if, x_if, y_if, z_if = bnoIndex.quaternion[3],bnoIndex.quaternion[0],bnoIndex.quaternion[1],bnoIndex.quaternion[2]
#         
#         # Convert quaternions to rotation matrices
#         rot_matrix_bh = np.array([
#             [1 - 2*(y_bh**2 + z_bh**2), 2*(x_bh*y_bh - w_bh*z_bh), 2*(x_bh*z_bh + w_bh*y_bh)],
#             [2*(x_bh*y_bh + w_bh*z_bh), 1 - 2*(x_bh**2 + z_bh**2), 2*(y_bh*z_bh - w_bh*x_bh)],
#             [2*(x_bh*z_bh - w_bh*y_bh), 2*(y_bh*z_bh + w_bh*x_bh), 1 - 2*(x_bh**2 + y_bh**2)]
#         ])
# 
#         rot_matrix_if = np.array([
#             [1 - 2*(y_if**2 + z_if**2), 2*(x_if*y_if - w_if*z_if), 2*(x_if*z_if + w_if*y_if)],
#             [2*(x_if*y_if + w_if*z_if), 1 - 2*(x_if**2 + z_if**2), 2*(y_if*z_if - w_if*x_if)],
#             [2*(x_if*z_if - w_if*y_if), 2*(y_if*z_if + w_if*x_if), 1 - 2*(x_if**2 + y_if**2)]
#         ])
# 
#         # Calculate the relative rotation matrix
#         relative_rot_matrix = np.dot(rot_matrix_if, np.linalg.inv(rot_matrix_bh))
# 
#         # Extract Euler angles (in radians)
#         roll, pitch, yaw = np.arctan2(relative_rot_matrix[2, 1], relative_rot_matrix[2, 2]), -np.asin(relative_rot_matrix[2, 0]), np.arctan2(relative_rot_matrix[1, 0], relative_rot_matrix[0, 0])
#         
#         print(str(roll) + " " + str(pitch) + " " + str(yaw))
        
        indexToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(indexQuaternion))
#         print(str(indexToHandQuaternion.w) + " " + str(indexToHandQuaternion.x) + " " + str(indexToHandQuaternion.y) + " " + str(indexToHandQuaternion.z))
        
#         indexToHandQuaternion = quaternion_multiply(quaternion_conjugate(handQuaternion), indexQuaternion)
#         rotation_matrix = quaternion_rotation_matrix(indexToHandQuaternion)
#         euler_angles = rotationMatrixToEulerAngles(rotation_matrix)
#         print(euler_angles)
        
        indexCurlAmount = getCurl(indexToHandQuaternion)
        indexAngle = int(indexCurlAmount*180/3.14)
        if indexAngle <= 0 and indexAngle >= -180:
            index_axis = int(682 * -indexAngle / 180)
        elif indexAngle <= 180 and indexAngle >= 90:
            index_axis = int(682 - (341 / 90) * (indexAngle - 180))
#         print("indexAngle: " + str(indexAngle))
    
    if middleActive:
        middleQuaternion = Quaternion(bnoMiddle.quaternion[3],bnoMiddle.quaternion[0],bnoMiddle.quaternion[1],bnoMiddle.quaternion[2])
        middleToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(middleQuaternion))
        middleCurlAmount = getCurl(middleToHandQuaternion)
        middleAngle = int(middleCurlAmount*180/3.14)
        if middleAngle <= 0 and middleAngle >= -180:
            middle_axis = int(682 * -middleAngle / 180)
        elif middleAngle <= 180 and middleAngle >= 90:
            middle_axis = int(682 - (341 / 90) * (middleAngle - 180))
#         print("middleAngle: " + str(middleAngle))
    
    if ringActive:
        ringQuaternion = Quaternion(bnoRing.quaternion[3],bnoRing.quaternion[0],bnoRing.quaternion[1],bnoRing.quaternion[2])
        ringToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(ringQuaternion))
        ringCurlAmount = getCurl(ringToHandQuaternion)
        ringAngle = int(ringCurlAmount*180/3.14)
        if ringAngle <= 0 and ringAngle >= -180:
            ring_axis = int(682 * -ringAngle / 180)
        elif ringAngle <= 180 and ringAngle >= 90:
            ring_axis = int(682 - (341 / 90) * (ringAngle - 180))
#         print("ringAngle: " + str(ringAngle))

    if pinkyActive:
        pinkyQuaternion = Quaternion(bnoPinky.quaternion[3],bnoPinky.quaternion[0],bnoPinky.quaternion[1],bnoPinky.quaternion[2])
        pinkyToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(pinkyQuaternion))
        pinkyCurlAmount = getCurl(pinkyQuaternion)
        pinkyAngle = int(pinkyCurlAmount*180/3.14)
        if pinkyAngle <= 0 and pinkyAngle >= -180:
            pinky_axis = int(682 * -pinkyAngle / 180)
        elif pinkyAngle <= 180 and pinkyAngle >= 90:
            pinky_axis = int(682 - (341 / 90) * (pinkyAngle - 180))
#         print("pinkyAngle: " + str(pinkyAngle))
    
    thumbstick_x_axis_inverted = 0
    thumbstick_y_axis_inverted = 0
    trigger_axis_inverted = 0
    index_axis_inverted = 0
    middle_axis_inverted = 0
    ring_axis_inverted = 0
    pinky_axis_inverted = 0
    for i in range(10):
        bit = (thumbstick_x_axis >> i) & 1  # Get the ith bit from the original_value
        thumbstick_x_axis_inverted |= (bit << (9 - i))  # Set the bit in the reversed_value
        bit = (thumbstick_y_axis >> i) & 1  # Get the ith bit from the original_value
        thumbstick_y_axis_inverted |= (bit << (9 - i))  # Set the bit in the reversed_value
        bit = (trigger_axis >> i) & 1  # Get the ith bit from the original_value
        trigger_axis_inverted |= (bit << (9 - i))  # Set the bit in the reversed_value
        bit = (index_axis >> i) & 1  # Get the ith bit from the original_value
        index_axis_inverted |= (bit << (9 - i))  # Set the bit in the reversed_value
        bit = (middle_axis >> i) & 1  # Get the ith bit from the original_value
        middle_axis_inverted |= (bit << (9 - i))  # Set the bit in the reversed_value
        bit = (ring_axis >> i) & 1  # Get the ith bit from the original_value
        ring_axis_inverted |= (bit << (9 - i))  # Set the bit in the reversed_value
        bit = (pinky_axis >> i) & 1  # Get the ith bit from the original_value
        pinky_axis_inverted |= (bit << (9 - i))  # Set the bit in the reversed_value
    
    first_header_32bits = (1 << 24) + (frame_id << 16) + (3 << 8) + (0 << 0)
    first_data_32_bits  = (system_button << 31) + (a_button << 30) + (b_button << 29) + (trigger_button << 28) + (grip_button << 27) + (thumbstick_button << 26) + (menu_button << 25) + (thumbstick_enable << 24) + (thumbstick_x_axis_inverted << 14) + (thumbstick_y_axis_inverted << 4) + (trigger_axis_inverted >> 6)
    second_data_32_bits = (trigger_axis_inverted << 26) + (index_axis_inverted << 16) + (middle_axis_inverted << 6) + (ring_axis_inverted >> 4)
    third_data_32_bits  = (ring_axis_inverted << 28) + (pinky_axis_inverted << 18)
    
    byte1 = ((first_data_32_bits >> 24) & 0xFF)
    byte2 = ((first_data_32_bits >> 16) & 0xFF)
    byte3 = ((first_data_32_bits >> 8) & 0xFF)
    byte4 = ((first_data_32_bits >> 0) & 0xFF)
    byte1Inverted = 0
    byte2Inverted = 0
    byte3Inverted = 0
    byte4Inverted = 0
    
    for i in range(8):
        bit = (byte1 >> i) & 1  # Get the ith bit from the original_value
        byte1Inverted |= (bit << (7 - i))  # Set the bit in the reversed_value
        bit = (byte2 >> i) & 1  # Get the ith bit from the original_value
        byte2Inverted |= (bit << (7 - i))  # Set the bit in the reversed_value
        bit = (byte3 >> i) & 1  # Get the ith bit from the original_value
        byte3Inverted |= (bit << (7 - i))  # Set the bit in the reversed_value
        bit = (byte4 >> i) & 1  # Get the ith bit from the original_value
        byte4Inverted |= (bit << (7 - i))  # Set the bit in the reversed_value
    first_data_32_bits  = (byte1Inverted << 24) + (byte2Inverted << 16) + (byte3Inverted << 8) + (byte4Inverted << 0)
    
    byte1 = ((second_data_32_bits >> 24) & 0xFF)
    byte2 = ((second_data_32_bits >> 16) & 0xFF)
    byte3 = ((second_data_32_bits >> 8) & 0xFF)
    byte4 = ((second_data_32_bits >> 0) & 0xFF)
    byte1Inverted = 0
    byte2Inverted = 0
    byte3Inverted = 0
    byte4Inverted = 0
    for i in range(8):
        bit = (byte1 >> i) & 1  # Get the ith bit from the original_value
        byte1Inverted |= (bit << (7 - i))  # Set the bit in the reversed_value
        bit = (byte2 >> i) & 1  # Get the ith bit from the original_value
        byte2Inverted |= (bit << (7 - i))  # Set the bit in the reversed_value
        bit = (byte3 >> i) & 1  # Get the ith bit from the original_value
        byte3Inverted |= (bit << (7 - i))  # Set the bit in the reversed_value
        bit = (byte4 >> i) & 1  # Get the ith bit from the original_value
        byte4Inverted |= (bit << (7 - i))  # Set the bit in the reversed_value
    second_data_32_bits = (byte1Inverted << 24) + (byte2Inverted << 16) + (byte3Inverted << 8) + (byte4Inverted << 0)

    byte1 = ((third_data_32_bits >> 24) & 0xFF)
    byte2 = ((third_data_32_bits >> 16) & 0xFF)
    byte3 = ((third_data_32_bits >> 8) & 0xFF)
    byte4 = ((third_data_32_bits >> 0) & 0xFF)
    byte1Inverted = 0
    byte2Inverted = 0
    byte3Inverted = 0
    byte4Inverted = 0
    for i in range(8):
        bit = (byte1 >> i) & 1  # Get the ith bit from the original_value
        byte1Inverted |= (bit << (7 - i))  # Set the bit in the reversed_value
        bit = (byte2 >> i) & 1  # Get the ith bit from the original_value
        byte2Inverted |= (bit << (7 - i))  # Set the bit in the reversed_value
        bit = (byte3 >> i) & 1  # Get the ith bit from the original_value
        byte3Inverted |= (bit << (7 - i))  # Set the bit in the reversed_value
        bit = (byte4 >> i) & 1  # Get the ith bit from the original_value
        byte4Inverted |= (bit << (7 - i))  # Set the bit in the reversed_value
    third_data_32_bits = (byte1Inverted << 24) + (byte2Inverted << 16) + (byte3Inverted << 8) + (byte4Inverted << 0)
    
    fullframe = array.array('L',[first_header_32bits, second_header_32bits, first_data_32_bits, second_data_32_bits, third_data_32_bits])
#     print(fullframe)
    
    sm.write(fullframe)
    time.sleep(0.01)
    frame_id = frame_id + 1
    if frame_id > 255:
        frame_id = 0