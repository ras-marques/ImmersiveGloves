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

# confirmed ok
class Quaternion:
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

# confirmed ok
def quaternion_conjugate(q):
    return Quaternion(q.w, -q.x, -q.y, -q.z)

# confirmed ok
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

thumbActive = False
indexActive = True
middleActive = True
ringActive = True
pinkyActive = True

i2c_0 = 0
i2c_1 = 0
i2c_2 = 0
i2c_3 = 0
i2c_4 = 0
i2c_5 = 0
bnoRef = 0
bnoThumb = 0
bnoIndex = 0
bnoMiddle = 0
bnoRing = 0
bnoPinky = 0

i2c_0 = bitbangio.I2C(board.GP16, board.GP17, frequency = 400000, timeout = 8000)
bnoRef = BNO08X_I2C(i2c_0, None, 0x4B)
bnoRef.enable_feature(BNO_REPORT_ROTATION_VECTOR)

if thumbActive:
    i2c_1 = bitbangio.I2C(board.GP10, board.GP11, frequency = 400000, timeout = 8000)
    bnoIndex = BNO08X_I2C(i2c_1, None, 0x4B)
    bnoIndex.enable_feature(BNO_REPORT_ROTATION_VECTOR)
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

spi_protocol_rev = 1
frame_id = 0
report_mode = 3         # MI_PROTOCOL_REVISION_GENERIC
status = 0              # 0 normal, 1 bootloader
input_data_length = 10  # 10 bytes, 80 bits, see a bit below
backchannel_length = 0  # only for RX, probably for haptics that I am not using yet (YET!)
event_data_length = 0   # only for RX, probably for haptics that I am not using yet (YET!)
reserved = 0

index_axis = 0           # 10  0
middle_axis = 0          # 10  10
ring_axis = 0            # 10  20
pinky_axis = 0           # 10  30
index_splay_axis = 0     # 10  40
middle_splay_axis = 0    # 10  50
ring_splay_axis = 0      # 10  60
pinky_splay_axis = 0     # 10  70
# 80 bits are 10 bytes

first_header_32bits  = (spi_protocol_rev << 24) + (frame_id << 16) + (report_mode << 8) + (status << 0)
second_header_32bits = (input_data_length << 24) + (backchannel_length << 16) + (event_data_length << 8) + (reserved << 0)

# during my tests, for some reason the finger curl was only working if the reference IMU quaternion is the one in the definition below.
# so for everything to work, I define the quaternion below and rotate everything to this coordinate frame.
# for that, I take the relative quaternion from the reference IMU to this quaternion and then use this relative quaternion to rotate every IMU into this reference frame
# then, finally, I do all my calculations.
handQuaternionThatWorks = Quaternion(np.sqrt(2)/2,0,0,-np.sqrt(2)/2)

while True:
    handQuaternion = Quaternion(bnoRef.quaternion[3],bnoRef.quaternion[0],bnoRef.quaternion[1],bnoRef.quaternion[2])                    # get the reference IMU quaternion
    relativeQuaternion = quaternion_multiply(handQuaternionThatWorks, quaternion_conjugate(handQuaternion))                             # get the relative quaternion between the reference IMU quaternion and the coordinate frame where my calculations work
    handQuaternion = quaternion_multiply(relativeQuaternion, handQuaternion)                                                            # rotate the handQuaternion to be in the coordinate frame where my calculations work

    if indexActive:
        indexQuaternion = Quaternion(bnoIndex.quaternion[3],bnoIndex.quaternion[0],bnoIndex.quaternion[1],bnoIndex.quaternion[2])       # get the index IMU quaternion
        indexQuaternion = quaternion_multiply(relativeQuaternion, indexQuaternion)                                                      # rotate the indexQuaternion to be in the coordinate frame where my calculations work
        indexToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(indexQuaternion))                              # get the relative quaternion between the reference IMU quaternion and the index IMU quaternion
        indexCurlAmount = getCurl(indexToHandQuaternion)                                                                                # get the curl angle in radians from the quaternion calculated above
        indexAngle = int(indexCurlAmount*180/3.14)                                                                                      # convert the curl angle to degrees
        
        # remap the angle output in degrees to a value between 0 and 1023: the following calculations were a bit hacky when I implemented them and I didn't document them properly, still, they were based on observations of the outputs and you may be able to reverse engineer an explanation
        if indexAngle <= 0 and indexAngle >= -180:
            index_axis = int(682 * -indexAngle / 180)
        elif indexAngle <= 180 and indexAngle >= 90:
            index_axis = int(682 - (341 / 90) * (indexAngle - 180))
    
    if middleActive:
        middleQuaternion = Quaternion(bnoMiddle.quaternion[3],bnoMiddle.quaternion[0],bnoMiddle.quaternion[1],bnoMiddle.quaternion[2])  # get the middle IMU quaternion
        middleQuaternion = quaternion_multiply(relativeQuaternion, middleQuaternion)                                                    # rotate the middleQuaternion to be in the coordinate frame where my calculations work
        middleToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(middleQuaternion))                            # get the relative quaternion between the reference IMU quaternion and the middle IMU quaternion
        middleCurlAmount = getCurl(middleToHandQuaternion)                                                                              # get the curl angle in radians from the quaternion calculated above
        middleAngle = int(middleCurlAmount*180/3.14)                                                                                    # convert the curl angle to degrees
        
        # remap the angle output in degrees to a value between 0 and 1023: the following calculations were a bit hacky when I implemented them and I didn't document them properly, still, they were based on observations of the outputs and you may be able to reverse engineer an explanation
        if middleAngle <= 0 and middleAngle >= -180:
            middle_axis = int(682 * -middleAngle / 180)
        elif middleAngle <= 180 and middleAngle >= 90:
            middle_axis = int(682 - (341 / 90) * (middleAngle - 180))
    
    if ringActive:
        ringQuaternion = Quaternion(bnoRing.quaternion[3],bnoRing.quaternion[0],bnoRing.quaternion[1],bnoRing.quaternion[2])            # get the ring IMU quaternion
        ringQuaternion = quaternion_multiply(relativeQuaternion, ringQuaternion)                                                        # rotate the ringQuaternion to be in the coordinate frame where my calculations work
        ringToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(ringQuaternion))                                # get the relative quaternion between the reference IMU quaternion and the ring IMU quaternion
        ringCurlAmount = getCurl(ringToHandQuaternion)                                                                                  # get the curl angle in radians from the quaternion calculated above
        ringAngle = int(ringCurlAmount*180/3.14)                                                                                        # convert the curl angle to degrees
        
        # remap the angle output in degrees to a value between 0 and 1023: the following calculations were a bit hacky when I implemented them and I didn't document them properly, still, they were based on observations of the outputs and you may be able to reverse engineer an explanation
        if ringAngle <= 0 and ringAngle >= -180:
            ring_axis = int(682 * -ringAngle / 180)
        elif ringAngle <= 180 and ringAngle >= 90:
            ring_axis = int(682 - (341 / 90) * (ringAngle - 180))

    if pinkyActive:
        pinkyQuaternion = Quaternion(bnoPinky.quaternion[3],bnoPinky.quaternion[0],bnoPinky.quaternion[1],bnoPinky.quaternion[2])       # get the pinky IMU quaternion
        pinkyQuaternion = quaternion_multiply(relativeQuaternion, pinkyQuaternion)                                                      # rotate the pinkyQuaternion to be in the coordinate frame where my calculations work
        pinkyToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(pinkyQuaternion))                              # get the relative quaternion between the reference IMU quaternion and the pinky IMU quaternion
        pinkyCurlAmount = getCurl(pinkyToHandQuaternion)                                                                                # get the curl angle in radians from the quaternion calculated above
        pinkyAngle = int(pinkyCurlAmount*180/3.14)                                                                                      # convert the curl angle to degrees
        
        # remap the angle output in degrees to a value between 0 and 1023: the following calculations were a bit hacky when I implemented them and I didn't document them properly, still, they were based on observations of the outputs and you may be able to reverse engineer an explanation
        if pinkyAngle <= 0 and pinkyAngle >= -180:
            pinky_axis = int(682 * -pinkyAngle / 180)
        elif pinkyAngle <= 180 and pinkyAngle >= 90:
            pinky_axis = int(682 - (341 / 90) * (pinkyAngle - 180))
    
    # I did a lot of tweaking without knowing exactly what was going on, but I compared the original signals from the development board with the ones I generated using the rp2040 PIO, and figured I needed to invert the bit order for the tundra tracker to receive the SPI communication correctly
    # the next few lines take of this inversion and the header construction
    index_axis_inverted = 0
    middle_axis_inverted = 0
    ring_axis_inverted = 0
    pinky_axis_inverted = 0
    index_splay_axis_inverted = 0
    middle_splay_axis_inverted = 0
    ring_splay_axis_inverted = 0
    pinky_splay_axis_inverted = 0
    for i in range(10):
        bit = (index_axis >> i) & 1  # Get the ith bit from the original_value
        index_axis_inverted |= (bit << (9 - i))  # Set the bit in the reversed_value
        bit = (middle_axis >> i) & 1  # Get the ith bit from the original_value
        middle_axis_inverted |= (bit << (9 - i))  # Set the bit in the reversed_value
        bit = (ring_axis >> i) & 1  # Get the ith bit from the original_value
        ring_axis_inverted |= (bit << (9 - i))  # Set the bit in the reversed_value
        bit = (pinky_axis >> i) & 1  # Get the ith bit from the original_value
        pinky_axis_inverted |= (bit << (9 - i))  # Set the bit in the reversed_value
        bit = (index_splay_axis >> i) & 1  # Get the ith bit from the original_value
        index_splay_axis_inverted |= (bit << (9 - i))  # Set the bit in the reversed_value
        bit = (middle_splay_axis >> i) & 1  # Get the ith bit from the original_value
        middle_splay_axis_inverted |= (bit << (9 - i))  # Set the bit in the reversed_value
        bit = (ring_splay_axis >> i) & 1  # Get the ith bit from the original_value
        ring_splay_axis_inverted |= (bit << (9 - i))  # Set the bit in the reversed_value
        bit = (pinky_splay_axis >> i) & 1  # Get the ith bit from the original_value
        pinky_splay_axis_inverted |= (bit << (9 - i))  # Set the bit in the reversed_value
    
    first_header_32bits  = (spi_protocol_rev << 24) + (frame_id << 16) + (report_mode << 8) + (status << 0)
    first_data_32_bits  = (index_axis_inverted << 22) + (middle_axis_inverted << 12) + (ring_axis_inverted << 2) + (ring_axis_inverted >> 8)
    second_data_32_bits  = (pinky_axis_inverted << 24) + (index_splay_axis_inverted << 14) + (middle_splay_axis_inverted << 4) + (ring_splay_axis_inverted >> 6)
    third_data_32_bits  = (ring_splay_axis_inverted << 26) + (pinky_splay_axis_inverted << 16)
    
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
    
    sm.write(fullframe)      # write the fullframe to the PIO
    time.sleep(0.001)        # wait for 1 ms
    frame_id = frame_id + 1  # increment the frame_id
    if frame_id > 255:       # be sure to keep frame_id below 256 to fit one byte
        frame_id = 0