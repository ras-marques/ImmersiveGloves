#include "BNO085.h"
#include "Quaternion.h"
#include <cmath>

BNO085 bnoRef;
BNO085 bnoThumb;

Quaternion handQuaternionThatWorks;
Quaternion handQuaternion;
Quaternion relativeQuaternion;

Quaternion thumbQuaternion;
Quaternion thumbNeutralToHandQuaternion;
Quaternion thumbNeutralJoystickToHandQuaternion;

Quaternion indexQuaternion;
Quaternion middleQuaternion;
Quaternion ringQuaternion;
Quaternion pinkyQuaternion;

float getCurl(Quaternion q){
  return atan2(2 * (q.x * q.w - q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
}

float getSplay(Quaternion q){
  return atan2(2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.y * q.y + q.z * q.z));
}

float getRoll(Quaternion q){
  return atan2(2 * (q.y * q.z + q.w * q.x), 1 - 2 * (q.x * q.x + q.y * q.y));
}

Quaternion quaternionFromAngle(float angle, int axis){
  // angle is in radians, x axis is 0, y axis is 1, z axis is 2
  if (axis == 0){
    Quaternion q;
    q.w = cos(angle/2);
    q.x = sin(angle/2);
    q.y = 0;
    q.z = 0;
    return q;
  }
  if (axis == 1){
    Quaternion q;
    q.w = cos(angle/2);
    q.x = 0;
    q.y = sin(angle/2);
    q.z = 0;
    return q;
  }
  if (axis == 2){
    Quaternion q;
    q.w = cos(angle/2);
    q.x = 0;
    q.y = 0;
    q.z = sin(angle/2);
    return q;
  }
}

// this will be sent by serial to the main MCU - 23 bytes are enough for 184 bits, this struct has 183
typedef struct __attribute__( ( packed, aligned( 1 ) ) )
{
  uint8_t       a               : 1;  //0
  uint8_t       b               : 1;  //1
  uint8_t       thumbstick_en   : 1;  //2
  uint16_t      thumbstick_x    : 10; //3
  uint16_t      thumbstick_y    : 10; //13
  uint16_t      thumbCurl       : 10; //23
  uint16_t      thumbSplay      : 10; //33
  uint16_t      refQuaternion_w : 32; //55
  uint16_t      refQuaternion_x : 32; //87
  uint16_t      refQuaternion_y : 32; //119
  uint16_t      refQuaternion_z : 32; //151
} serial_data_t;
serial_data_t serial_data;

// the setup function runs once when you press reset or power the board
void setup() {

  Serial.begin(115200);
  while(!Serial)

  delay(1000);
  Serial.println("ImmersiveGloves starting...");


  // Initialize I2C
  _i2c_init(i2c0, 400000);             // Init I2C0 peripheral at 400kHz
  gpio_set_function(0, GPIO_FUNC_I2C); // set pin 4 as an I2C pin (SDA in this case)
  gpio_set_function(1, GPIO_FUNC_I2C); // set pin 5 as an I2C pin (SCL in this case)
  gpio_pull_up(0);                     // use internal pull up on pin 4
  gpio_pull_up(1);                     // use internal pull up on pin 5
  Serial.println("I2C0 configured");

  // _i2c_init(i2c1, 400000);             // Init I2C1 peripheral at 400kHz
  // gpio_set_function(14, GPIO_FUNC_I2C); // set pin 2 as an I2C pin (SDA in this case)
  // gpio_set_function(15, GPIO_FUNC_I2C); // set pin 3 as an I2C pin (SCL in this case)
  // gpio_pull_up(14);                     // use internal pull up on pin 2
  // gpio_pull_up(15);                     // use internal pull up on pin 3
  // Serial.println("I2C1 configured");
  
  // Set up our UART with the required speed.
  uart_init(uart1, 115200);
  // Set the TX and RX pins by using the function select on the GPIO
  // Set datasheet for more information on function select
  gpio_set_function(4, GPIO_FUNC_UART);
  gpio_set_function(5, GPIO_FUNC_UART);

  // Reference functions below:
  // uart_putc_raw(uart1, 'A');           // Send out a character without any conversions
  // uart_putc(uart1, 'B');               // Send out a character but do CR/LF conversions
  // uart_puts(uart1, " Hello, UART!\n"); // Send out a string, with CR/LF conversions

  delay(1000);                         // Give the IMUs time to boot up

  bnoRef.begin(i2c0, 0x4A);
  bnoThumb.begin(i2c0, 0x4B);
  // bnoIndex.begin(i2c0, 0x4B);
  // bnoMiddle.begin(i2c1, 0x4B);
  // bnoRing.begin(i2c1, 0x4B);
  // bnoPinky.begin(i2c1, 0x4B);
  
  handQuaternionThatWorks.w = sqrt(2)/2;
  handQuaternionThatWorks.x = 0;
  handQuaternionThatWorks.y = 0;
  handQuaternionThatWorks.z = -sqrt(2)/2;

  // for the thumb movement, I needed to get a first neutral position from which to take rotations
  // the relative rotation to the reference IMU is represented by the quaternion below, taken experimentally
  thumbNeutralToHandQuaternion.w = 0.623351;
  thumbNeutralToHandQuaternion.x = 0.0097146;
  thumbNeutralToHandQuaternion.y = 0.754592;
  thumbNeutralToHandQuaternion.z = 0.204794;

  // for the joystick emulation, I need a neutral rotation from which to take roll and pitch and map it into joystick values
  // the relative rotation to the reference IMU is represented by the quaternion below, taken experimentally
  thumbNeutralJoystickToHandQuaternion.w = 0.586889;
  thumbNeutralJoystickToHandQuaternion.x = -0.168701;
  thumbNeutralJoystickToHandQuaternion.y = 0.779479;
  thumbNeutralJoystickToHandQuaternion.z = 0.140207;

  gpio_set_dir(22,false); // set GPIO22 as input
}

bool thumbActive = true;
bool indexActive = false;
bool middleActive = false;
bool ringActive = false;
bool pinkyActive = false;

bool joystickIsEnabled = false;
int joystick_x = 512;
int joystick_y = 512;
int thumb_axis = 0;
int thumb_splay_axis = 0;

// the loop function runs over and over again forever
void loop() {
  bnoRef.getReadings();
  bnoThumb.getReadings();

  if(bnoRef.hasNewGameQuaternion){
    uint8_t accuracy;
    bnoRef.getGameQuat(handQuaternion.x, handQuaternion.y, handQuaternion.z, handQuaternion.w, accuracy);
    // handQuaternion.printMe();
    relativeQuaternion = quaternion_multiply(handQuaternionThatWorks, quaternion_conjugate(handQuaternion)); // get the relative quaternion between the reference IMU quaternion and the coordinate frame where my calculations work
    handQuaternion = quaternion_multiply(relativeQuaternion, handQuaternion);                                           // rotate the handQuaternion to be in the coordinate frame where my calculations work
    // handQuaternion.printMe();
  }

  if(bnoThumb.hasNewGameQuaternion){
    uint8_t accuracy;
    bnoThumb.getGameQuat(thumbQuaternion.x, thumbQuaternion.y, thumbQuaternion.z, thumbQuaternion.w, accuracy);           // get the thumb IMU quaternion
    thumbQuaternion = quaternion_multiply(relativeQuaternion, thumbQuaternion);                                           // rotate the thumbQuaternion to be in the coordinate frame where my calculations work
    Quaternion thumbQuaternionBackupForJoystick = thumbQuaternion;
    
    // weirdly I don't need this, but don't know why
    Quaternion thumbToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(thumbQuaternion));        // get the relative quaternion between the reference IMU quaternion and the thumb IMU quaternion
    
    thumbQuaternion = quaternion_multiply(thumbNeutralToHandQuaternion, thumbQuaternion);                                 // rotate the relative quaternion between the reference IMU quaternion and the thumb IMU quaternion by the neutral thumb quaternion (is it? I don't know if I did exactly this, but it works)
    float thumbCurlAmount = getCurl(thumbQuaternion);
    int thumbAngle = (int)(thumbCurlAmount*180/3.14);
    int thumb_axis = (int)(512+512*thumbAngle/90.);
    if (thumb_axis < 0)
        thumb_axis = 0;
    else if (thumb_axis > 1023)
        thumb_axis = 1023;
    //Serial.print(thumb_axis);
    
    Quaternion thumbCurlQuaternion = quaternionFromAngle(thumbCurlAmount, 0);                                                         // create a quaternion that represents just the amount of curl in the x axis
    Quaternion thumbDecurledQuaternion = quaternion_multiply(thumbCurlQuaternion, thumbQuaternion);                                   // rotate the indexQuaternion by the curl angle in the x axis        
    Quaternion thumbDecurledToNeutralQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(thumbDecurledQuaternion)); // get the relative quaternion between the reference IMU quaternion and the quaternion representing the index IMU rotated back by the curl angle
    int thumbSplayAmount = getSplay(thumbDecurledToNeutralQuaternion);                                                                // get the splay angle in radians from the quaternion calculated above
    int thumbSplayAngle = (int)(thumbSplayAmount*180/3.14);                                                                           // convert the splay angle to degrees
    thumb_splay_axis = (int)(512+512*thumbSplayAngle/42.);
    if (thumb_splay_axis < 0)
      thumb_splay_axis = 0;
    else if (thumb_splay_axis > 1023)
      thumb_splay_axis = 1023;
    //Serial.print(thumb_splay_axis)
    
    // inverted logic, this is when the joystick is enabled
    if (!gpio_get(22)){
      if(!joystickIsEnabled){
        //thumbNeutralJoystickToHandQuaternion = thumbToHandQuaternion # fix this value on the top and don't reset it each time joystick is reenabled
        //thumbToHandQuaternion.printMe()
        
        Quaternion thumbJoystickQuaternion = quaternion_multiply(thumbNeutralJoystickToHandQuaternion, thumbQuaternionBackupForJoystick); //thumbQuaternionBackupForJoystick.rotateBy(thumbNeutralJoystickToHandQuaternion)
        
        int joystick_x_angleRadians = getRoll(thumbJoystickQuaternion);
        int joystick_x_angleDegrees = (int)(joystick_x_angleRadians*180/3.14);
        joystick_x = (int)((joystick_x_angleDegrees + 25)*1023/50.);
        if (joystick_x < 0)
            joystick_x = 0;
        else if (joystick_x > 1023)
            joystick_x = 1023;
        
        int joystick_y_angleRadians = getCurl(thumbJoystickQuaternion);
        int joystick_y_angleDegrees = (int)(joystick_y_angleRadians*180/3.14);
        joystick_y = (int)((joystick_y_angleDegrees + 25)*1023/50);
        if (joystick_y < 0)
            joystick_y = 0;
        else if (joystick_y > 1023)
            joystick_y = 1023;
        
        joystickIsEnabled = true;
      }
    }
    else{
      joystick_x = 512;
      joystick_y = 512;
      
      joystickIsEnabled = false;
    }
    
    Serial.print("x: ");
    Serial.println(joystick_x);
    Serial.print("y: ");
    Serial.println(joystick_y);
  }

  serial_data.thumbstick_en = joystickIsEnabled;
  serial_data.thumbstick_x = joystick_x;
  serial_data.thumbstick_y = joystick_y;
  serial_data.thumbCurl = thumb_axis;
  serial_data.thumbSplay = thumb_splay_axis;
  serial_data.refQuaternion_w = (int)(32767. * relativeQuaternion.w);
  serial_data.refQuaternion_x = (int)(32767. * relativeQuaternion.x);
  serial_data.refQuaternion_y = (int)(32767. * relativeQuaternion.y);
  serial_data.refQuaternion_z = (int)(32767. * relativeQuaternion.z);
  uint8_t *data_ptr = (uint8_t *)&serial_data;
  size_t data_size = sizeof(serial_data_t);
  for (size_t i = 0; i < data_size; i++) {
    uart_putc_raw(uart1, data_ptr[i]); // Print each byte as a two-digit hexadecimal number
  }
}