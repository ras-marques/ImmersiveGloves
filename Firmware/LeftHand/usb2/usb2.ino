#include "BNO085.h"
#include "Quaternion.h"
#include <cmath>

#define THUMB_CONTACT_PIN 22
#define INDEX_CONTACT_PIN 23
#define MIDDLE_CONTACT_PIN 24
#define RING_CONTACT_PIN 25
#define PINKY_CONTACT_PIN 26

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

// this will be sent by serial to the main MCU - 14 bytes are enough for 112 bits, this struct has 108
typedef struct __attribute__( ( packed, aligned( 1 ) ) )
{
  uint8_t       pinky           : 1;  //0
  uint8_t       ring            : 1;  //1
  uint8_t       middle          : 1;  //2
  uint8_t       index           : 1;  //3
  uint16_t      thumbstick_x    : 10; //4
  uint16_t      thumbstick_y    : 10; //14
  uint16_t      thumbCurl       : 10; //24
  uint16_t      thumbSplay      : 10; //34
  int16_t       refQuaternion_w : 16; //44
  int16_t       refQuaternion_x : 16; //60
  int16_t       refQuaternion_y : 16; //76
  int16_t       refQuaternion_z : 16; //92
} serial_data_t;
serial_data_t serial_data;

// the setup function runs once when you press reset or power the board
void setup() {

  Serial.begin(115200);
  // while(!Serial) // can't use this here, otherwise it only works with usb connected!

  delay(1000);
  Serial.println("ImmersiveGloves starting...");
  delay(3000); // wait 3 seconds, USB1 only waits 1 second, so that we are sure USB2 has UART aligned for now - probably will make this better later


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

  bnoRef.begin(i2c0, 0x4B);
  bnoThumb.begin(i2c0, 0x4A);
  // bnoIndex.begin(i2c0, 0x4B);
  // bnoMiddle.begin(i2c1, 0x4A);
  // bnoRing.begin(i2c0, 0x4B);
  // bnoPinky.begin(i2c1, 0x4A);
  
  handQuaternionThatWorks.w = sqrt(2)/2;
  handQuaternionThatWorks.x = 0;
  handQuaternionThatWorks.y = 0;
  handQuaternionThatWorks.z = -sqrt(2)/2;

  // for the thumb movement, I needed to get a first neutral position from which to take rotations
  // the relative rotation to the reference IMU is represented by the quaternion below, taken experimentally
  // thumbNeutralToHandQuaternion.w = 0.623351;
  // thumbNeutralToHandQuaternion.x = 0.0097146;
  // thumbNeutralToHandQuaternion.y = 0.754592;
  // thumbNeutralToHandQuaternion.z = 0.204794;

  thumbNeutralToHandQuaternion.w = 0.55;
  thumbNeutralToHandQuaternion.x = -0.39;
  thumbNeutralToHandQuaternion.y = 0.69;
  thumbNeutralToHandQuaternion.z = -0.27;

  // for the joystick emulation, I need a neutral rotation from which to take roll and pitch and map it into joystick values
  // the relative rotation to the reference IMU is represented by the quaternion below, taken experimentally
  // thumbNeutralJoystickToHandQuaternion.w = 0.586889;
  // thumbNeutralJoystickToHandQuaternion.x = -0.168701;
  // thumbNeutralJoystickToHandQuaternion.y = 0.779479;
  // thumbNeutralJoystickToHandQuaternion.z = 0.140207;
  thumbNeutralJoystickToHandQuaternion.w = 0.55;
  thumbNeutralJoystickToHandQuaternion.x = -0.44;
  thumbNeutralJoystickToHandQuaternion.y = 0.57;
  thumbNeutralJoystickToHandQuaternion.z = -0.43;

  // _gpio_init(22);
  // gpio_set_dir(22,false); // set GPIO22 as input
  pinMode(THUMB_CONTACT_PIN,OUTPUT);
  digitalWrite(THUMB_CONTACT_PIN, false);
  pinMode(INDEX_CONTACT_PIN,INPUT_PULLUP);
  pinMode(MIDDLE_CONTACT_PIN,INPUT_PULLUP);
  pinMode(RING_CONTACT_PIN,INPUT_PULLUP);
  pinMode(PINKY_CONTACT_PIN,INPUT_PULLUP);
}

bool joystickIsEnabled = false;
int joystick_x = 512;
int joystick_y = 512;
int thumb_axis = 0;
int thumb_splay_axis = 0;

int millisLast = 0;
bool ledState = true;

// the loop function runs over and over again forever
void loop() {
  // while(1){
  //   if(!digitalRead(INDEX_CONTACT_PIN)) Serial.print("index ");
  //   if(!digitalRead(MIDDLE_CONTACT_PIN)) Serial.print("middle ");
  //   if(!digitalRead(RING_CONTACT_PIN)) Serial.print("ring ");
  //   if(!digitalRead(PINKY_CONTACT_PIN)) Serial.print("pinky ");
  //   Serial.println("");
  // }

  // if(millis() - millisLast > 1000){
  //   ledState = !ledState;
  //   // Serial.println(ledState);
  //   // gpio_put(25, ledState);
  //   digitalWrite(LED_BUILTIN, ledState);  // turn the LED on (HIGH is the voltage level)
  //   millisLast = millis();
  // }

  bnoRef.getReadings();
  bnoThumb.getReadings();

  if(bnoRef.hasNewQuaternion){
    // Serial.println("REF");
    float radAccuracy;
    uint8_t accuracy;
    bnoRef.getQuat(handQuaternion.x, handQuaternion.y, handQuaternion.z, handQuaternion.w, radAccuracy, accuracy);
    // handQuaternion.printMe();
    relativeQuaternion = quaternion_multiply(handQuaternionThatWorks, quaternion_conjugate(handQuaternion)); // get the relative quaternion between the reference IMU quaternion and the coordinate frame where my calculations work
    handQuaternion = quaternion_multiply(relativeQuaternion, handQuaternion);                                           // rotate the handQuaternion to be in the coordinate frame where my calculations work
    // handQuaternion.printMe();
  }

  if(bnoThumb.hasNewQuaternion){
    // Serial.println("THUMB");
    float radAccuracy;
    uint8_t accuracy;
    bnoThumb.getQuat(thumbQuaternion.x, thumbQuaternion.y, thumbQuaternion.z, thumbQuaternion.w, radAccuracy, accuracy);           // get the thumb IMU quaternion
    //thumbQuaternion.printMe();
    thumbQuaternion = quaternion_multiply(relativeQuaternion, thumbQuaternion);                                           // rotate the thumbQuaternion to be in the coordinate frame where my calculations work
    // thumbQuaternion.printMe();
    Quaternion thumbQuaternionBackupForJoystick;
    thumbQuaternionBackupForJoystick.w = thumbQuaternion.w;
    thumbQuaternionBackupForJoystick.x = thumbQuaternion.x;
    thumbQuaternionBackupForJoystick.y = thumbQuaternion.y;
    thumbQuaternionBackupForJoystick.z = thumbQuaternion.z;
    // thumbQuaternionBackupForJoystick.printMe();
    
    // weirdly I don't need this, but don't know why
    Quaternion thumbToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(thumbQuaternion));        // get the relative quaternion between the reference IMU quaternion and the thumb IMU quaternion
    
    thumbQuaternion = quaternion_multiply(thumbNeutralToHandQuaternion, thumbQuaternion);                                 // rotate the relative quaternion between the reference IMU quaternion and the thumb IMU quaternion by the neutral thumb quaternion (is it? I don't know if I did exactly this, but it works)
    float thumbCurlAmount = getCurl(thumbQuaternion);
    int thumbAngle = (int)(thumbCurlAmount*180/3.14);
    thumbAngle = thumbAngle - 40;
    if(thumbAngle < -100) thumbAngle = 140;
    if(thumbAngle < 0) thumbAngle = 0;
    // Serial.println(thumbAngle);
    thumb_axis = 1023*thumbAngle/140.;
    // Serial.println(thumb_axis);
    
    Quaternion thumbCurlQuaternion = quaternionFromAngle(thumbCurlAmount, 0);                                                         // create a quaternion that represents just the amount of curl in the x axis
    Quaternion thumbDecurledQuaternion = quaternion_multiply(thumbCurlQuaternion, thumbQuaternion);                                   // rotate the indexQuaternion by the curl angle in the x axis        
    Quaternion thumbDecurledToNeutralQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(thumbDecurledQuaternion)); // get the relative quaternion between the reference IMU quaternion and the quaternion representing the index IMU rotated back by the curl angle
    float thumbSplayAmount = getSplay(thumbDecurledToNeutralQuaternion);                                                                // get the splay angle in radians from the quaternion calculated above
    int thumbSplayAngle = (int)(thumbSplayAmount*180/3.14);                                                                           // convert the splay angle to degrees
    // Serial.println(thumbSplayAngle);
    if(thumbSplayAngle > 0) thumbSplayAngle = 180 - thumbSplayAngle;
    else thumbSplayAngle = - 180 - thumbSplayAngle;
    thumbSplayAngle += 35;
    if(thumbSplayAngle > 70) thumbSplayAngle = 70;
    else if(thumbSplayAngle < 0) thumbSplayAngle = 0;
    thumb_splay_axis = 1023.*thumbSplayAngle/70;
    // Serial.println(thumb_splay_axis);

    // thumb_splay_axis = (int)(512+512*thumbSplayAngle/42.);
    // if (thumb_splay_axis < 0)
    //   thumb_splay_axis = 0;
    // else if (thumb_splay_axis > 1023)
    //   thumb_splay_axis = 1023;
    //Serial.print(thumb_splay_axis)
    
    // inverted logic, this is when the joystick is enabled
    if (!digitalRead(INDEX_CONTACT_PIN)){
      // Serial.println("index");
      // if(!joystickIsEnabled){
        //thumbNeutralJoystickToHandQuaternion = thumbToHandQuaternion # fix this value on the top and don't reset it each time joystick is reenabled
        //thumbToHandQuaternion.printMe()
        
        Quaternion thumbJoystickQuaternion = quaternion_multiply(thumbNeutralJoystickToHandQuaternion, thumbQuaternionBackupForJoystick); //thumbQuaternionBackupForJoystick.rotateBy(thumbNeutralJoystickToHandQuaternion)
        
        float joystick_x_angleRadians = getRoll(thumbJoystickQuaternion);
        // Serial.println(joystick_x_angleRadians);
        float joystick_x_angleDegrees = joystick_x_angleRadians*180/3.14;
        Serial.println(joystick_x_angleDegrees); // -150 to -20
        joystick_x_angleDegrees += 20;
        // Serial.println(joystick_x_angleDegrees);
        joystick_x = -joystick_x_angleDegrees*1023/130.;
        // Serial.println(joystick_x);
        if (joystick_x < 0)
            joystick_x = 0;
        else if (joystick_x > 1023)
            joystick_x = 1023;
        
        float joystick_y_angleRadians = getCurl(thumbJoystickQuaternion);
        float joystick_y_angleDegrees = joystick_y_angleRadians*180/3.14;
        // Serial.println(joystick_y_angleDegrees); //90 to 110
        joystick_y_angleDegrees -= 90;
        joystick_y = joystick_y_angleDegrees*1023/20.;
        if (joystick_y < 0)
            joystick_y = 0;
        else if (joystick_y > 1023)
            joystick_y = 1023;
        
        joystickIsEnabled = true;
      // }
    }
    else{
      joystick_x = 512;
      joystick_y = 512;
      
      joystickIsEnabled = false;
    }
    
    // Serial.print("x: ");
    // Serial.print(joystick_x);
    // Serial.print("\ty: ");
    // Serial.println(joystick_y);
  }

  serial_data.pinky = !digitalRead(PINKY_CONTACT_PIN);
  serial_data.ring = !digitalRead(RING_CONTACT_PIN);
  serial_data.middle = !digitalRead(MIDDLE_CONTACT_PIN);
  serial_data.index = !digitalRead(INDEX_CONTACT_PIN);
  serial_data.thumbstick_x = joystick_x;
  serial_data.thumbstick_y = joystick_y;
  serial_data.thumbCurl = thumb_axis;
  serial_data.thumbSplay = thumb_splay_axis;
  serial_data.refQuaternion_w = (int)(32767. * relativeQuaternion.w);
  serial_data.refQuaternion_x = (int)(32767. * relativeQuaternion.x);
  serial_data.refQuaternion_y = (int)(32767. * relativeQuaternion.y);
  serial_data.refQuaternion_z = (int)(32767. * relativeQuaternion.z);

  // relativeQuaternion.printMe();

  // Serial.print(serial_data.pinky);
  // Serial.print("\t");
  // Serial.print(serial_data.ring);
  // Serial.print("\t");
  // Serial.print(serial_data.middle);
  // Serial.print("\t");
  // Serial.print(serial_data.index);
  // Serial.print("\t");
  // Serial.print(serial_data.thumbstick_x);
  // Serial.print("\t");
  // Serial.print(serial_data.thumbstick_y);
  // Serial.print("\t");
  // Serial.print(serial_data.thumbCurl);
  // Serial.print("\t");
  // Serial.print(serial_data.thumbSplay);
  // Serial.print("\t");
  // Serial.print(serial_data.refQuaternion_w);
  // Serial.print("\t");
  // Serial.print(serial_data.refQuaternion_x);
  // Serial.print("\t");
  // Serial.print(serial_data.refQuaternion_y);
  // Serial.print("\t");
  // Serial.println(serial_data.refQuaternion_z);

  uint8_t *data_ptr = (uint8_t *)&serial_data;
  size_t data_size = sizeof(serial_data_t);
  uart_putc_raw(uart1, 170); // Print initiator
  uart_putc_raw(uart1, 170); // Print initiator
  // Serial.print(170);
  // Serial.print("\t");
  // Serial.print(170);
  // Serial.print("\t");
  for (size_t i = 0; i < data_size; i++) {
    uart_putc_raw(uart1, data_ptr[i]); // Print each byte as a two-digit hexadecimal number
    // Serial.print(data_ptr[i]);
    // Serial.print("\t");
  }
  // Serial.println("");
}