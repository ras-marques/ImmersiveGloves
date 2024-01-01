#include "BNO085.h"
#include "Quaternion.h"
#include <cmath>

BNO085 bnoRef;
BNO085 bnoThumb;
BNO085 bnoIndex;
BNO085 bnoMiddle;
BNO085 bnoRing;
BNO085 bnoPinky;

Quaternion handQuaternionThatWorks;
// Quaternion handQuaternionThatWorks = Quaternion(sqrt(2)/2,0,0,-sqrt(2)/2);
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

bool joystickIsEnabled = false;
int joystick_x = 512;
int joystick_y = 512;

bool thumbActive = true;
bool indexActive = false;
bool middleActive = false;
bool ringActive = false;
bool pinkyActive = false;

// the loop function runs over and over again forever
void loop() {
  bnoRef.getReadings();
  bnoThumb.getReadings();
  // bnoIndex.getReadings();

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
    int thumb_splay_axis = (int)(512+512*thumbSplayAngle/42.);
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

  if(indexActive && bnoIndex.hasNewGameQuaternion){
    uint8_t accuracy;
    bnoIndex.getGameQuat(indexQuaternion.x, indexQuaternion.y, indexQuaternion.z, indexQuaternion.w, accuracy);                    // get the index IMU quaternion
    // indexQuaternion.printMe();
    indexQuaternion = quaternion_multiply(relativeQuaternion, indexQuaternion);                                                    // rotate the indexQuaternion to be in the coordinate frame where my calculations work
    Quaternion indexToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(indexQuaternion));                 // get the relative quaternion between the reference IMU quaternion and the index IMU quaternion
    // indexToHandQuaternion.printMe();

    float indexCurlAmount = getCurl(indexToHandQuaternion);                                                                        // get the curl angle in radians from the quaternion calculated above
    // Serial.println(indexCurlAmount);
    int indexAngle = (int)(indexCurlAmount*180/3.14);                                                                              // convert the curl angle to degrees
    int index_axis = 0;
    // remap the angle output in degrees to a value between 0 and 1023: the following calculations were a bit hacky when I implemented them and I didn't document them properly, still, they were based on observations of the outputs and you may be able to reverse engineer an explanation
    if (indexAngle <= 0 && indexAngle >= -180)
      index_axis = (int)(682 * -indexAngle / 180.);
    else if (indexAngle <= 180 && indexAngle >= 90)
      index_axis = (int)(682 - (341. / 90) * (indexAngle - 180));
    
    Quaternion indexCurlQuaternion = quaternionFromAngle(indexCurlAmount, 0);                                                      // create a quaternion that represents just the amount of curl in the x axis
    Quaternion indexDecurledQuaternion = quaternion_multiply(indexCurlQuaternion, indexQuaternion);                                // rotate the indexQuaternion by the curl angle in the x axis
    Quaternion indexDecurledToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(indexDecurledQuaternion)); // get the relative quaternion between the reference IMU quaternion and the quaternion representing the index IMU rotated back by the curl angle
    float indexSplayAmount = getSplay(indexDecurledToHandQuaternion);                                                              // get the splay angle in radians from the quaternion calculated above
    int indexSplayAngle = (int)(indexSplayAmount*180/3.14);                                                                        // convert the splay angle to degrees
    //Serial.print(indexSplayAngle);
    // finger   min  rest  max
    // index    -22   18    35
    int index_splay_axis = 0;
    if (indexSplayAngle <= 18)
      index_splay_axis = (int)(512+512*(indexSplayAngle-18)/40.);
    else
      index_splay_axis = (int)(512+512*(indexSplayAngle-18)/17.);
    if (index_splay_axis < 0)
      index_splay_axis = 0;
    else if (index_splay_axis > 1023)
      index_splay_axis = 1023;

    Serial.print(index_axis);
    Serial.print(",");
    Serial.println(index_splay_axis);
  }

  // bool success = receivePacket();
  // if(!success){/*Serial.println("no report available");*/}
  // else Serial.println("report available");
}