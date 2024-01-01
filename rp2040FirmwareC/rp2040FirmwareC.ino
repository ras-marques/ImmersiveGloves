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
Quaternion thumbQuaternion;
Quaternion indexQuaternion;
Quaternion middleQuaternion;
Quaternion ringQuaternion;
Quaternion pinkyQuaternion;
Quaternion relativeQuaternion;

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
  gpio_set_function(4, GPIO_FUNC_I2C); // set pin 4 as an I2C pin (SDA in this case)
  gpio_set_function(5, GPIO_FUNC_I2C); // set pin 5 as an I2C pin (SCL in this case)
  gpio_pull_up(4);                     // use internal pull up on pin 4
  gpio_pull_up(5);                     // use internal pull up on pin 5
  Serial.println("I2C0 configured");

  _i2c_init(i2c1, 400000);             // Init I2C1 peripheral at 400kHz
  gpio_set_function(14, GPIO_FUNC_I2C); // set pin 2 as an I2C pin (SDA in this case)
  gpio_set_function(15, GPIO_FUNC_I2C); // set pin 3 as an I2C pin (SCL in this case)
  gpio_pull_up(14);                     // use internal pull up on pin 2
  gpio_pull_up(15);                     // use internal pull up on pin 3
  Serial.println("I2C1 configured");
  
  delay(1000);                         // Give the IMUs time to boot up

  bnoRef.begin(i2c0, 0x4A);
  // bnoThumb.begin(i2c1, 0x4B);
  bnoIndex.begin(i2c1, 0x4B);
  // bnoMiddle.begin(i2c1, 0x4B);
  // bnoRing.begin(i2c1, 0x4B);
  // bnoPinky.begin(i2c1, 0x4B);
  
  handQuaternionThatWorks.w = sqrt(2)/2;
  handQuaternionThatWorks.x = 0;
  handQuaternionThatWorks.y = 0;
  handQuaternionThatWorks.z = -sqrt(2)/2;

  // pinMode(LED_BUILTIN, OUTPUT);
}

bool thumbActive = false;
bool indexActive = true;
bool middleActive = false;
bool ringActive = false;
bool pinkyActive = false;

// the loop function runs over and over again forever
void loop() {
  bnoRef.getReadings();
  // bnoThumb.getReadings();
  bnoIndex.getReadings();

  if(bnoRef.hasNewGameQuaternion){
    uint8_t accuracy;
    bnoRef.getGameQuat(handQuaternion.x, handQuaternion.y, handQuaternion.z, handQuaternion.w, accuracy);
    // handQuaternion.printMe();
    relativeQuaternion = quaternion_multiply(handQuaternionThatWorks, quaternion_conjugate(handQuaternion)); // get the relative quaternion between the reference IMU quaternion and the coordinate frame where my calculations work
    handQuaternion = quaternion_multiply(relativeQuaternion, handQuaternion);                                           // rotate the handQuaternion to be in the coordinate frame where my calculations work
    // handQuaternion.printMe();
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