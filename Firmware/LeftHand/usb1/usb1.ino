#include "BNO085.h"
#include "Quaternion.h"
#include <cmath>

BNO085 bnoIndex;
BNO085 bnoMiddle;
BNO085 bnoRing;
BNO085 bnoPinky;

Quaternion handQuaternion;
Quaternion relativeQuaternion;

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

// this will be sent by serial to the main MCU - 14 bytes are enough for 112 bits, this struct has 107
typedef struct __attribute__( ( packed, aligned( 1 ) ) )
{
  uint8_t       a               : 1;  //0
  uint8_t       b               : 1;  //1
  uint8_t       thumbstick_en   : 1;  //2
  uint16_t      thumbstick_x    : 10; //3
  uint16_t      thumbstick_y    : 10; //13
  uint16_t      thumbCurl       : 10; //23
  uint16_t      thumbSplay      : 10; //33
  int16_t       refQuaternion_w : 16; //43
  int16_t       refQuaternion_x : 16; //59
  int16_t       refQuaternion_y : 16; //75
  int16_t       refQuaternion_z : 16; //91
} serial_data_t;

union serialDataUnion {
  uint8_t chars_rxed_array[14];
  serial_data_t rxed_data;
} serial_data;

// Make a top level struct that packs the button data along with the rest of the controller analog values
typedef struct __attribute__( ( packed, aligned( 1 ) ) )
{
  uint16_t      thumb_curl      : 10;  //0
  uint16_t      index_curl      : 10;  //10
  uint16_t      middle_curl     : 10;  //20
  uint16_t      ring_curl       : 10;  //30
  uint16_t      pinky_curl      : 10;  //40
  uint16_t      thumb_splay     : 10;  //50
  uint16_t      index_splay     : 10;  //60
  uint16_t      middle_splay    : 10;  //70
  uint16_t      ring_splay      : 10;  //80
  uint16_t      pinky_splay     : 10;  //90
  uint16_t      thumbstick_x    : 10;  //100
  uint16_t      thumbstick_y    : 10;  //110
  uint8_t       thumbstick_en   : 1;   //120
  uint8_t       a               : 1;   //121
  uint8_t       b               : 1;   //122
} controller_data_t;
controller_data_t controller_data;

int chars_rxed = 0;
int uart_successes = 0;
int last_received_char = 0;
// RX interrupt handler
void on_uart_rx() {
  while (uart_is_readable(uart1)) {
      uint8_t received_char = uart_getc(uart1);
      if(received_char == 170 && last_received_char == 170){
        chars_rxed = 0;
        continue;
      }
      last_received_char = received_char;
      serial_data.chars_rxed_array[chars_rxed] = received_char;
      chars_rxed++;
      if(chars_rxed == 14){
        controller_data.thumb_curl = serial_data.rxed_data.thumbCurl;
        controller_data.thumb_splay = serial_data.rxed_data.thumbSplay;
        controller_data.thumbstick_x = serial_data.rxed_data.thumbstick_x;
        controller_data.thumbstick_y = serial_data.rxed_data.thumbstick_y;
        controller_data.thumbstick_en = serial_data.rxed_data.thumbstick_en;
        controller_data.a = serial_data.rxed_data.a;
        controller_data.b = serial_data.rxed_data.b;
        relativeQuaternion.w = serial_data.rxed_data.refQuaternion_w / 32767.;
        relativeQuaternion.x = serial_data.rxed_data.refQuaternion_x / 32767.;
        relativeQuaternion.y = serial_data.rxed_data.refQuaternion_y / 32767.;
        relativeQuaternion.z = serial_data.rxed_data.refQuaternion_z / 32767.;
        chars_rxed = 0;
        uart_successes++;
        break;
      }
  }
}

// the setup function runs once when you press reset or power the board
void setup() {

  Serial.begin(115200);
  // while(!Serial) // can't use this here, otherwise it only works with usb connected!

  delay(1000);
  Serial.println("ImmersiveGloves starting...");
  delay(1000); // wait 1 seconds, USB2 waits 3 seconds, so that we are sure USB2 has UART aligned for now - probably will make this better later

  // Initialize I2C
  _i2c_init(i2c0, 400000);             // Init I2C0 peripheral at 400kHz
  gpio_set_function(0, GPIO_FUNC_I2C); // set pin 0 as an I2C pin (SDA in this case)
  gpio_set_function(1, GPIO_FUNC_I2C); // set pin 1 as an I2C pin (SCL in this case)
  gpio_pull_up(0);                     // use internal pull up on pin 0
  gpio_pull_up(1);                     // use internal pull up on pin 1
  Serial.println("I2C0 configured");

  _i2c_init(i2c1, 400000);             // Init I2C1 peripheral at 400kHz
  gpio_set_function(2, GPIO_FUNC_I2C); // set pin 2 as an I2C pin (SDA in this case)
  gpio_set_function(3, GPIO_FUNC_I2C); // set pin 3 as an I2C pin (SCL in this case)
  gpio_pull_up(2);                     // use internal pull up on pin 2
  gpio_pull_up(3);                     // use internal pull up on pin 3
  Serial.println("I2C1 configured");

  handQuaternion.w = sqrt(2)/2;
  handQuaternion.x = 0;
  handQuaternion.y = 0;
  handQuaternion.z = -sqrt(2)/2;
  
  delay(1000);                         // Give the IMUs time to boot up

  // Set up our UART with the required speed.
  uart_init(uart1, 115200);
  // Set the TX and RX pins by using the function select on the GPIO
  // Set datasheet for more information on function select
  gpio_set_function(4, GPIO_FUNC_UART);
  gpio_set_function(5, GPIO_FUNC_UART);

  // Turn off FIFO's - we want to do this character by character
  uart_set_fifo_enabled(uart1, false);

  // Set up and enable the interrupt handlers
  irq_set_exclusive_handler(UART1_IRQ, on_uart_rx);
  irq_set_enabled(UART1_IRQ, true);

  // Now enable the UART to send interrupts - RX only
  uart_set_irq_enables(uart1, true, false);

  // Reference functions below:
  // uart_putc_raw(uart1, 'A');           // Send out a character without any conversions
  // uart_putc(uart1, 'B');               // Send out a character but do CR/LF conversions
  // uart_puts(uart1, " Hello, UART!\n"); // Send out a string, with CR/LF conversions

  bnoIndex.begin(i2c0, 0x4A);
  bnoMiddle.begin(i2c0, 0x4B);
  bnoRing.begin(i2c1, 0x4B);
  bnoPinky.begin(i2c1, 0x4A);

  pinMode(LED_BUILTIN, OUTPUT);
}

bool joystickIsEnabled = false;
int joystick_x = 512;
int joystick_y = 512;
int thumb_axis = 0;
int thumb_splay_axis = 0;
int index_axis = 0;
int index_splay_axis = 0;
int middle_axis = 0;
int middle_splay_axis = 0;
int ring_axis = 0;
int ring_splay_axis = 0;
int pinky_axis = 0;
int pinky_splay_axis = 0;

int millisLast = 0;
bool ledState = true;

// the loop function runs over and over again forever
void loop() {
  // if(millis() - millisLast > 10){
  //   ledState = !ledState;
  //   // Serial.println(ledState);
  //   // gpio_put(25, ledState);
  //   digitalWrite(LED_BUILTIN, ledState);  // turn the LED on (HIGH is the voltage level)
  //   millisLast = millis();
  //   // for(int i = 0; i < 14; i++){
  //   //   Serial.print(serial_data.chars_rxed_array[i]);
  //   //   Serial.print("\t");
  //   // }
  //   // Serial.println("");

  //   // Serial.print(controller_data.thumb_curl);
  //   // Serial.print(",");
  //   // Serial.println(controller_data.thumb_splay);
  //   // relativeQuaternion.printMe();
  // }

  Serial.println(millis() - millisLast);
  millisLast = millis();

  bnoIndex.getReadings();
  bnoMiddle.getReadings();
  bnoRing.getReadings();
  bnoPinky.getReadings();

  if(bnoIndex.hasNewGameQuaternion){
    uint8_t accuracy;
    bnoIndex.getGameQuat(indexQuaternion.x, indexQuaternion.y, indexQuaternion.z, indexQuaternion.w, accuracy);                    // get the index IMU quaternion
    // indexQuaternion.printMe();
    indexQuaternion = quaternion_multiply(relativeQuaternion, indexQuaternion);                                                    // rotate the indexQuaternion to be in the coordinate frame where my calculations work
    Quaternion indexToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(indexQuaternion));                 // get the relative quaternion between the reference IMU quaternion and the index IMU quaternion
    // indexToHandQuaternion.printMe();

    float indexCurlAmount = getCurl(indexToHandQuaternion);                                                                        // get the curl angle in radians from the quaternion calculated above
    // Serial.println(indexCurlAmount);
    int indexAngle = (int)(indexCurlAmount*180/3.14);                                                                              // convert the curl angle to degrees
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
    if (indexSplayAngle <= 18)
      index_splay_axis = (int)(512+512*(indexSplayAngle-18)/40.);
    else
      index_splay_axis = (int)(512+512*(indexSplayAngle-18)/17.);
    if (index_splay_axis < 0)
      index_splay_axis = 0;
    else if (index_splay_axis > 1023)
      index_splay_axis = 1023;
  }

  if(bnoMiddle.hasNewGameQuaternion){
    uint8_t accuracy;
    bnoMiddle.getGameQuat(middleQuaternion.x, middleQuaternion.y, middleQuaternion.z, middleQuaternion.w, accuracy);                    // get the middle IMU quaternion
    // middleQuaternion.printMe();
    middleQuaternion = quaternion_multiply(relativeQuaternion, middleQuaternion);                                                    // rotate the middleQuaternion to be in the coordinate frame where my calculations work
    Quaternion middleToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(middleQuaternion));                 // get the relative quaternion between the reference IMU quaternion and the middle IMU quaternion
    // middleToHandQuaternion.printMe();

    float middleCurlAmount = getCurl(middleToHandQuaternion);                                                                        // get the curl angle in radians from the quaternion calculated above
    // Serial.println(middleCurlAmount);
    int middleAngle = (int)(middleCurlAmount*180/3.14);                                                                              // convert the curl angle to degrees
    // remap the angle output in degrees to a value between 0 and 1023: the following calculations were a bit hacky when I implemented them and I didn't document them properly, still, they were based on observations of the outputs and you may be able to reverse engineer an explanation
    if (middleAngle <= 0 && middleAngle >= -180)
      middle_axis = (int)(682 * -middleAngle / 180.);
    else if (middleAngle <= 180 && middleAngle >= 90)
      middle_axis = (int)(682 - (341. / 90) * (middleAngle - 180));
    
    Quaternion middleCurlQuaternion = quaternionFromAngle(middleCurlAmount, 0);                                                      // create a quaternion that represents just the amount of curl in the x axis
    Quaternion middleDecurledQuaternion = quaternion_multiply(middleCurlQuaternion, middleQuaternion);                                // rotate the middleQuaternion by the curl angle in the x axis
    Quaternion middleDecurledToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(middleDecurledQuaternion)); // get the relative quaternion between the reference IMU quaternion and the quaternion representing the middle IMU rotated back by the curl angle
    float middleSplayAmount = getSplay(middleDecurledToHandQuaternion);                                                              // get the splay angle in radians from the quaternion calculated above
    int middleSplayAngle = (int)(middleSplayAmount*180/3.14);                                                                        // convert the splay angle to degrees
    //Serial.print(middleSplayAngle);
    // finger   min  rest  max
    // middle    -11    4   28
    if (middleSplayAngle <= 4)
      middle_splay_axis = (int)(512+512*(middleSplayAngle-4)/15.);
    else
      middle_splay_axis = (int)(512+512*(middleSplayAngle-4)/24.);
    if (middle_splay_axis < 0)
      middle_splay_axis = 0;
    else if (middle_splay_axis > 1023)
      middle_splay_axis = 1023;
  }

  if(bnoRing.hasNewGameQuaternion){
    uint8_t accuracy;
    bnoRing.getGameQuat(ringQuaternion.x, ringQuaternion.y, ringQuaternion.z, ringQuaternion.w, accuracy);                    // get the ring IMU quaternion
    // ringQuaternion.printMe();
    ringQuaternion = quaternion_multiply(relativeQuaternion, ringQuaternion);                                                    // rotate the ringQuaternion to be in the coordinate frame where my calculations work
    Quaternion ringToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(ringQuaternion));                 // get the relative quaternion between the reference IMU quaternion and the ring IMU quaternion
    // ringToHandQuaternion.printMe();

    float ringCurlAmount = getCurl(ringToHandQuaternion);                                                                        // get the curl angle in radians from the quaternion calculated above
    // Serial.println(ringCurlAmount);
    int ringAngle = (int)(ringCurlAmount*180/3.14);                                                                              // convert the curl angle to degrees
    // remap the angle output in degrees to a value between 0 and 1023: the following calculations were a bit hacky when I implemented them and I didn't document them properly, still, they were based on observations of the outputs and you may be able to reverse engineer an explanation
    if (ringAngle <= 0 && ringAngle >= -180)
      ring_axis = (int)(682 * -ringAngle / 180.);
    else if (ringAngle <= 180 && ringAngle >= 90)
      ring_axis = (int)(682 - (341. / 90) * (ringAngle - 180));
    
    Quaternion ringCurlQuaternion = quaternionFromAngle(ringCurlAmount, 0);                                                      // create a quaternion that represents just the amount of curl in the x axis
    Quaternion ringDecurledQuaternion = quaternion_multiply(ringCurlQuaternion, ringQuaternion);                                // rotate the ringQuaternion by the curl angle in the x axis
    Quaternion ringDecurledToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(ringDecurledQuaternion)); // get the relative quaternion between the reference IMU quaternion and the quaternion representing the ring IMU rotated back by the curl angle
    float ringSplayAmount = getSplay(ringDecurledToHandQuaternion);                                                              // get the splay angle in radians from the quaternion calculated above
    int ringSplayAngle = (int)(ringSplayAmount*180/3.14);                                                                        // convert the splay angle to degrees
    //Serial.print(ringSplayAngle);
    // finger   min  rest  max
    // ring     -14    -5   14
    if (ringSplayAngle <= -5)
      ring_splay_axis = (int)(512+512*(ringSplayAngle+5)/9.);
    else
      ring_splay_axis = (int)(512+512*(ringSplayAngle+5)/21.);
    if (ring_splay_axis < 0)
      ring_splay_axis = 0;
    else if (ring_splay_axis > 1023)
      ring_splay_axis = 1023;
  }

  if(bnoPinky.hasNewGameQuaternion){
    uint8_t accuracy;
    bnoPinky.getGameQuat(pinkyQuaternion.x, pinkyQuaternion.y, pinkyQuaternion.z, pinkyQuaternion.w, accuracy);                    // get the pinky IMU quaternion
    // pinkyQuaternion.printMe();
    pinkyQuaternion = quaternion_multiply(relativeQuaternion, pinkyQuaternion);                                                    // rotate the pinkyQuaternion to be in the coordinate frame where my calculations work
    Quaternion pinkyToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(pinkyQuaternion));                 // get the relative quaternion between the reference IMU quaternion and the pinky IMU quaternion
    // pinkyToHandQuaternion.printMe();

    float pinkyCurlAmount = getCurl(pinkyToHandQuaternion);                                                                        // get the curl angle in radians from the quaternion calculated above
    // Serial.println(pinkyCurlAmount);
    int pinkyAngle = (int)(pinkyCurlAmount*180/3.14);                                                                              // convert the curl angle to degrees
    // remap the angle output in degrees to a value between 0 and 1023: the following calculations were a bit hacky when I implemented them and I didn't document them properly, still, they were based on observations of the outputs and you may be able to reverse engineer an explanation
    if (pinkyAngle <= 0 && pinkyAngle >= -180)
      pinky_axis = (int)(682 * -pinkyAngle / 180.);
    else if (pinkyAngle <= 180 && pinkyAngle >= 90)
      pinky_axis = (int)(682 - (341. / 90) * (pinkyAngle - 180));
    
    Quaternion pinkyCurlQuaternion = quaternionFromAngle(pinkyCurlAmount, 0);                                                      // create a quaternion that represents just the amount of curl in the x axis
    Quaternion pinkyDecurledQuaternion = quaternion_multiply(pinkyCurlQuaternion, pinkyQuaternion);                                // rotate the pinkyQuaternion by the curl angle in the x axis
    Quaternion pinkyDecurledToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(pinkyDecurledQuaternion)); // get the relative quaternion between the reference IMU quaternion and the quaternion representing the pinky IMU rotated back by the curl angle
    float pinkySplayAmount = getSplay(pinkyDecurledToHandQuaternion);                                                              // get the splay angle in radians from the quaternion calculated above
    int pinkySplayAngle = (int)(pinkySplayAmount*180/3.14);                                                                        // convert the splay angle to degrees
    //Serial.print(pinkySplayAngle);
    // finger   min  rest  max
    // pinky    -40    25   -2
    if (pinkySplayAngle <= 10)
      pinky_splay_axis = (int)(512+512*(pinkySplayAngle+25)/15.);
    else
      pinky_splay_axis = (int)(512+512*(pinkySplayAngle+25)/23.);
    if (pinky_splay_axis < 0)
      pinky_splay_axis = 0;
    else if (pinky_splay_axis > 1023)
      pinky_splay_axis = 1023;
  }

  Serial.print(thumb_axis);
  Serial.print("\t");
  Serial.print(thumb_splay_axis);
  Serial.print("\t");
  Serial.print(index_axis);
  Serial.print("\t");
  Serial.print(index_splay_axis);
  Serial.print("\t");
  Serial.print(middle_axis);
  Serial.print("\t");
  Serial.print(middle_splay_axis);
  Serial.print("\t");
  Serial.print(ring_axis);
  Serial.print("\t");
  Serial.print(ring_splay_axis);
  Serial.print("\t");
  Serial.print(pinky_axis);
  Serial.print("\t");
  Serial.println(pinky_splay_axis);
}