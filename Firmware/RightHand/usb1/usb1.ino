// #include "tundra_mapped_input.h"
#include "BNO085.h"
#include "Finger.h"
#include "Quaternion.h"
#include <cmath>

#include <hardware/pio.h>

// Our assembled program:
#include "spi_slave.pio.h"

// Any button interaction in the glove is done by touching the thumb to one of the fingers.
// This makes accidental button presses very frequent. To avoid this, a double click on the A button (ring finger) must be executed.
// This action will toggle inputs on and off, while never disabling hand tracking.
bool disableInputs = true;

// Create TMI object to communicate with Tundra Tracker
// TMI tundra_tracker;
// void csn_irq( uint gpio, uint32_t event_mask );  // Chip select IRQ must be top level so let's make one and connect it later in setup

BNO085 bnoIndex;
BNO085 bnoMiddle;
BNO085 bnoRing;
BNO085 bnoPinky;

Finger fingerIndex;
Finger fingerMiddle;
Finger fingerRing;
Finger fingerPinky;

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

union serialDataUnion {
  uint8_t chars_rxed_array[14];
  serial_data_t rxed_data;
} serial_data;

void printSerialData(){
  Serial.print(serial_data.rxed_data.pinky);
  Serial.print("\t");
  Serial.print(serial_data.rxed_data.ring);
  Serial.print("\t");
  Serial.print(serial_data.rxed_data.middle);
  Serial.print("\t");
  Serial.print(serial_data.rxed_data.index);
  Serial.print("\t");
  Serial.print(serial_data.rxed_data.thumbstick_x);
  Serial.print("\t");
  Serial.print(serial_data.rxed_data.thumbstick_y);
  Serial.print("\t");
  Serial.print(serial_data.rxed_data.thumbCurl);
  Serial.print("\t");
  Serial.print(serial_data.rxed_data.thumbSplay);
  Serial.print("\t");
  Serial.print(serial_data.rxed_data.refQuaternion_w);
  Serial.print("\t");
  Serial.print(serial_data.rxed_data.refQuaternion_x);
  Serial.print("\t");
  Serial.print(serial_data.rxed_data.refQuaternion_y);
  Serial.print("\t");
  Serial.println(serial_data.rxed_data.refQuaternion_z);
}

// Make a top level struct that packs the button data along with the rest of the controller analog values
typedef struct __attribute__( ( packed, aligned( 1 ) ) )
{
  uint16_t      thumb_curl        : 10;  //0
  uint16_t      index_curl        : 10;  //10
  uint16_t      middle_curl       : 10;  //20
  uint16_t      ring_curl         : 10;  //30
  uint16_t      pinky_curl        : 10;  //40
  uint16_t      thumb_splay       : 10;  //50
  uint16_t      index_splay       : 10;  //60
  uint16_t      middle_splay      : 10;  //70
  uint16_t      ring_splay        : 10;  //80
  uint16_t      pinky_splay       : 10;  //90
  uint16_t      trigger           : 10;  //100
  uint16_t      thumbstick_x      : 10;  //110
  uint16_t      thumbstick_y      : 10;  //120
  uint8_t       thumbstick_click  : 1;   //130
  uint8_t       triggerbtn        : 1;   //131
  uint8_t       a                 : 1;   //132
  uint8_t       b                 : 1;   //133
  uint8_t       system            : 1;   //134
} controller_data_t;
controller_data_t controller_data;
// 5 32-bit integers can represent  160 bits. This struct has 135.

uint8_t received_characters[32] = {};

int chars_rxed = 0;
int uart_successes = 0;
int last_received_char = 0;
long microsLastReceived = 0;
bool hasSerialData = false;
// RX interrupt handler
void on_uart_rx() {
  while (uart_is_readable(uart1)) {
    if(micros() - microsLastReceived > 5000){
      chars_rxed = 0;
    }
    received_characters[chars_rxed & 0x1F] = uart_getc(uart1);
    microsLastReceived = micros();
    chars_rxed++;
  }
  if(chars_rxed == 18){
    hasSerialData = true;
    chars_rxed = 0;
  }
}

int debugTransitions = 0;
bool checkDoubleClick(uint64_t button_history){
  if(button_history & 0x1){
    debugTransitions = -1;
    return false; // button still pressed, can't be a double click
  } else{
    // check how many transitions from 0 to 1 and 1 to 0 happened. On a double click, there should be exactly 4
    bool lastState = false;
    int numberOfTransitions = 0;
    for(int i = 0; i < 63; i++){
      bool currentState = button_history & (((uint64_t)1) << i);
      if(lastState != currentState) numberOfTransitions++;
      lastState = currentState;
    }
    debugTransitions = numberOfTransitions;
    if(numberOfTransitions == 4) return true; // double click detected
  }

  return false; // catch all default returning false - no double click detected
}

int interpretErrorType = 0;
int interpretSum = 0;
// comms happen at around 100Hz between USB2 and USB1 and we want to look at the last 500ms or so for double clicks, so we store the last 64 entries of each button in a 64bit int for the last 640ms.
uint64_t thumbstick_btn = 0;
uint64_t a_btn = 0;
uint64_t b_btn = 0;
uint64_t system_btn = 0;
void interpret_serial_data(){
  if((received_characters[0] != 0x55) || (received_characters[1] != 0x55)){
    interpretErrorType = 1;
  	return;
  }
  uint16_t sum = 0;

  for(int i = 0; i < 16; i++){
    sum += received_characters[i];
  }

  interpretSum = sum;

  if(sum != ((received_characters[17] << 8) + received_characters[16])){
    interpretErrorType = 2;
  	return;
  }

  // now that it's confirmed data was received ok, fill serial_data union with the actual data that was received
  for(int i = 0; i < 14; i++){
    serial_data.chars_rxed_array[i] = received_characters[i + 2];
  }

  // finally parse the union serial_data into controller_data that will be sent to the tracker
  controller_data.thumb_curl = serial_data.rxed_data.thumbCurl;
  controller_data.thumb_splay = serial_data.rxed_data.thumbSplay;

  // left shift all FIFOs
  thumbstick_btn = thumbstick_btn << 1;
  a_btn = a_btn << 1;
  b_btn = b_btn << 1;
  system_btn = system_btn << 1;

  thumbstick_btn |= serial_data.rxed_data.index;
  b_btn |= serial_data.rxed_data.middle;
  a_btn |= serial_data.rxed_data.ring;
  system_btn |= serial_data.rxed_data.pinky;

  if(checkDoubleClick(a_btn)) {
    disableInputs = !disableInputs;
    a_btn = 0; // we must reset a_btn here to 0, otherwise there will be multiple double clicks detected until the FIFO is emptied naturally in ~640ms
  }

  if(!disableInputs){
    controller_data.thumbstick_x = serial_data.rxed_data.thumbstick_x;
    controller_data.thumbstick_y = serial_data.rxed_data.thumbstick_y;
    
    controller_data.thumbstick_click = serial_data.rxed_data.index;
    controller_data.b = serial_data.rxed_data.middle;
    controller_data.a = serial_data.rxed_data.ring;
    controller_data.system = serial_data.rxed_data.pinky;

    int tempVariable = controller_data.index_curl * 2;
    if(tempVariable > 1023) controller_data.trigger = 1023;
    else controller_data.trigger = tempVariable;
    if(controller_data.trigger > 850) controller_data.triggerbtn = true;
    else controller_data.triggerbtn = false;
  } else {
    controller_data.thumbstick_x = 512;
    controller_data.thumbstick_y = 512;
    controller_data.thumbstick_click = false;
    controller_data.a = false;
    controller_data.b = false;
    controller_data.system = false;
    controller_data.trigger = 0;
    controller_data.triggerbtn = false;
  }

  relativeQuaternion.w = serial_data.rxed_data.refQuaternion_w / 32767.;
  relativeQuaternion.x = serial_data.rxed_data.refQuaternion_x / 32767.;
  relativeQuaternion.y = serial_data.rxed_data.refQuaternion_y / 32767.;
  relativeQuaternion.z = serial_data.rxed_data.refQuaternion_z / 32767.;
}

PIO pio = pio0;
uint offset;
uint sm;
// the setup function runs once when you press reset or power the board
void setup() {

  Serial.begin(115200);
  // while(!Serial) // can't use this here, otherwise it only works with usb connected!

  delay(2000);
  Serial.println("ImmersiveGloves starting...");
  delay(1000); // wait 1 seconds, USB2 waits 3 seconds, so that we are sure USB2 has UART aligned for now - probably will make this better later

  offset = pio_add_program(pio, &spi_slave_program);
  sm = pio_claim_unused_sm(pio, true);
  int mosi = 12;
  int cs = 13;
  int sclk = 14;
  int miso = 15;
  spi_slave_init(pio, sm, offset, mosi, cs, sclk, miso);

  // init the communication to Tundra Tracker, setup the CS irq callback (this has to be at Top level in arduino it seems)
  // tundra_tracker.init( );
  // gpio_set_irq_enabled_with_callback( tundra_tracker.get_cs_pin(), GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &csn_irq );

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

  delay(1000);                         // Give the IMUs time to boot up

  Serial.println("Initializing Index IMU");
  bnoIndex.begin(i2c1, 0x4A);
  Serial.println("Initializing Middle IMU");
  bnoMiddle.begin(i2c1, 0x4B);
  Serial.println("Initializing Ring IMU");
  bnoRing.begin(i2c0, 0x4A);
  Serial.println("Initializing Pinky IMU");
  bnoPinky.begin(i2c0, 0x4B);

  // sets initial calibration of splay angles for each finger during initialization
  // begin(float degreesMin, float degreesMid, float degreesMax)
  fingerIndex.begin(-30, 5, 25);
  fingerMiddle.begin(5, 19, 30);
  fingerRing.begin(-25, -9, 0);
  fingerPinky.begin(-53, -27, -5);

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

  controller_data.thumb_curl = 530;
  controller_data.index_curl = 550;
  controller_data.middle_curl = 600;
  controller_data.ring_curl = 620;
  controller_data.pinky_curl = 640;
  controller_data.thumb_splay = 530;
  controller_data.index_splay = 550;
  controller_data.middle_splay = 600;
  controller_data.ring_splay = 620;
  controller_data.pinky_splay = 640;
}

void printControllerData(){
  Serial.print(controller_data.thumb_curl);
  Serial.print("\t");
  Serial.print(controller_data.index_curl);
  Serial.print("\t");
  Serial.print(controller_data.middle_curl);
  Serial.print("\t");
  Serial.print(controller_data.ring_curl);
  Serial.print("\t");
  Serial.print(controller_data.pinky_curl);
  Serial.print("\t");
  Serial.print(controller_data.thumb_splay);
  Serial.print("\t");
  Serial.print(controller_data.index_splay);
  Serial.print("\t");
  Serial.print(controller_data.middle_splay);
  Serial.print("\t");
  Serial.print(controller_data.ring_splay);
  Serial.print("\t");
  Serial.print(controller_data.pinky_splay);
  Serial.print("\t");
  Serial.print(controller_data.thumbstick_x);
  Serial.print("\t");
  Serial.print(controller_data.thumbstick_y);
  Serial.print("\t");
  Serial.print(controller_data.thumbstick_click);
  Serial.print("\t");
  Serial.print(controller_data.a);
  Serial.print("\t");
  Serial.print(controller_data.b);
  Serial.print("\t");
  Serial.print(controller_data.system);
  Serial.print("\t");
  Serial.print(controller_data.trigger);
  Serial.print("\t");
  Serial.println(controller_data.triggerbtn);
}

int indexCurl_angleDegrees, middleCurl_angleDegrees, ringCurl_angleDegrees, pinkyCurl_angleDegrees;
int indexSplay_angleDegrees, middleSplay_angleDegrees, ringSplay_angleDegrees, pinkySplay_angleDegrees;

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

int millisCallibrationStart = 0;
bool preparingCalibration = false;
bool runningCalibration = false;
void runSplayCalibration(){
  static bool calibrating = false;
  static float minIndexSplay = 360., maxIndexSplay = -360.;
  static float minMiddleSplay = 360., maxMiddleSplay = -360.;
  static float minRingSplay = 360., maxRingSplay = -360.;
  static float minPinkySplay = 360., maxPinkySplay = -360.;
  
  if(!calibrating){
    // Serial.println("calibration started!");
    millisCallibrationStart = millis();
    calibrating = true;
  }
  
  if(fingerIndex.splayDegrees < minIndexSplay) minIndexSplay = fingerIndex.splayDegrees;
  if(fingerIndex.splayDegrees > maxIndexSplay) maxIndexSplay = fingerIndex.splayDegrees;
  if(fingerMiddle.splayDegrees < minMiddleSplay) minMiddleSplay = fingerMiddle.splayDegrees;
  if(fingerMiddle.splayDegrees > maxMiddleSplay) maxMiddleSplay = fingerMiddle.splayDegrees;
  if(fingerRing.splayDegrees < minRingSplay) minRingSplay = fingerRing.splayDegrees;
  if(fingerRing.splayDegrees > maxRingSplay) maxRingSplay = fingerRing.splayDegrees;
  if(fingerPinky.splayDegrees < minPinkySplay) minPinkySplay = fingerPinky.splayDegrees;
  if(fingerPinky.splayDegrees > maxPinkySplay) maxPinkySplay = fingerPinky.splayDegrees;

  if(millis() - millisCallibrationStart > 30000) {
    fingerIndex.calibrateSplayDegrees(minIndexSplay, fingerIndex.splayDegrees, maxIndexSplay);
    fingerMiddle.calibrateSplayDegrees(minMiddleSplay, fingerMiddle.splayDegrees, maxMiddleSplay);
    fingerRing.calibrateSplayDegrees(minRingSplay, fingerRing.splayDegrees, maxRingSplay);
    fingerPinky.calibrateSplayDegrees(minPinkySplay, fingerPinky.splayDegrees, maxPinkySplay);

    minIndexSplay = -360.; maxIndexSplay = 360.;
    minMiddleSplay = -360.; maxMiddleSplay = 360.;
    minRingSplay = -360.; maxRingSplay = 360.;
    minPinkySplay = -360.; maxPinkySplay = 360.;

    calibrating = false;
    runningCalibration = false;
    // Serial.println("calibration ended!");
  }
}

bool thumbCurlUp = true, indexCurlUp = true, middleCurlUp = true, ringCurlUp = true, pinkyCurlUp = true;
bool thumbSplayUp = true, indexSplayUp = true, middleSplayUp = true, ringSplayUp = true, pinkySplayUp = true;

int triggerbtnCounter = 0;
int abtnCounter = 0;
int bbtnCounter = 0;
int thumbstickbtnCounter = 0;
int systembtnCounter = 0;

void test(Quaternion q){
  Serial.print(atan2(2 * (q.x * q.w - q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))*180/3.14);
  Serial.print(" ");
  Serial.print(2 * (q.x * q.w - q.y * q.z));
  Serial.print(" ");
  Serial.println(1 - 2 * (q.x * q.x + q.y * q.y));
}

int millisWhenAllFingersAreTogether = 0;
bool allFingersWereTouchingBefore = false;
bool hasNewData = false;
// the loop function runs over and over again forever
void loop() {
  if(hasSerialData){
    interpret_serial_data();
    hasSerialData = false;
    hasNewData = true;
  } 
  bnoIndex.getReadings();
  bnoMiddle.getReadings();
  bnoRing.getReadings();
  bnoPinky.getReadings();

  if(bnoIndex.hasNewQuaternion){
    // Serial.println("index");
    Quaternion sensorQuaternion;
    float radAccuracy;
    uint8_t accuracy;
    bnoIndex.getQuat(sensorQuaternion.x, sensorQuaternion.y, sensorQuaternion.z, sensorQuaternion.w, radAccuracy, accuracy);  // get the index IMU quaternion

    fingerIndex.computeAxesValues(relativeQuaternion, sensorQuaternion);  // compute curl and splay axis from reference quaternion and finger quaternion

    controller_data.index_curl = fingerIndex.curlAxis;
    controller_data.index_splay = fingerIndex.splayAxis;
  }

  if(bnoMiddle.hasNewQuaternion){
    // Serial.println("middle");
    Quaternion sensorQuaternion;
    float radAccuracy;
    uint8_t accuracy;
    bnoMiddle.getQuat(sensorQuaternion.x, sensorQuaternion.y, sensorQuaternion.z, sensorQuaternion.w, radAccuracy, accuracy);  // get the middle IMU quaternion
    
    fingerMiddle.computeAxesValues(relativeQuaternion, sensorQuaternion);  // compute curl and splay axis from reference quaternion and finger quaternion

    controller_data.middle_curl = fingerMiddle.curlAxis;
    controller_data.middle_splay = fingerMiddle.splayAxis;
  }

  if(bnoRing.hasNewQuaternion){
    // Serial.println("ring");
    Quaternion sensorQuaternion;
    float radAccuracy;
    uint8_t accuracy;
    bnoRing.getQuat(sensorQuaternion.x, sensorQuaternion.y, sensorQuaternion.z, sensorQuaternion.w, radAccuracy, accuracy);                    // get the ring IMU quaternion
    
    fingerRing.computeAxesValues(relativeQuaternion, sensorQuaternion);  // compute curl and splay axis from reference quaternion and finger quaternion

    controller_data.ring_curl = fingerRing.curlAxis;
    controller_data.ring_splay = fingerRing.splayAxis;
  }

  if(bnoPinky.hasNewQuaternion){
    // Serial.println("pinky");
    Quaternion sensorQuaternion;
    float radAccuracy;
    uint8_t accuracy;
    bnoPinky.getQuat(sensorQuaternion.x, sensorQuaternion.y, sensorQuaternion.z, sensorQuaternion.w, radAccuracy, accuracy);                    // get the pinky IMU quaternion
    // Serial.print(radAccuracy);
    // Serial.print(" ");
    // Serial.println(accuracy);

    fingerPinky.computeAxesValues(relativeQuaternion, sensorQuaternion);  // compute curl and splay axis from reference quaternion and finger quaternion

    controller_data.pinky_curl = fingerPinky.curlAxis;
    controller_data.pinky_splay = fingerPinky.splayAxis;
  }

  // printControllerData();
  // printSerialData();

  if(!runningCalibration){
    if(serial_data.rxed_data.index && serial_data.rxed_data.middle && serial_data.rxed_data.ring && serial_data.rxed_data.pinky){
      if(!allFingersWereTouchingBefore){
        millisWhenAllFingersAreTogether = millis();
        allFingersWereTouchingBefore = true;
        preparingCalibration = true;
        // Serial.println("Preparing calibration");
      }
    } else {
      allFingersWereTouchingBefore = false;
      preparingCalibration = false;
    }

    if(preparingCalibration && millis() - millisWhenAllFingersAreTogether > 5000){
      // Serial.println("run calibration");
      runningCalibration = true;
      preparingCalibration = false;
    }
  } else {
    runSplayCalibration();

    controller_data.index_curl = 0;
    controller_data.index_splay = 512;
    controller_data.middle_curl = 0;
    controller_data.middle_splay = 512;
    controller_data.ring_curl = 0;
    controller_data.ring_splay = 512;
    controller_data.pinky_curl = 0;
    controller_data.pinky_splay = 512;
  }

  // Serial.println(debugTransitions);
  // Serial.println(a_btn);
  // Serial.println(serial_data.rxed_data.ring);
  // Serial.println(" ");
  // if(disableInputs) Serial.println("Inputs disabled");


  // if(controller_data.thumb_curl == 700) thumbCurlUp = false;
  // else if(controller_data.thumb_curl == 520) thumbCurlUp = true;
  // if(controller_data.index_curl == 700) indexCurlUp = false;
  // else if(controller_data.index_curl == 520) indexCurlUp = true;
  // if(controller_data.middle_curl == 700) middleCurlUp = false;
  // else if(controller_data.middle_curl == 520) middleCurlUp = true;
  // if(controller_data.ring_curl == 700) ringCurlUp = false;
  // else if(controller_data.ring_curl == 520) ringCurlUp = true;
  // if(controller_data.pinky_curl == 700) pinkyCurlUp = false;
  // else if(controller_data.pinky_curl == 520) pinkyCurlUp = true;

  // if(thumbCurlUp) controller_data.thumb_curl++; 
  // else controller_data.thumb_curl--;
  // if(indexCurlUp) controller_data.index_curl++; 
  // else controller_data.index_curl--;
  // if(middleCurlUp) controller_data.middle_curl++; 
  // else controller_data.middle_curl--;
  // if(ringCurlUp) controller_data.ring_curl++; 
  // else controller_data.ring_curl--;
  // if(pinkyCurlUp) controller_data.pinky_curl++; 
  // else controller_data.pinky_curl--;

  // if(controller_data.thumb_splay == 700) thumbSplayUp = false;
  // else if(controller_data.thumb_splay == 520) thumbSplayUp = true;
  // if(controller_data.index_splay == 700) indexSplayUp = false;
  // else if(controller_data.index_splay == 520) indexSplayUp = true;
  // if(controller_data.middle_splay == 700) middleSplayUp = false;
  // else if(controller_data.middle_splay == 520) middleSplayUp = true;
  // if(controller_data.ring_splay == 700) ringSplayUp = false;
  // else if(controller_data.ring_splay == 520) ringSplayUp = true;
  // if(controller_data.pinky_splay == 700) pinkySplayUp = false;
  // else if(controller_data.pinky_splay == 520) pinkySplayUp = true;

  // if(thumbSplayUp) controller_data.thumb_splay++; 
  // else controller_data.thumb_splay--;
  // if(indexSplayUp) controller_data.index_splay++; 
  // else controller_data.index_splay--;
  // if(middleSplayUp) controller_data.middle_splay++; 
  // else controller_data.middle_splay--;
  // if(ringSplayUp) controller_data.ring_splay++; 
  // else controller_data.ring_splay--;
  // if(pinkySplayUp) controller_data.pinky_splay++; 
  // else controller_data.pinky_splay--;

  // printControllerData();

  // if(controller_data.a) {
  //   abtnCounter++;
  //   Serial.print(abtnCounter);
  //   Serial.print(" ");
  //   Serial.println("a");
  // } if(controller_data.b) {
  //   bbtnCounter++;
  //   Serial.print(bbtnCounter);
  //   Serial.print(" ");
  //   Serial.println("b");
  // } if(controller_data.system) {
  //   systembtnCounter++;
  //   Serial.print(systembtnCounter);
  //   Serial.print(" ");
  //   Serial.println("system");
  // } if(controller_data.triggerbtn) {
  //   triggerbtnCounter++;
  //   Serial.print(triggerbtnCounter);
  //   Serial.print(" ");
  //   Serial.println("trigger");
  // } if(controller_data.thumbstick_click) {
  //   thumbstickbtnCounter++;
  //   Serial.print(thumbstickbtnCounter);
  //   Serial.print(" ");
  //   Serial.println("thumbstick");
  // } 

  // if(controller_data.thumb_curl == 0) controller_data.thumb_curl = 100;
  // if(controller_data.index_curl == 0) controller_data.index_curl = 100;
  // if(controller_data.middle_curl == 0) controller_data.middle_curl = 100;
  // if(controller_data.ring_curl == 0) controller_data.ring_curl = 100;
  // if(controller_data.pinky_curl == 0) controller_data.pinky_curl = 100;

  // I did a lot of tweaking without knowing exactly what was going on, but I compared the original signals from the development board with the ones I generated using the rp2040 PIO, and figured I needed to invert the bit order for the tundra tracker to receive the SPI communication correctly
  // the next few lines take of this inversion and the header construction
  int thumb_axis = controller_data.thumb_curl;
  int index_axis = controller_data.index_curl;
  int middle_axis = controller_data.middle_curl;
  int ring_axis = controller_data.ring_curl;
  int pinky_axis = controller_data.pinky_curl;
  int thumb_splay_axis = controller_data.thumb_splay;
  int index_splay_axis = controller_data.index_splay;
  int middle_splay_axis = controller_data.middle_splay;
  int ring_splay_axis = controller_data.ring_splay;
  int pinky_splay_axis = controller_data.pinky_splay;
  int trigger = controller_data.trigger;
  int joystick_x = controller_data.thumbstick_x;
  int joystick_y = controller_data.thumbstick_y;
  int thumbstick_click = controller_data.thumbstick_click;
  int triggerbtn = controller_data.triggerbtn;
  int a = controller_data.a;
  int b = controller_data.b;
  int system = controller_data.system;

  int thumb_axis_inverted = 0;
  int index_axis_inverted = 0;
  int middle_axis_inverted = 0;
  int ring_axis_inverted = 0;
  int pinky_axis_inverted = 0;
  int thumb_splay_axis_inverted = 0;
  int index_splay_axis_inverted = 0;
  int middle_splay_axis_inverted = 0;
  int ring_splay_axis_inverted = 0;
  int pinky_splay_axis_inverted = 0;
  int trigger_inverted = 0;
  int joystick_x_inverted = 0;
  int joystick_y_inverted = 0;
  bool bit = 0;
  for(int i = 0; i < 10; i++){
    bit = (thumb_axis >> i) & 1;  // Get the ith bit from the original_value
    thumb_axis_inverted |= (bit << (9 - i));  // Set the bit in the reversed_value
    bit = (index_axis >> i) & 1;  // Get the ith bit from the original_value
    index_axis_inverted |= (bit << (9 - i));  // Set the bit in the reversed_value
    bit = (middle_axis >> i) & 1;  // Get the ith bit from the original_value
    middle_axis_inverted |= (bit << (9 - i));  // Set the bit in the reversed_value
    bit = (ring_axis >> i) & 1;  // Get the ith bit from the original_value
    ring_axis_inverted |= (bit << (9 - i));  // Set the bit in the reversed_value
    bit = (pinky_axis >> i) & 1;  // Get the ith bit from the original_value
    pinky_axis_inverted |= (bit << (9 - i));  // Set the bit in the reversed_value
    bit = (thumb_splay_axis >> i) & 1;  // Get the ith bit from the original_value


    thumb_splay_axis_inverted |= (bit << (9 - i));  // Set the bit in the reversed_value
    bit = (index_splay_axis >> i) & 1;  // Get the ith bit from the original_value

    index_splay_axis_inverted |= (bit << (9 - i));  // Set the bit in the reversed_value
    bit = (middle_splay_axis >> i) & 1;  // Get the ith bit from the original_value
    middle_splay_axis_inverted |= (bit << (9 - i));  // Set the bit in the reversed_value
    bit = (ring_splay_axis >> i) & 1;  // Get the ith bit from the original_value
    ring_splay_axis_inverted |= (bit << (9 - i));  // Set the bit in the reversed_value
    bit = (pinky_splay_axis >> i) & 1;  // Get the ith bit from the original_value
    pinky_splay_axis_inverted |= (bit << (9 - i));  // Set the bit in the reversed_value
    bit = (trigger >> i) & 1;  // Get the ith bit from the original_value

    trigger_inverted |= (bit << (9 - i));  // Set the bit in the reversed_value
    bit = (joystick_x >> i) & 1;  // Get the ith bit from the original_value
    joystick_x_inverted |= (bit << (9 - i));  // Set the bit in the reversed_value
    bit = (joystick_y >> i) & 1;  // Get the ith bit from the original_value
    joystick_y_inverted |= (bit << (9 - i));  // Set the bit in the reversed_value
  }

  int spi_protocol_rev = 1;
  static int frame_id = 0;
  int report_mode = 3;         // MI_PROTOCOL_REVISION_GENERIC
  int status = 0;              // 0 normal, 1 bootloader
  // int input_data_length = 17;  // 17 bytes, 136 bits
  int input_data_length = 40;  // 16 bytes, 128 bits
  // int input_data_length = 15;  // 17 bytes, 136 bits
  int backchannel_length = 0;  // only for RX, probably for haptics that I am not using yet (YET!)
  int event_data_length = 0;   // only for RX, probably for haptics that I am not using yet (YET!)
  int reserved = 0;

  int header1 = (spi_protocol_rev << 24) + (frame_id << 16) + (report_mode << 8) + (status << 0);
  int header2 = (input_data_length << 24) + (backchannel_length << 16) + (event_data_length << 8) + (reserved << 0);
  int data1 = (thumb_axis_inverted << 22) + (index_axis_inverted << 12) + (middle_axis_inverted << 2) + (ring_axis_inverted >> 8);
  int data2 = (ring_axis_inverted << 24) + (pinky_axis_inverted << 14) + (thumb_splay_axis_inverted << 4) + (index_splay_axis_inverted >> 6);
  int data3 = (index_splay_axis_inverted << 26) + (middle_splay_axis_inverted << 16) + (ring_splay_axis_inverted << 6) + (pinky_splay_axis_inverted >> 4);
  int data4 = (pinky_splay_axis_inverted << 28) + (trigger_inverted << 18) + (joystick_x_inverted << 8) + (joystick_y_inverted >> 2);
  int data5 = (joystick_y_inverted << 30) + (thumbstick_click << 29) + (triggerbtn << 28) + (a << 27) + (b << 26) + (system << 25);

  byte byte1 = 0;
  byte byte2 = 0;
  byte byte3 = 0;
  byte byte4 = 0;
  byte byte1Inverted = 0;
  byte byte2Inverted = 0;
  byte byte3Inverted = 0;
  byte byte4Inverted = 0;

  // byte1 = ((header1 >> 24) & 0xFF);
  // byte2 = ((header1 >> 16) & 0xFF);
  // byte3 = ((header1 >> 8) & 0xFF);
  // byte4 = ((header1 >> 0) & 0xFF);
  // byte1Inverted = 0;
  // byte2Inverted = 0;
  // byte3Inverted = 0;
  // byte4Inverted = 0;

  // for(int i = 0; i < 8; i++){
  //   bit = (byte1 >> i) & 1;  // Get the ith bit from the original_value
  //   byte1Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
  //   bit = (byte2 >> i) & 1;  // Get the ith bit from the original_value
  //   byte2Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
  //   bit = (byte3 >> i) & 1;  // Get the ith bit from the original_value
  //   byte3Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
  //   bit = (byte4 >> i) & 1;  // Get the ith bit from the original_value
  //   byte4Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
  // }
  // header1 = (byte1Inverted << 24) + (byte2Inverted << 16) + (byte3Inverted << 8) + (byte4Inverted << 0);
  
  // byte1 = ((header2 >> 24) & 0xFF);
  // byte2 = ((header2 >> 16) & 0xFF);
  // byte3 = ((header2 >> 8) & 0xFF);
  // byte4 = ((header2 >> 0) & 0xFF);
  // byte1Inverted = 0;
  // byte2Inverted = 0;
  // byte3Inverted = 0;
  // byte4Inverted = 0;

  // for(int i = 0; i < 8; i++){
  //   bit = (byte1 >> i) & 1;  // Get the ith bit from the original_value
  //   byte1Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
  //   bit = (byte2 >> i) & 1;  // Get the ith bit from the original_value
  //   byte2Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
  //   bit = (byte3 >> i) & 1;  // Get the ith bit from the original_value
  //   byte3Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
  //   bit = (byte4 >> i) & 1;  // Get the ith bit from the original_value
  //   byte4Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
  // }
  // header2 = (byte1Inverted << 24) + (byte2Inverted << 16) + (byte3Inverted << 8) + (byte4Inverted << 0);

  byte1 = ((data1 >> 24) & 0xFF);
  byte2 = ((data1 >> 16) & 0xFF);
  byte3 = ((data1 >> 8) & 0xFF);
  byte4 = ((data1 >> 0) & 0xFF);
  byte1Inverted = 0;
  byte2Inverted = 0;
  byte3Inverted = 0;
  byte4Inverted = 0;

  for(int i = 0; i < 8; i++){
    bit = (byte1 >> i) & 1;  // Get the ith bit from the original_value
    byte1Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
    bit = (byte2 >> i) & 1;  // Get the ith bit from the original_value
    byte2Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
    bit = (byte3 >> i) & 1;  // Get the ith bit from the original_value
    byte3Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
    bit = (byte4 >> i) & 1;  // Get the ith bit from the original_value
    byte4Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
  }
  data1 = (byte1Inverted << 24) + (byte2Inverted << 16) + (byte3Inverted << 8) + (byte4Inverted << 0);
  
  byte1 = ((data2 >> 24) & 0xFF);
  byte2 = ((data2 >> 16) & 0xFF);
  byte3 = ((data2 >> 8) & 0xFF);
  byte4 = ((data2 >> 0) & 0xFF);
  byte1Inverted = 0;
  byte2Inverted = 0;
  byte3Inverted = 0;
  byte4Inverted = 0;
  
  for(int i = 0; i < 8; i++){
    bit = (byte1 >> i) & 1;  // Get the ith bit from the original_value
    byte1Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
    bit = (byte2 >> i) & 1;  // Get the ith bit from the original_value
    byte2Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
    bit = (byte3 >> i) & 1;  // Get the ith bit from the original_value
    byte3Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
    bit = (byte4 >> i) & 1;  // Get the ith bit from the original_value
    byte4Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
  }
  data2 = (byte1Inverted << 24) + (byte2Inverted << 16) + (byte3Inverted << 8) + (byte4Inverted << 0);

  byte1 = ((data3 >> 24) & 0xFF);
  byte2 = ((data3 >> 16) & 0xFF);
  byte3 = ((data3 >> 8) & 0xFF);
  byte4 = ((data3 >> 0) & 0xFF);
  byte1Inverted = 0;
  byte2Inverted = 0;
  byte3Inverted = 0;
  byte4Inverted = 0;
  
  for(int i = 0; i < 8; i++){
    bit = (byte1 >> i) & 1;  // Get the ith bit from the original_value
    byte1Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
    bit = (byte2 >> i) & 1;  // Get the ith bit from the original_value
    byte2Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
    bit = (byte3 >> i) & 1;  // Get the ith bit from the original_value
    byte3Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
    bit = (byte4 >> i) & 1;  // Get the ith bit from the original_value
    byte4Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
  }
  data3 = (byte1Inverted << 24) + (byte2Inverted << 16) + (byte3Inverted << 8) + (byte4Inverted << 0);

  byte1 = ((data4 >> 24) & 0xFF);
  byte2 = ((data4 >> 16) & 0xFF);
  byte3 = ((data4 >> 8) & 0xFF);
  byte4 = ((data4 >> 0) & 0xFF);
  byte1Inverted = 0;
  byte2Inverted = 0;
  byte3Inverted = 0;
  byte4Inverted = 0;
  
  for(int i = 0; i < 8; i++){
    bit = (byte1 >> i) & 1;  // Get the ith bit from the original_value
    byte1Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
    bit = (byte2 >> i) & 1;  // Get the ith bit from the original_value
    byte2Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
    bit = (byte3 >> i) & 1;  // Get the ith bit from the original_value
    byte3Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
    bit = (byte4 >> i) & 1;  // Get the ith bit from the original_value
    byte4Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
  }
  data4 = (byte1Inverted << 24) + (byte2Inverted << 16) + (byte3Inverted << 8) + (byte4Inverted << 0);

  byte1 = ((data5 >> 24) & 0xFF);
  byte2 = ((data5 >> 16) & 0xFF);
  byte3 = ((data5 >> 8) & 0xFF);
  byte4 = ((data5 >> 0) & 0xFF);
  byte1Inverted = 0;
  byte2Inverted = 0;
  byte3Inverted = 0;
  byte4Inverted = 0;
  
  for(int i = 0; i < 8; i++){
    bit = (byte1 >> i) & 1;  // Get the ith bit from the original_value
    byte1Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
    bit = (byte2 >> i) & 1;  // Get the ith bit from the original_value
    byte2Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
    bit = (byte3 >> i) & 1;  // Get the ith bit from the original_value
    byte3Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
    bit = (byte4 >> i) & 1;  // Get the ith bit from the original_value
    byte4Inverted |= (bit << (7 - i));  // Set the bit in the reversed_value
  }
  data5 = (byte1Inverted << 24) + (byte2Inverted << 16) + (byte3Inverted << 8) + (byte4Inverted << 0);

  // Serial.println(frame_id);
  pio_sm_put_blocking(pio, sm, header1);
  pio_sm_put_blocking(pio, sm, header2);
  pio_sm_put_blocking(pio, sm, data1);
  pio_sm_put_blocking(pio, sm, data2);
  pio_sm_put_blocking(pio, sm, data3);
  pio_sm_put_blocking(pio, sm, data4);
  pio_sm_put_blocking(pio, sm, data5);

  frame_id++;
  if(frame_id > 255){
    frame_id = 0;
  }

  // Flag will be true when the library is ready for new data
  // if ( tundra_tracker.data_ready() )
  // {
  //   // Copy our controller struct to the data packet
  //   tundra_tracker.send_data( &controller_data, sizeof(controller_data) );

  //   // House keeping function for the library that should be ran right after data is ready
  //   tundra_tracker.handle_rx_data( );
  // }
}

// Callback for SPI Chipselect, just connect in the tmi irq function
// void csn_irq( uint gpio, uint32_t event_mask )
// {
//   tundra_tracker.csn_irq( gpio, event_mask );
// }