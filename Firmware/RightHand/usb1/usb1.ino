#include "tundra_mapped_input.h"
#include "BNO085.h"
#include "Finger.h"
#include "Quaternion.h"
#include <cmath>

// Create TMI object to communicate with Tundra Tracker
TMI tundra_tracker;
void csn_irq( uint gpio, uint32_t event_mask );  // Chip select IRQ must be top level so let's make one and connect it later in setup

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
  uint16_t      thumbstick_x      : 10;  //100
  uint16_t      thumbstick_y      : 10;  //110
  uint8_t       thumbstick_click  : 1;   //120
  uint8_t       a                 : 1;   //121
  uint8_t       b                 : 1;   //122
  uint8_t       system            : 1;   //123
} controller_data_t;
controller_data_t controller_data;

uint8_t received_characters[32] = {};

int chars_rxed = 0;
int uart_successes = 0;
int last_received_char = 0;
long microsLastReceived = 0;
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
  if(chars_rxed == 17){
    interpret_serial_data();
    chars_rxed = 0;
  }
}

int interpretErrorType = 0;
int interpretSum = 0;
void interpret_serial_data(){
  if((received_characters[0] != 0x55) || (received_characters[1] != 0x55)){
    interpretErrorType = 1;
  	return;
  }
  uint8_t sum = 0;

  for(int i = 0; i < 16; i++){
    sum += received_characters[i];
  }

  interpretSum = sum;

  if(sum != received_characters[16]){
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
  controller_data.thumbstick_x = serial_data.rxed_data.thumbstick_x;
  controller_data.thumbstick_y = serial_data.rxed_data.thumbstick_y;
  controller_data.thumbstick_click = serial_data.rxed_data.index;
  controller_data.a = serial_data.rxed_data.middle;
  controller_data.b = serial_data.rxed_data.ring;
  controller_data.system = serial_data.rxed_data.pinky;
  relativeQuaternion.w = serial_data.rxed_data.refQuaternion_w / 32767.;
  relativeQuaternion.x = serial_data.rxed_data.refQuaternion_x / 32767.;
  relativeQuaternion.y = serial_data.rxed_data.refQuaternion_y / 32767.;
  relativeQuaternion.z = serial_data.rxed_data.refQuaternion_z / 32767.;
}

// the setup function runs once when you press reset or power the board
void setup() {

  Serial.begin(115200);
  // while(!Serial) // can't use this here, otherwise it only works with usb connected!

  delay(2000);
  Serial.println("ImmersiveGloves starting...");
  delay(1000); // wait 1 seconds, USB2 waits 3 seconds, so that we are sure USB2 has UART aligned for now - probably will make this better later

  // init the communication to Tundra Tracker, setup the CS irq callback (this has to be at Top level in arduino it seems)
  tundra_tracker.init( );
  gpio_set_irq_enabled_with_callback( tundra_tracker.get_cs_pin(), GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &csn_irq );

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
  Serial.println(controller_data.system);
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
    Serial.println("calibration started!");
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
    Serial.println("calibration ended!");
  }
}

int millisWhenAllFingersAreTogether = 0;
bool allFingersWereTouchingBefore = false;
// the loop function runs over and over again forever
void loop() {
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

    fingerPinky.computeAxesValues(relativeQuaternion, sensorQuaternion);  // compute curl and splay axis from reference quaternion and finger quaternion

    controller_data.pinky_curl = fingerPinky.curlAxis;
    controller_data.pinky_splay = fingerPinky.splayAxis;
  }

  // printControllerData();

  if(!runningCalibration){
    if(serial_data.rxed_data.index && serial_data.rxed_data.middle && serial_data.rxed_data.ring && serial_data.rxed_data.pinky){
      if(!allFingersWereTouchingBefore){
        millisWhenAllFingersAreTogether = millis();
        allFingersWereTouchingBefore = true;
        preparingCalibration = true;
        Serial.println("Preparing calibration");
      }
    } else {
      allFingersWereTouchingBefore = false;
      preparingCalibration = false;
    }

    if(preparingCalibration && millis() - millisWhenAllFingersAreTogether > 5000){
      Serial.println("run calibration");
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

  // Flag will be true when the library is ready for new data
  if ( tundra_tracker.data_ready() )
  {
    // Copy our controller struct to the data packet
    tundra_tracker.send_data( &controller_data, sizeof(controller_data) );

    // House keeping function for the library that should be ran right after data is ready
    tundra_tracker.handle_rx_data( );
  }
}

// Callback for SPI Chipselect, just connect in the tmi irq function
void csn_irq( uint gpio, uint32_t event_mask )
{
  tundra_tracker.csn_irq( gpio, event_mask );
}