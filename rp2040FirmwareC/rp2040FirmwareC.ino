#include "BNO085.h"

//SH-2 Protocol (always starting with this)
// Byte  Field                                 Example
//  0    Length LSB                            21
//  1    Length MSB                             0
//  2    Channel                                2
//  3    SeqNum                                 0

//Then depending on what we want to do, the format of the next bytes may change
//if we want to enable the accel report, 

//  4    Report ID                                           0xFD  Set Feature Command
//  5    Feature Report ID                                   0x01  Accelerometer
//  6    Feature flags                                       0
//  7    Change sensitivity [absolute | relative ] LSB       0
//  8    Change sensitivity [absolute | relative ] MSB       0
//  9    Report Interval LSB                                 10000&255
// 10    Report Interval                                     (10000>>8)&255
// 11    Report Interval                                     (10000>>16)&255
// 12    Report Interval MSB                                 (10000>>24)&255
// 13    Batch Interval LSB                                  0
// 14    Batch Interval                                      0
// 15    Batch Interval                                      0
// 16    Batch Interval MSB                                  0
// 17    Sensor-specific configuration word LSB              0
// 18    Sensor-specific configuration word                  0
// 19    Sensor-specific configuration word                  0
// 20    Sensor-specific configuration word MSB              0

BNO085 bnoRef;
BNO085 bnoThumb;

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
  bnoThumb.begin(i2c1, 0x4B);
  
  // pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  bnoRef.getReadings();
  bnoThumb.getReadings();
  // bool success = receivePacket();
  // if(!success){/*Serial.println("no report available");*/}
  // else Serial.println("report available");
}