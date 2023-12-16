/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/

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

#define BNO_I2C_BUFFER_LENGTH 32

bool checkReports(){
  static uint8_t buffer[30];
  int read_check = i2c_read_blocking(i2c0, 0x4A, buffer, 4, false);
  if(read_check == PICO_ERROR_GENERIC) return false;
  int length = ((buffer[1] & 0x7f) << 8) | buffer[0]; // 
}

// the setup function runs once when you press reset or power the board
void setup() {

  Serial.begin(115200);
  while(!Serial)

  delay(100);
  Serial.println("ImmersiveGloves starting...");

  // Initialize I2C
  _i2c_init(i2c0, 400000);             // Init I2C0 peripheral at 400kHz
  gpio_set_function(4, GPIO_FUNC_I2C); // set pin 4 as an I2C pin (SDA in this case)
  gpio_set_function(5, GPIO_FUNC_I2C); // set pin 4 as an I2C pin (SCL in this case)
  gpio_pull_up(4);                     // use internal pull up on pin 4
  gpio_pull_up(5);                     // use internal pull up on pin 5
  delay(1000);                         // Give the IMUs time to boot up

  // enable the accel report and terminate the connection (that's what that last "false" argument is for)
  int accelReportPeriodInMicroseconds = 100000;
  static const uint8_t cmd_acc[]  = {
    21, 0, 2, 0, 0xFD, 0x01,  0, 0, 0,
    (accelReportPeriodInMicroseconds>>0)&255,
    (accelReportPeriodInMicroseconds>>8)&255,
    (accelReportPeriodInMicroseconds>>16)&255,
    (accelReportPeriodInMicroseconds>>24)&255,
    0, 0, 0, 0, 0, 0, 0, 0
  };
  i2c_write_blocking(i2c0, 0x4A, cmd_acc, sizeof(cmd_acc), false);

  // int write_check;
  // int read_check;
  // write_check = i2c_write_blocking(i2c0, 0x4A, cmd_acc, sizeof(cmd_acc), false);
  // if(write_check == PICO_ERROR_GENERIC){
  //   Serial.println("ERROR WRITING");
  // }
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  checkReports();

  char debug[30];
  // sprintf(debug, "%d,%d,%d,%d,%d",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);
  // Serial.println(debug);
  int length = ((buffer[1] & 0x7f) << 8) | buffer[0];
  // Serial.println(length);
  while(length)
  if(length > BNO_I2C_BUFFER_LENGTH - 4) length = BNO_I2C_BUFFER_LENGTH - 4;


  // byte error, address;
  // int nDevices;

  // Serial.println("Scanning...");

  // nDevices = 0;
  // for(address = 1; address < 127; address++ ) 
  // {
  //   // The i2c_scanner uses the return value of
  //   // the Write.endTransmisstion to see if
  //   // a device did acknowledge to the address.
  //   Wire.beginTransmission(address);
  //   error = Wire.endTransmission();

  //   if (error == 0)
  //   {
  //     Serial.print("I2C device found at address 0x");
  //     if (address<16) 
  //       Serial.print("0");
  //     Serial.print(address,HEX);
  //     Serial.println("  !");

  //     nDevices++;
  //   }
  //   else if (error==4) 
  //   {
  //     Serial.print("Unknown error at address 0x");
  //     if (address<16) 
  //       Serial.print("0");
  //     Serial.println(address,HEX);
  //   }    
  // }
  // if (nDevices == 0)
  //   Serial.println("No I2C devices found\n");
  // else
  //   Serial.println("done\n");

  // delay(5000);           // wait 5 seconds for next scan

  // // Serial.print("test");
  // // digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  // // delay(1000);                      // wait for a second
  // // digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  // // delay(1000);                      // wait for a second
}

// void Scani2c(int wireSelection){
//   byte errorCode;
//   byte deviceAddress;
//   int totalDevices = 0;
//   // Serial.println("Scanning...");
//   for(deviceAddress = 8; deviceAddress < 120; deviceAddress++ ){
//     Wire.beginTransmission(deviceAddress);
//     errorCode = Wire.endTransmission();

//     if(errorCode == 0){
//       Serial.println(deviceAddress, HEX);
//     }
//   }
// }