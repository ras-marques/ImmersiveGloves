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

#define CHANNEL_COMMAND 0
#define CHANNEL_EXECUTABLE 1
#define CHANNEL_CONTROL 2
#define CHANNEL_REPORTS 3
#define CHANNEL_WAKE_REPORTS 4
#define CHANNEL_GYRO 5

//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_RAW_ACCELEROMETER 0x14
#define SENSOR_REPORTID_RAW_GYROSCOPE 0x15
#define SENSOR_REPORTID_RAW_MAGNETOMETER 0x16
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E
#define SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 0x29

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define MAX_PACKET_SIZE 128
#define BNO_I2C_BUFFER_LENGTH 32
#define BNO_ADDRESS 0x4A

uint8_t shtpHeader[5];
uint8_t shtpData[MAX_PACKET_SIZE];

uint8_t sequenceNumber[7];
//Given the data packet, send the header then the dataLength = 17
//Returns false if sensor does not ACK
boolean sendPacket(i2c_inst_t* i2cInterface, uint8_t address, uint8_t channelNumber, uint8_t dataLength)
{
  uint8_t packetLength = dataLength + 4; //Add four bytes for the header
  uint8_t packet[packetLength];
  packet[0] = packetLength & 0xFF;
  packet[1] = packetLength >> 8;
  packet[2] = channelNumber;
  packet[3] = sequenceNumber[channelNumber]++;

  // fill the rest of the packet with what is received as data
  for(uint8_t i = 0; i < dataLength; i++){
    packet[i+4] = shtpData[i];
  }

  char debug[MAX_PACKET_SIZE];
  sprintf(debug, "sent: %d", packet[0]);
  for(uint8_t i = 1; i < packetLength; i++){
    sprintf(debug, "%s,%d", debug, packet[i]);
  }
  uint8_t write_check = i2c_write_blocking(i2cInterface, address, packet, packetLength, false);
  Serial.print("write_check: ");
  Serial.println(write_check);
  if(write_check == PICO_ERROR_GENERIC) return false;

  Serial.println(debug);
  return true;
}

// After power on or completed reset, the BNO will send two messages. One
// reply and one unsolicited message.
// 1) Reset message on SHTP channel 1
// 2) Unsolicited initialization message on SHTP channel 2
// See 5.2.1 on BNO08X datasheet.
// Wait For both of these packets specifically up to a max time and exit
// after both packets are read or max waiting time is reached.
void waitForCompletedReset(uint32_t timeout)
{
	uint32_t tInitialResetTimeMS = millis();
	bool tResetCompleteReceived = false;
	bool tUnsolicitedResponseReceived = false;
	shtpHeader[2] = 0; // Make sure we aren't reading old data.Length = 17
	shtpData[0] = 0;

	// Wait requested timeout for the two packets. OR Until we get both reset responses.
	while(
    millis() - tInitialResetTimeMS < timeout
    &&
	  (!tResetCompleteReceived || !tUnsolicitedResponseReceived)) {
    // Serial.println("checking reset responses");
		receivePacket();
		if (shtpHeader[2] == 1 && shtpData[0] == 0x01) {
			tResetCompleteReceived = true;
      Serial.println("tResetCompleteReceived");
		}
		if (shtpHeader[2] == 2 && shtpData[0] == 0xF1) {
			tUnsolicitedResponseReceived = true;
      Serial.println("tUnsolicitedResponseReceived");
		}
	}
}

//Send command to reset IC
//Read all advertisement packets from sensor
void softReset(void)
{
  Serial.println("Soft reset");
	// after power-on sensor is resetting by itself
	// let's handle that POTENTIAL reset with shorter timeout
	// in case sensor was not resetted - like after issuing RESET command
	waitForCompletedReset(1000);
	shtpData[0] = 1; //Reset

	//Attempt to start communication with sensor
	sendPacket(i2c0, BNO_ADDRESS, CHANNEL_EXECUTABLE, 1); //Transmit packet on channel 1, 1 byte
  // delay(500);
  // sendPacket(i2c0, BNO_ADDRESS, CHANNEL_EXECUTABLE, 1); //Transmit packet on channel 1, 1 byte
  // delay(500);

	// now that reset should occur for sure, so let's try with longer timeout
	waitForCompletedReset(5000);
}

// bool getData(int bytesRemaining, int reportType){
//   uint8_t buffer[BNO_I2C_BUFFER_LENGTH];
//   uint8_t shtpDataIndex = 0;
//   Serial.print("bytesRemaining: ");
//   Serial.println(bytesRemaining);

//   while(bytesRemaining > 0){
//     int numberOfBytesToRead = bytesRemaining;
//     if(numberOfBytesToRead > BNO_I2C_BUFFER_LENGTH - 4) numberOfBytesToRead = BNO_I2C_BUFFER_LENGTH - 4; // only ask for 4+28 bytes at a time - 4 bytes are the header portion

//     Serial.print("asking for ");
//     Serial.print(numberOfBytesToRead);
//     Serial.println(" bytes");
//     uint8_t read_check = i2c_read_blocking(i2c0, BNO_ADDRESS, buffer, numberOfBytesToRead + 4, false);
//     if(read_check == PICO_ERROR_GENERIC){
//       Serial.println("ERROR READING DATA");
//       return false;
//     }

//     char debug[50];
//     uint8_t debugIndex = 0;
//     sprintf(debug, "received:");
//     sprintf(debug, "%s %d", debug, buffer[0]);
//     for(uint8_t i = 1; i < numberOfBytesToRead + 4; i++){
//       sprintf(debug, "%s,%d", debug, buffer[i]);
//     }
//     Serial.println(debug);
//     Serial.println("");

//     // if(reportType == CHANNEL_REPORTS){
//       for(uint8_t i = 0; i < numberOfBytesToRead; i++){
//         if(shtpDataIndex < MAX_PACKET_SIZE) shtpData[shtpDataIndex++] = buffer[i+4]; // fill shtpData with the relevant parts of buffer, meaning the header is discarded
//       }
//     // }

//     bytesRemaining -= numberOfBytesToRead;
//     Serial.print("bytesRemaining: ");
//     Serial.println(bytesRemaining);
//   }
//   Serial.println("Got data, returning true");
//   return true; // all good
// }

bool getData(int bytesRemaining, int reportType = 2){
  uint16_t dataSpot = 0; //Start at the beginning of shtpData array
  uint8_t buffer[BNO_I2C_BUFFER_LENGTH];

	//Setup a series of chunked 32 byte reads
	while (bytesRemaining > 0)
	{
		uint16_t numberOfBytesToRead = bytesRemaining;
		if (numberOfBytesToRead > (BNO_I2C_BUFFER_LENGTH - 4))
			numberOfBytesToRead = (BNO_I2C_BUFFER_LENGTH - 4);

    uint8_t read_check = i2c_read_blocking(i2c0, BNO_ADDRESS, buffer, numberOfBytesToRead + 4, false);
    if(read_check == PICO_ERROR_GENERIC){
      Serial.println("ERROR READING DATA");
      return false;
    }

		for (uint8_t x = 0; x < numberOfBytesToRead; x++)
		{
			uint8_t incoming = buffer[4 + x];
			if (dataSpot < MAX_PACKET_SIZE)
			{
				shtpData[dataSpot++] = incoming; //Store data into the shtpData array
			}
			else
			{
				//Do nothing with the data
			}
		}

		bytesRemaining -= numberOfBytesToRead;
	}
	return (true); //Done!
}

// bool receivePacket(){
//   // Serial.println("checking dataAvailable");
//   int8_t read_check = i2c_read_blocking(i2c0, BNO_ADDRESS, shtpHeader, 4, false);
//   delay(20);
//   if(read_check == PICO_ERROR_GENERIC) {
//     Serial.println("ERROR READING HEADER");
//     return false;
//   }

//   // char debug[30];
//   // sprintf(debug, "shtpHeader: %d,%d,%d,%d",shtpHeader[0],shtpHeader[1],shtpHeader[2],shtpHeader[3]);
//   // Serial.println(debug);

//   uint8_t length = ((shtpHeader[1] & 0x7f) << 8) | shtpHeader[0];
//   length -= 4;
//   if(length == 0) return false;
//   Serial.print("Will get ");
//   Serial.print(length);
//   Serial.println(" bytes of data");
//   if(!getData(length, shtpHeader[2])) return false; // an error ocurred during getData
//   return true;
// }

bool receivePacket(){
  int8_t read_check = i2c_read_blocking(i2c0, BNO_ADDRESS, shtpHeader, 4, false);

  //Get the first four bytes, aka the packet header
  uint8_t packetLSB = shtpHeader[0];
  uint8_t packetMSB = shtpHeader[1];
  uint8_t channelNumber = shtpHeader[2];
  uint8_t sequenceNumber = shtpHeader[3]; //Not sure if we need to store this or not

  //Calculate the number of data bytes in this packet
  uint16_t dataLength = (((uint16_t)packetMSB) << 8) | ((uint16_t)packetLSB);
  dataLength &= ~(1 << 15); //Clear the MSbit.
  //This bit indicates if this package is a continuation of the last. Ignore it for now.
  //TODO catch this as an error and exit

  // if (_printDebug == true)
  // {
  // 	_debugPort->print(F("receivePacket (I2C): dataLength is: "));
  // 	_debugPort->println(dataLength);
  // }

  if (dataLength == 0)
  {
    // Serial.println("dataLength == 0");
    //Packet is empty
    return (false); //All done
  }
  dataLength -= 4; //Remove the header bytes from the data count

  // if(channelNumber == 2) Serial.println("CHANNEL 2!");
  getData(dataLength);
  return true;
}

//Sends the packet to enable the rotation vector
void enableRotationVector(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the ar/vr stabilized rotation vector
void enableARVRStabilizedRotationVector(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the rotation vector
void enableGameRotationVector(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_GAME_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the ar/vr stabilized rotation vector
void enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the accelerometer
void enableAccelerometer(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_ACCELEROMETER, timeBetweenReports);
}

//Sends the packet to enable the gravity
void enableGravity(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_GRAVITY, timeBetweenReports);
}
//Sends the packet to enable the accelerometer
void enableLinearAccelerometer(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports);
}

//Sends the packet to enable the gyro
void enableGyro(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_GYROSCOPE, timeBetweenReports);
}

//Sends the packet to enable the magnetometer
void enableMagnetometer(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports);
}

//Sends the packet to enable the high refresh-rate gyro-integrated rotation vector
void enableGyroIntegratedRotationVector(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the tap detector
void enableTapDetector(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_TAP_DETECTOR, timeBetweenReports);
}

//Sends the packet to enable the step counter
void enableStepCounter(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_STEP_COUNTER, timeBetweenReports);
}

//Sends the packet to enable the Stability Classifier
void enableStabilityClassifier(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_STABILITY_CLASSIFIER, timeBetweenReports);
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
void enableRawAccelerometer(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_RAW_ACCELEROMETER, timeBetweenReports);
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
void enableRawGyro(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_RAW_GYROSCOPE, timeBetweenReports);
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
void enableRawMagnetometer(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_RAW_MAGNETOMETER, timeBetweenReports);
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports)
{
	setFeatureCommand(reportID, timeBetweenReports, 0); //No specific config
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
//Also sets the specific config word. Useful for personal activity classifier
void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig)
{
	long microsBetweenReports = (long)timeBetweenReports * 1000L;

	shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;	 //Set feature command. Reference page 55
	shtpData[1] = reportID;							   //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
	shtpData[2] = 0;								   //Feature flags
	shtpData[3] = 0;								   //Change sensitivity (LSB)
	shtpData[4] = 0;								   //Change sensitivity (MSB)
	shtpData[5] = (microsBetweenReports >> 0) & 0xFF;  //Report interval (LSB) in microseconds. 0x7A120 = 500ms
	shtpData[6] = (microsBetweenReports >> 8) & 0xFF;  //Report interval
	shtpData[7] = (microsBetweenReports >> 16) & 0xFF; //Report interval
	shtpData[8] = (microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
	shtpData[9] = 0;								   //Batch Interval (LSB)
	shtpData[10] = 0;								   //Batch Interval
	shtpData[11] = 0;								   //Batch Interval
	shtpData[12] = 0;								   //Batch Interval (MSB)
	shtpData[13] = (specificConfig >> 0) & 0xFF;	   //Sensor-specific config (LSB)
	shtpData[14] = (specificConfig >> 8) & 0xFF;	   //Sensor-specific config
	shtpData[15] = (specificConfig >> 16) & 0xFF;	  //Sensor-specific config
	shtpData[16] = (specificConfig >> 24) & 0xFF;	  //Sensor-specific config (MSB)

	//Transmit packet on channel 2, 17 bytes
	sendPacket(i2c0, BNO_ADDRESS, CHANNEL_CONTROL, 17);
}

uint16_t parseInputReport(void)
{
  boolean _printDebug = false; //Flag to print debugging variables
  //These are the raw sensor values (without Q applied) pulled from the user requested Input Report
	uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
	uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
	uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
	uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;
	uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
	uint16_t rawGameQuatI, rawGameQuatJ, rawGameQuatK, rawGameQuatReal, quatGameAccuracy;
	uint16_t rawMagQuatI, rawMagQuatJ, rawMagQuatK, rawMagQuatReal, rawMagQuatRadianAccuracy, quatMagAccuracy;
	bool hasNewQuaternion, hasNewGameQuaternion, hasNewMagQuaternion, hasNewAccel_;
	uint16_t rawFastGyroX, rawFastGyroY, rawFastGyroZ;
	uint8_t tapDetector;
	bool hasNewTap;
	uint16_t stepCount;
	uint32_t timeStamp;
	uint8_t stabilityClassifier;
	uint8_t activityClassifier;
	uint8_t *_activityConfidences;						  //Array that store the confidences of the 9 possible activities
  uint8_t calibrationStatus;							  //Byte R0 of ME Calibration Response
	uint16_t memsRawAccelX, memsRawAccelY, memsRawAccelZ; //Raw readings from MEMS sensor
	uint16_t memsRawGyroX, memsRawGyroY, memsRawGyroZ;	//Raw readings from MEMS sensor
	uint16_t memsRawMagX, memsRawMagY, memsRawMagZ;		  //Raw readings from MEMS sensor

	//These Q values are defined in the datasheet but can also be obtained by querying the meta data records
	//See the read metadata example for more info
	int16_t rotationVector_Q1 = 14;
	int16_t rotationVectorAccuracy_Q1 = 12; //Heading accuracy estimate in radians. The Q point is 12.
	int16_t accelerometer_Q1 = 8;
	int16_t linear_accelerometer_Q1 = 8;
	int16_t gyro_Q1 = 9;
	int16_t magnetometer_Q1 = 4;
	int16_t angular_velocity_Q1 = 10;

	//Calculate the number of data bytes in this packet
	int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
	dataLength &= ~(1 << 15); //Clear the MSbit. This bit indicates if this package is a continuation of the last.
	//Ignore it for now. TODO catch this as an error and exit

	dataLength -= 4; //Remove the header bytes from the data count

	timeStamp = ((uint32_t)shtpData[4] << (8 * 3)) | ((uint32_t)shtpData[3] << (8 * 2)) | ((uint32_t)shtpData[2] << (8 * 1)) | ((uint32_t)shtpData[1] << (8 * 0));

	// The gyro-integrated input reports are sent via the special gyro channel and do no include the usual ID, sequence, and status fields
	if(shtpHeader[2] == CHANNEL_GYRO) {
		rawQuatI = (uint16_t)shtpData[1] << 8 | shtpData[0];
		rawQuatJ = (uint16_t)shtpData[3] << 8 | shtpData[2];
		rawQuatK = (uint16_t)shtpData[5] << 8 | shtpData[4];
		rawQuatReal = (uint16_t)shtpData[7] << 8 | shtpData[6];
		rawFastGyroX = (uint16_t)shtpData[9] << 8 | shtpData[8];
		rawFastGyroY = (uint16_t)shtpData[11] << 8 | shtpData[10];
		rawFastGyroZ = (uint16_t)shtpData[13] << 8 | shtpData[12];

		return SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR;
	}

	uint8_t status = shtpData[5 + 2] & 0x03; //Get status bits
	uint16_t data1 = (uint16_t)shtpData[5 + 5] << 8 | shtpData[5 + 4];
	uint16_t data2 = (uint16_t)shtpData[5 + 7] << 8 | shtpData[5 + 6];
	uint16_t data3 = (uint16_t)shtpData[5 + 9] << 8 | shtpData[5 + 8];
	uint16_t data4 = 0;
	uint16_t data5 = 0; //We would need to change this to uin32_t to capture time stamp value on Raw Accel/Gyro/Mag reports

	if (dataLength - 5 > 9)
	{
		data4 = (uint16_t)shtpData[5 + 11] << 8 | shtpData[5 + 10];
	}
	if (dataLength - 5 > 11)
	{
		data5 = (uint16_t)shtpData[5 + 13] << 8 | shtpData[5 + 12];
	}

	//Store these generic values to their proper global variable
	if (shtpData[5] == SENSOR_REPORTID_ACCELEROMETER || shtpData[5] == SENSOR_REPORTID_GRAVITY)
	{
		hasNewAccel_ = true;
		accelAccuracy = status;
		rawAccelX = data1;
		rawAccelY = data2;
		rawAccelZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_LINEAR_ACCELERATION)
	{
		accelLinAccuracy = status;
		rawLinAccelX = data1;
		rawLinAccelY = data2;
		rawLinAccelZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_GYROSCOPE)
	{
		gyroAccuracy = status;
		rawGyroX = data1;
		rawGyroY = data2;
		rawGyroZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_MAGNETIC_FIELD)
	{
		magAccuracy = status;
		rawMagX = data1;
		rawMagY = data2;
		rawMagZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_ROTATION_VECTOR ||
		shtpData[5] == SENSOR_REPORTID_GAME_ROTATION_VECTOR ||
		shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR ||
		shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR)
	{
		hasNewQuaternion = true;
		quatAccuracy = status;
		rawQuatI = data1;
		rawQuatJ = data2;
		rawQuatK = data3;
		rawQuatReal = data4;

		//Only available on rotation vector and ar/vr stabilized rotation vector,
		// not game rot vector and not ar/vr stabilized rotation vector
		rawQuatRadianAccuracy = data5;

		if(shtpData[5] == SENSOR_REPORTID_ROTATION_VECTOR || shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR) {
			hasNewMagQuaternion = true;
			quatMagAccuracy = status;
			rawMagQuatI = data1;
			rawMagQuatJ = data2;
			rawMagQuatK = data3;
			rawMagQuatReal = data4;
			rawMagQuatRadianAccuracy = data5;
		}
		if(shtpData[5] == SENSOR_REPORTID_GAME_ROTATION_VECTOR || shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR) {
			hasNewGameQuaternion = true;
			quatGameAccuracy = status;
			rawGameQuatI = data1;
			rawGameQuatJ = data2;
			rawGameQuatK = data3;
			rawGameQuatReal = data4;
      Serial.print(rawGameQuatI);
      Serial.print(",");
      Serial.print(rawGameQuatJ);
      Serial.print(",");
      Serial.print(rawGameQuatK);
      Serial.print(",");
      Serial.println(rawGameQuatReal);
		}
	}
	else if (shtpData[5] == SENSOR_REPORTID_TAP_DETECTOR)
	{
		tapDetector = shtpData[5 + 4]; //Byte 4 only
		hasNewTap = true;
	}
	else if (shtpData[5] == SENSOR_REPORTID_STEP_COUNTER)
	{
		stepCount = data3; //Bytes 8/9
	}
	else if (shtpData[5] == SENSOR_REPORTID_STABILITY_CLASSIFIER)
	{
		stabilityClassifier = shtpData[5 + 4]; //Byte 4 only
	}
	else if (shtpData[5] == SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER)
	{
		activityClassifier = shtpData[5 + 5]; //Most likely state

		//Load activity classification confidences into the array
		for (uint8_t x = 0; x < 9; x++)					   //Hardcoded to max of 9. TODO - bring in array size
			_activityConfidences[x] = shtpData[5 + 6 + x]; //5 bytes of timestamp, byte 6 is first confidence byte
	}
	else if (shtpData[5] == SENSOR_REPORTID_RAW_ACCELEROMETER)
	{
		memsRawAccelX = data1;
		memsRawAccelY = data2;
		memsRawAccelZ = data3;
    // Serial.println("accel");
	}
	else if (shtpData[5] == SENSOR_REPORTID_RAW_GYROSCOPE)
	{
		memsRawGyroX = data1;
		memsRawGyroY = data2;
		memsRawGyroZ = data3;
    // Serial.println("gyro");
	}
	else if (shtpData[5] == SENSOR_REPORTID_RAW_MAGNETOMETER)
	{
		memsRawMagX = data1;
		memsRawMagY = data2;
		memsRawMagZ = data3;
    Serial.println("magn");
	}
	else if (shtpData[5] == SHTP_REPORT_COMMAND_RESPONSE)
	{
		if (_printDebug == true)
		{
			Serial.println("!");
		}
		//The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
		uint8_t command = shtpData[5 + 2]; //This is the Command byte of the response

		if (command == COMMAND_ME_CALIBRATE)
		{
			if (_printDebug == true)
			{
				Serial.println("ME Cal report found!");
			}
			calibrationStatus = shtpData[5 + 5]; //R0 - Status (0 = success, non-zero = fail)
		}
	}
	else
	{
		//This sensor report ID is unhandled.
		//See reference manual to add additional feature reports as needed
		return 0;
	}

	//TODO additional feature reports may be strung together. Parse them all.
	return shtpData[5];
}

uint16_t parseCommandReport(void){
  uint8_t calibrationStatus;							  //Byte R0 of ME Calibration Response
	if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE)
	{
		//The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
		uint8_t command = shtpData[2]; //This is the Command byte of the response

		if (command == COMMAND_ME_CALIBRATE)
		{
			calibrationStatus = shtpData[5 + 0]; //R0 - Status (0 = success, non-zero = fail)
		}
		return shtpData[0];
	}
	else
	{
		//This sensor report ID is unhandled.
		//See reference manual to add additional feature reports as needed
	}

	//TODO additional feature reports may be strung together. Parse them all.
	return 0;
}

uint16_t getReadings(void){
	if (receivePacket() == true)
	{
    // Serial.println("Receive packet true");
    // Serial.print("shtpHeader[2]: ");
    // Serial.println(shtpHeader[2]);
    // Serial.print("shtpData[0]: ");
    // Serial.println(shtpData[0]);
		//Check to see if this packet is a sensor reporting its data to us
		if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
		{
			return parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
		}
		else if (shtpHeader[2] == CHANNEL_CONTROL)
		{
			return parseCommandReport(); //This will update responses to commands, calibrationStatus, etc.
		}
		else if(shtpHeader[2] == CHANNEL_GYRO)
		{
		return parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
		}
	}
	return 0;
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
  delay(1000);                         // Give the IMUs time to boot up

  Serial.println("I2C0 configured");

  //Begin by resetting the IMU
	softReset();

  // shtpData[0] = 2; //On
	// //Attempt to start communication with sensor
  // for(uint8_t i = 0; i < 255; i++){
	//   sendPacket(i2c0, BNO_ADDRESS, CHANNEL_EXECUTABLE, 1); //Transmit packet on channel 1, 1 byte
  //   delay(100);
  // }

  //Check communication with device
	shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	shtpData[1] = 0;							  //Reserved

	//Transmit packet on interface i2c0, address BNO_ADDRESS, channel CHANNEL_CONTROL, 2 bytes
  Serial.println("");
  Serial.println("Getting Product ID");
	sendPacket(i2c0, BNO_ADDRESS, CHANNEL_CONTROL, 2);

  // for(uint8_t i = 0; i < 3; i++){
  //   Serial.print("iteration ");
  //   Serial.println(i);
  //   int8_t read_check = i2c_read_blocking(i2c0, BNO_ADDRESS, shtpHeader, 4, false);
  //   if(read_check == PICO_ERROR_GENERIC) {
  //     Serial.println("ERROR READING HEADER");
  //     delay(500);
  //   } else {
  //     uint16_t bytesAvailable = (shtpHeader[1] << 8)+shtpHeader[0];
  //     int channel = shtpHeader[2];
  //     Serial.print("Channel ");
  //     Serial.print(channel);
  //     Serial.print(" has ");
  //     Serial.print(bytesAvailable);
  //     Serial.println(" bytes available to read");

  //     bool result = getData(bytesAvailable - 4, channel);
  //     if(result) Serial.println("OK I guess..");
  //   }
  // }

  // for(uint8_t i = 0; i < 255; i++){
	//   sendPacket(i2c0, BNO_ADDRESS, CHANNEL_CONTROL, 2);
  //   delay(100);
  // }

	uint32_t tInitialResetTimeMS = millis();
	bool tBoardInfoReceived = false;

  Serial.println("Getting ID...");

	// Wait max 2.5s for the product_id_response and ignore other packets received during that time.
	while (millis() - tInitialResetTimeMS < 10000 && (!tBoardInfoReceived)) {
		receivePacket();
    Serial.println("product id requested: ");
    // Serial.println(shtpHeader[2]);
		if (shtpHeader[2] == 2 && shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
			tBoardInfoReceived = true;
			uint8_t swMajor = shtpData[2];
			uint8_t swMinor = shtpData[3];
			uint32_t swPartNumber = ((uint32_t)shtpData[7] << 24) | ((uint32_t)shtpData[6] << 16) | ((uint32_t)shtpData[5] << 8) | ((uint32_t)shtpData[4]);
			uint32_t swBuildNumber = ((uint32_t)shtpData[11] << 24) | ((uint32_t)shtpData[10] << 16) | ((uint32_t)shtpData[9] << 8) | ((uint32_t)shtpData[8]);
			uint16_t swVersionPatch = ((uint16_t)shtpData[13] << 8) | ((uint16_t)shtpData[12]);
      Serial.print("SW Version Major: 0x");
      Serial.print(swMajor, HEX);
      Serial.print(" SW Version Minor: 0x");
      Serial.print(swMinor, HEX);
      Serial.print(" SW Part Number: 0x");
      Serial.print(swPartNumber, HEX);
      Serial.print(" SW Build Number: 0x");
      Serial.print(swBuildNumber, HEX);
      Serial.print(" SW Version Patch: 0x");
      Serial.print(swVersionPatch, HEX);
		}
	}

  if(!tBoardInfoReceived) Serial.println("BNO not detected!");
  else{
    Serial.println("");
    Serial.println("Enabling ARVR stabilized game rotation vector");
    enableARVRStabilizedGameRotationVector(10);
    
    // Serial.println("");
    // Serial.println("Enabling raw accelerometer");
    // enableRawAccelerometer(10);

    // Serial.println("");
    // Serial.println("Enabling raw gyro");
    // enableRawGyro(10);
    
    // Serial.println("");
    // Serial.println("Enabling raw magnetometer");
    // enableRawMagnetometer(10);
  }


  // pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  getReadings();
  // bool success = receivePacket();
  // if(!success){/*Serial.println("no report available");*/}
  // else Serial.println("report available");
}