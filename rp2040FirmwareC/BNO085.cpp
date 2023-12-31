#include "BNO085.h"

BNO085::BNO085(){}

//Given the data packet, send the header then the dataLength = 17
//Returns false if sensor does not ACK
boolean BNO085::sendPacket(uint8_t channelNumber, uint8_t dataLength)
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
  uint8_t write_check = i2c_write_blocking(deviceInterface, deviceAddress, packet, packetLength, false);
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
void BNO085::waitForCompletedReset(uint32_t timeout)
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
void BNO085::softReset(void)
{
  Serial.println("Soft reset");
	// after power-on sensor is resetting by itself
	// let's handle that POTENTIAL reset with shorter timeout
	// in case sensor was not resetted - like after issuing RESET command
	waitForCompletedReset(1000);
	shtpData[0] = 1; //Reset

	//Attempt to start communication with sensor
	sendPacket(CHANNEL_EXECUTABLE, 1); //Transmit packet on channel 1, 1 byte

	// now that reset should occur for sure, so let's try with longer timeout
	waitForCompletedReset(5000);
}

//Sends the packet to enable the rotation vector
void BNO085::enableRotationVector(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the ar/vr stabilized rotation vector
void BNO085::enableARVRStabilizedRotationVector(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the rotation vector
void BNO085::enableGameRotationVector(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_GAME_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the ar/vr stabilized rotation vector
void BNO085::enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the accelerometer
void BNO085::enableAccelerometer(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_ACCELEROMETER, timeBetweenReports);
}

//Sends the packet to enable the gravity
void BNO085::enableGravity(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_GRAVITY, timeBetweenReports);
}
//Sends the packet to enable the accelerometer
void BNO085::enableLinearAccelerometer(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports);
}

//Sends the packet to enable the gyro
void BNO085::enableGyro(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_GYROSCOPE, timeBetweenReports);
}

//Sends the packet to enable the magnetometer
void BNO085::enableMagnetometer(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports);
}

//Sends the packet to enable the high refresh-rate gyro-integrated rotation vector
void BNO085::enableGyroIntegratedRotationVector(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the tap detector
void BNO085::enableTapDetector(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_TAP_DETECTOR, timeBetweenReports);
}

//Sends the packet to enable the step counter
void BNO085::enableStepCounter(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_STEP_COUNTER, timeBetweenReports);
}

//Sends the packet to enable the Stability Classifier
void BNO085::enableStabilityClassifier(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_STABILITY_CLASSIFIER, timeBetweenReports);
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
void BNO085::enableRawAccelerometer(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_RAW_ACCELEROMETER, timeBetweenReports);
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
void BNO085::enableRawGyro(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_RAW_GYROSCOPE, timeBetweenReports);
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
void BNO085::enableRawMagnetometer(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_RAW_MAGNETOMETER, timeBetweenReports);
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
void BNO085::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports)
{
	setFeatureCommand(reportID, timeBetweenReports, 0); //No specific config
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
//Also sets the specific config word. Useful for personal activity classifier
void BNO085::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig)
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
	sendPacket(CHANNEL_CONTROL, 17);
}

bool BNO085::getData(uint16_t bytesRemaining){
  uint16_t dataSpot = 0; //Start at the beginning of shtpData array
  uint8_t buffer[BNO_I2C_BUFFER_LENGTH];

	//Setup a series of chunked 32 byte reads
	while (bytesRemaining > 0)
	{
		uint16_t numberOfBytesToRead = bytesRemaining;
		if (numberOfBytesToRead > (BNO_I2C_BUFFER_LENGTH - 4))
			numberOfBytesToRead = (BNO_I2C_BUFFER_LENGTH - 4);

    uint8_t read_check = i2c_read_blocking(deviceInterface, deviceAddress, buffer, numberOfBytesToRead + 4, false);
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

bool BNO085::receivePacket(){
  int8_t read_check = i2c_read_blocking(deviceInterface, deviceAddress, shtpHeader, 4, false);

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

bool BNO085::begin(i2c_inst_t* i2cInterface, uint8_t address){
  deviceInterface = i2cInterface;
  deviceAddress = address;
  //Begin by resetting the IMU
	softReset();

  //Check communication with device
	shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	shtpData[1] = 0;							  //Reserved

	//Transmit packet on interface i2c0, address BNO_ADDRESS, channel CHANNEL_CONTROL, 2 bytes
  Serial.println("");
  Serial.println("Getting Product ID");
	sendPacket(CHANNEL_CONTROL, 2);

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
}

uint16_t BNO085::parseInputReport(void)
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

uint16_t BNO085::parseCommandReport(void){
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

uint16_t BNO085::getReadings(void){
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