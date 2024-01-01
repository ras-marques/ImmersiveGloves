#ifndef BNO085_h
#define BNO085_h

#include <Arduino.h>

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

class BNO085 {
  public:
    BNO085();
    bool begin(i2c_inst_t* i2cInterface, uint8_t address);

    void softReset();	  //Try to reset the IMU via software
	  void waitForCompletedReset(uint32_t timeout);

    bool receivePacket(void);
    bool getData(uint16_t bytesRemaining); //Given a number of bytes, send the requests in I2C_BUFFER_LENGTH chunks
    bool sendPacket(uint8_t channelNumber, uint8_t dataLength);

    void enableRotationVector(uint16_t timeBetweenReports);
    void enableGameRotationVector(uint16_t timeBetweenReports);
    void enableARVRStabilizedRotationVector(uint16_t timeBetweenReports);
    void enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports);
    void enableAccelerometer(uint16_t timeBetweenReports);
    void enableGravity(uint16_t timeBetweenReports);
    void enableLinearAccelerometer(uint16_t timeBetweenReports);
    void enableGyro(uint16_t timeBetweenReports);
    void enableMagnetometer(uint16_t timeBetweenReports);
    void enableTapDetector(uint16_t timeBetweenReports);
    void enableStepCounter(uint16_t timeBetweenReports);
    void enableStabilityClassifier(uint16_t timeBetweenReports);
    void enableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToEnable, uint8_t (&activityConfidences)[9]);
    void enableRawAccelerometer(uint16_t timeBetweenReports);
    void enableRawGyro(uint16_t timeBetweenReports);
    void enableRawMagnetometer(uint16_t timeBetweenReports);
    void enableGyroIntegratedRotationVector(uint16_t timeBetweenReports);

    uint16_t getReadings(void);
	  uint16_t parseInputReport(void);   //Parse sensor readings out of report
    uint16_t parseCommandReport(void); //Parse command responses out of report

    float qToFloat(int16_t fixedPointValue, uint8_t qPoint); //Given a Q value, converts fixed point floating to regular floating point number
    void getGameQuat(float &i, float &j, float &k, float &real, uint8_t &accuracy);

    void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports);
	  void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig);

    //Global Variables
    i2c_inst_t* deviceInterface; // device interface (i2c0 or i2c1), initialized on begin
    uint8_t deviceAddress; // device address (0x4A or 0x4B), initialized on begin
    uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
    uint8_t shtpData[MAX_PACKET_SIZE];
    uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum

    bool hasNewQuaternion, hasNewGameQuaternion, hasNewMagQuaternion, hasNewAccel_;
  
  private:
    
    boolean _printDebug = false; //Flag to print debugging variables

    //These are the raw sensor values (without Q applied) pulled from the user requested Input Report
    uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
    uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
    uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
    uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;
    uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
    uint16_t rawGameQuatI, rawGameQuatJ, rawGameQuatK, rawGameQuatReal, quatGameAccuracy;
    uint16_t rawMagQuatI, rawMagQuatJ, rawMagQuatK, rawMagQuatReal, rawMagQuatRadianAccuracy, quatMagAccuracy;
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
};

#endif