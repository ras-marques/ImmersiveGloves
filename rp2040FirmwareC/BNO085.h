#ifndef BNO085_h
#define BNO085_h

#include <Arduino.h>

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
    
    void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports);
	  void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig);

    //Global Variables
    i2c_inst_t* deviceInterface; // device interface (i2c0 or i2c1), initialized on begin
    uint8_t deviceAddress; // device address (0x4A or 0x4B), initialized on begin
    uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
    uint8_t shtpData[MAX_PACKET_SIZE];
    uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
};

#endif