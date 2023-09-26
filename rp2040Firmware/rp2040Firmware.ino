#include <Arduino.h>
// This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give
// quartenion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.
// Note sensorValue.status gives calibration accuracy (which improves over time)
#include <Adafruit_BNO08x.h>
// #include <Adafruit_BNO08xSecond.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9


// #define FAST_MODE

// For SPI mode, we also need a RESET
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x  bno08xHandRef(BNO08X_RESET);
// Adafruit_BNO08xSecond  bno08xIndex(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08xHandRef.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
  // if (! bno08xIndex.enableReport(reportType, report_interval)) {
  //   Serial.println("Could not enable stabilized remote vector");
  // }
}

void setup(void) {

  Serial.begin(9600);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.print("test2");

  // byte error;
  if(!Wire.setSDA(4)){
    Serial.print("Could not set SDA");
    // while(true){}
  }
  if(!Wire.setSCL(5)){
    Serial.print("Could not set SCL");
    // while(true){}
  }

  Serial.print("all set ok");

  // Wire.begin();
  // Wire1.beginTransmission(0x4B);
  // error = Wire1.endTransmission();
  // Serial.print(error);
  // Wire1.beginTransmission(0x4B);
  // error = Wire1.endTransmission();
  // Serial.print(error);
  // Wire1.beginTransmission(0x4B);
  // error = Wire1.endTransmission();
  // Serial.print(error);

  // while(1){delay(10);}
  // Serial.println("Adafruit BNO08x test!");

  // bno08xHandRef.begin_I2C(0x4A, &Wire1);

  // Try to initialize!
  while(!bno08xHandRef.begin_I2C(0x4B, &Wire)){
    Serial.println("Failed to find BNO08xHandRef chip, retrying");
  }
  Serial.println("BNO08xHandRef Found!");

  // while(!bno08xIndex.begin_I2C(0x4B, &Wire1)){
  //   Serial.println("Failed to find BNO08xIndex chip, retrying");
  // }
  // Serial.println("BNO08xIndex Found!");

  // Try to initialize!
  // while(!bno08xIndex.begin_I2C()){
  //   Wire1.beginTransmission(0x4A, &Wire1);
  //   error = Wire1.endTransmission();
  //   Serial.print(error);
  //   Serial.println("Failed to find BNO08xIndex chip, retrying");
  // }
  // Serial.println("BNO08xIndex Found!");


  setReports(reportType, reportIntervalUs);

  // Serial.println("Reading events");
  delay(100);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void loop() {

  if (bno08xHandRef.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
  if (bno08xHandRef.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    static long last = 0;
    long now = micros();
    // Serial.print(now - last);             Serial.print("\t");
    last = now;
    // Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    Serial.print("refyaw: "); Serial.println(ypr.yaw);
    Serial.print("refpitch: "); Serial.println(ypr.pitch);
    Serial.print("refroll: "); Serial.println(ypr.roll);
    Serial.println("");
  }

  // if (bno08xIndex.wasReset()) {
  //   Serial.print("sensor was reset ");
  //   setReports(reportType, reportIntervalUs);
  // }
  
  // if (bno08xIndex.getSensorEvent(&sensorValue)) {
  //   // in this demo only one report type will be received depending on FAST_MODE define (above)
  //   switch (sensorValue.sensorId) {
  //     case SH2_ARVR_STABILIZED_RV:
  //       quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
  //     case SH2_GYRO_INTEGRATED_RV:
  //       // faster (more noise?)
  //       quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
  //       break;
  //   }
  //   static long last = 0;
  //   long now = micros();
  //   // Serial.print(now - last);             Serial.print("\t");
  //   last = now;
  //   // Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
  //   Serial.print("indexyaw: "); Serial.println(ypr.yaw);
  //   Serial.print("indexpitch: "); Serial.println(ypr.pitch);
  //   Serial.print("indexroll: "); Serial.println(ypr.roll);
  //   Serial.println("");
  // }
}
