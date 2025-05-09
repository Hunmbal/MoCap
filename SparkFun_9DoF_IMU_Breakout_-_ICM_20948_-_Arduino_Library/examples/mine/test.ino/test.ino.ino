#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM-20948_IMU

#define SERIAL_PORT Serial
#define SPI_PORT SPI 
#define CS_PIN 5     

ICM_20948_SPI myICM; // Create an ICM_20948_SPI object

void setup() {
      SERIAL_PORT.begin(250000); // Start the serial console
      delay(100);
      SPI_PORT.begin();
      SERIAL_PORT.println(F("Initializing ICM-20948..."));

      bool initialized = false;
      while (!initialized) {
        myICM.begin(CS_PIN, SPI_PORT);

        SERIAL_PORT.print(F("Initialization status: "));
        SERIAL_PORT.println(myICM.statusString());

        if (myICM.status != ICM_20948_Stat_Ok) {SERIAL_PORT.println(F("Trying again..."));delay(500);} 
        else {initialized = true;}
      }

      SERIAL_PORT.println(F("Device connected!"));
      // initSetup();

      bool success = true;
      success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok); // Initialize the DMP
      success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR) == ICM_20948_Stat_Ok);// Enable the DMP orientation sensor
      success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set DMP ODR rates // Set to the maximum rate
      // Enable FIFO and DMP
      success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
      success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
      // Reset DMP and FIFO
      success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
      success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);


      if (!success) {
          SERIAL_PORT.println(F("Enable DMP failed!"));
          SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
          while (1); // Do nothing more
      }
}

void loop() {
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    if ((data.header & DMP_header_bitmap_Quat9) > 0) {
      // Scale quaternion values to +/- 1
      double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Divide by 2^30
      double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;
      double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
      // printData()
          myICM.getAGMT();
      float ax = myICM.accX();
      float ay = myICM.accY();
      float az = myICM.accZ();
      float gx = myICM.gyrX();
      float gy = myICM.gyrY();
      float gz = myICM.gyrZ();
      printData2(ax,ay,az, gx,gy,gz, q0,q1,q2,q3);

    }
  }


  // delay(200);
  // if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) {
  //   delay(10); // Add a small delay if no more data is available
  // }
}



void printData(float ax, float ay, float az,
                float gx, float gy, float gz,
                  double q0, double q1, double q2, double q3) { 
  SERIAL_PORT.printf("data: %.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
   ax, ay, az, 
   gx, gy, gz,  
   q0, q1, q2, q3); 
}

void printData2(float ax, float ay, float az,
                float gx, float gy, float gz,
                  double q0, double q1, double q2, double q3) { 
  SERIAL_PORT.printf("data: %.3f,\t%.3f,\t%.3f,\t%.3f,\t%.3f,\t%.3f,\t%.3f,\t%.3f,\t%.3f,\t%.3f\n",
   ax, ay, az, 
   gx, gy, gz,  
   q0, q1, q2, q3); 
}



