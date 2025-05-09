#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM-20948_IMU
#include "string.h"
#include <BluetoothSerial.h>

#define SERIAL_PORT Serial
#define SPI_PORT SPI 

BluetoothSerial BT;
String strr = "";

struct Sensor {
    int pin;
    String name;
    int errCount;
    icm_20948_DMP_data_t data;
};


#define NUM_SENSORS 5 // Change this to the number of sensors, for now dont touch anything else plsssss
Sensor CS_PINS[NUM_SENSORS] = {
    {13, "r_elbow", 0}, 
     {27, "r_shoulder", 0},
    {15, "l_shoulder", 0},
    {4, "l_elbow", 0},
    {2, "chest", 0},
    // {32, "head", 0},

};

ICM_20948_SPI myICM[NUM_SENSORS]; 




void setup() {
    SERIAL_PORT.begin(250000);
    delay(100);
    SPI_PORT.begin();
    SPI_PORT.setFrequency(4000000); 

    BT.begin("MoCap");

    // Set CS pins as outputs and disable all sensors initially
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(CS_PINS[i].pin, OUTPUT);
        digitalWrite(CS_PINS[i].pin, HIGH);
        gpio_pullup_en((gpio_num_t)CS_PINS[i].pin);
        delay(100);
    }

    for (int i = 0; i < NUM_SENSORS; i++) {
        // digitalWrite(CS_PINS[i].pin, LOW);
        delay(10);
        initSensor(i);
        // digitalWrite(CS_PINS[i].pin, HIGH);
    }
}



// main loop, change stuff here only. dont date touch anything else :|
void loop() {
    
    strr = "";
    unsigned long startTime = millis();
    
    for (int i = 0; i < NUM_SENSORS; i++) {
      
      
        bool moreData = true;
        while (moreData) {
            icm_20948_DMP_data_t &data = CS_PINS[i].data;
            
            // Check FIFO count properly
            uint16_t fifoCount;
            myICM[i].getFIFOcount(&fifoCount);
            if (fifoCount > 512) {  // Prevent overflow
                myICM[i].resetFIFO();
                            delay(5);

                break;
            }
            
            myICM[i].readDMPdataFromFIFO(&data);
            delay(1);

            
            if (!myICM[i].isConnected()) {
                SERIAL_PORT.printf("Error in %s..........\n", CS_PINS[i].name);
                CS_PINS[i].errCount++;
            }

            if (CS_PINS[i].errCount > 10) {
                  CS_PINS[i].errCount = 0;
                  // initSensor(i);
            }
            
            
            if (myICM[i].status == ICM_20948_Stat_Ok || myICM[i].status == ICM_20948_Stat_FIFOMoreDataAvail) {
                if ((data.header & DMP_header_bitmap_Quat9) > 0) {
                    // Extract and print data
                    double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0;
                    double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;
                    double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;
                    double q0_squared = 1.0 - (q1*q1 + q2*q2 + q3*q3);
                    double q0 = (q0_squared > 0) ? sqrt(q0_squared) : 0.0;

                    myICM[i].getAGMT();
                    float ax = myICM[i].accX();
                    float ay = myICM[i].accY();
                    float az = myICM[i].accZ();
                    float gx = myICM[i].gyrX();
                    float gy = myICM[i].gyrY();
                    float gz = myICM[i].gyrZ();
                    // Read raw magnetometer data (bypassing DMP)
                    float mx = myICM[i].magX();
                    float my = myICM[i].magY();
                    float mz = myICM[i].magZ();
                    // float mx = (float)data.Compass.Data.X;
                    // float my = (float)data.Compass.Data.Y;
                    // float mz = (float)data.Compass.Data.Z;
                   
                    printData(i, ax, ay, az, gx, gy, gz, mx, my, mz, q0, q1, q2, q3);
                }
                moreData = (myICM[i].status == ICM_20948_Stat_FIFOMoreDataAvail);
                // CS_PINS[i].errCount++;
            } else {
                Serial.println("aaaaaaaaaaaaa");

                moreData = false;
                myICM[i].resetFIFO();
                CS_PINS[i].errCount++;
            }
        }
    }

    unsigned long endTime = millis();
    Serial.print("Time taken (ms): ");
    int timee = endTime - startTime; 
    Serial.println(timee);
    Serial.println("sent\n\n");
    // strr += String(timee) + "\n" + "sent";
    strr += "sent";
    BT.println(strr);
    delay(4); // Reduced delay to improve loop frequency
}


void printData(int sensorID, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, double q0, double q1, double q2, double q3) {
    // Create a String object to hold the formatted data
    String output = String(CS_PINS[sensorID].name) + ":" +
                    String(ax, 3) + "," + String(ay, 3) + "," + String(az, 3) + "," +
                    String(gx, 3) + "," + String(gy, 3) + "," + String(gz, 3) + "," +
                    String(mx, 3) + "," + String(my, 3) + "," + String(mz, 3) + "," +
                    String(q0, 3) + "," + String(q1, 3) + "," + String(q2, 3) + "," + String(q3, 3) + "\n";

    SERIAL_PORT.print(output);

    strr += output;
}



// void printDatamag(int sensorID, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz,double q0, double q1, double q2, double q3) {
//     SERIAL_PORT.printf("Sensor %d: %.3f,%.3f,%.3f\n", 
//         sensorID, mx,my,mz);
// }



void initSensor(int index) {
    SERIAL_PORT.printf("Initializing ICM-20948 on CS pin %s...\n", CS_PINS[index].name);
    bool initialized = false;
    // myICM[index].swReset(); 
    
    bool success = true;
    int tries = 0;
    while (tries < 5) {
        myICM[index].begin(CS_PINS[index].pin, SPI_PORT, 4000000);
        
        SERIAL_PORT.printf("%d Initialization status: ", index);
        SERIAL_PORT.println(myICM[index].statusString());

        if (myICM[index].status != ICM_20948_Stat_Ok) {
            SERIAL_PORT.printf("Trying again... .............. %s\n", CS_PINS[index].name);
                // Set CS pins as outputs and disable all sensors initially
            for (int i = 0; i < NUM_SENSORS; i++) {
                pinMode(CS_PINS[i].pin, OUTPUT);
                digitalWrite(CS_PINS[i].pin, HIGH);
            }
            digitalWrite(CS_PINS[index].pin, LOW);
            tries++;
            delay(500);
        } else {
            initialized = true;
        }
    
        SERIAL_PORT.println(F("Device connected!"));

        if (initialized) {
            success &= (myICM[index].initializeDMP() == ICM_20948_Stat_Ok);

            // success &= (myICM[index].enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD) == ICM_20948_Stat_Ok);
            // success &= (myICM[index].enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

            success &= (myICM[index].enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok); //fpr quat9
            // success &= (myICM[index].enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok); //fpr Quat9
            success &= (myICM[index].setDMPODRrate(DMP_ODR_Reg_Quat9, 3) == ICM_20948_Stat_Ok);

            // success &= (myICM[index].setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok);        // Set to 1Hz
            // success &= (myICM[index].setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to 1Hz
            
            success &= (myICM[index].enableFIFO() == ICM_20948_Stat_Ok);
            success &= (myICM[index].enableDMP() == ICM_20948_Stat_Ok);
            success &= (myICM[index].resetDMP() == ICM_20948_Stat_Ok);
            success &= (myICM[index].resetFIFO() == ICM_20948_Stat_Ok);

            if (!success) {
                SERIAL_PORT.printf("Enable DMP failed! (%s)\n", CS_PINS[index].name);
                tries++;
                delay(500);
            } else {
              break;
            }
          }
    }
        // Manually start magnetometer in continuous mode (100Hz)
    if (myICM[index].startupMagnetometer(false) != ICM_20948_Stat_Ok) { // false = full init (not minimal)
        SERIAL_PORT.println("Magnetometer failed to start!");
        // while(1);
    }
}