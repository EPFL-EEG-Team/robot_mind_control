/**
 * @file main.ino
 * @author Emile Janho Dit Hreich (emile.janhodithreich@epfl.ch)
 * @brief The main arduino script to control the car through eeg, emg an
 * @version 0.1
 * @date 2022-04-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// ==============================================================================
// LIBRARIES

#include <BluetoothSerial.h>
#include <stdio.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

// ==============================================================================
// MACROS and setup

#define SERIAL_BAUDRATE 9600
#define BLUETOOTH_BAUDRATE 9600

#define EEG_PIN 0
#define EMG_PIN 0
#define IMU_PIN 0
 
// -----------------------------------------------
// Bluetooth
BluetoothSerial SerialBT;

// -----------------------------------------------
// EEG

unsigned long now, prev;

float f = 0.0;
char c[50]  = {0}; //size of the number
char type[3]= {0};

int data_points = 0;

// -----------------------------------------------
// EMG

float value = 0f;
char emg[50]  = {0}; //size of the number

// -----------------------------------------------
// IMU
BNO080 IMU;
boolean isIMU = false;

float roll  = 0f;
float pitch = 0f;
float yaw   = 0f;

char imu[50]  = {0}; //size of the number


// ==============================================================================

/**
 * @brief 
 * 
 */
void setup()
{

    // Bluetooth
    SerialBT.begin("E3K");
    Serial.begin(BLUETOOTH_BAUDRATE);

    // time
    prev = micros();

    // IMU
    Wire.begin();
    isIMU = IMU.begin();
    if(isIMU == false){
      Serial.println("BNO080 not detected at default I2C address. IMU not connected or check your jumpers and the hookup guide.");
      Serial.println("Continuing....");
    }else{
      Wire.setClock(400000); //Increase I2C data rate to 400kHz
      myIMU.enableRotationVector(50); //Send data update every 50ms
      Serial.println(F("IMU enabled"));
      Serial.println(F("Rotation vector enabled"));
      Serial.println(F("Output in form i, j, k, real, accuracy"));
  }

}

/**
 * @brief 
 * 
 */
void loop()
{
  // -----------------------------------------
  // EEG
  now = micros();
  if(now - prev >= 2000){
    Serial.println("enabled");

    f = analogRead(EEG_PIN);
    sprintf(c, "%g", f);
    strcat(c, "_EEG\n");
    SerialBT.print(c);
          
    prev = now;

  }

  // ------------------------------------------
  // IMU
  if(isIMU == true){
    //Look for reports from the IMU
    if(IMU.dataAvailable()){
        roll  = (IMU.getRoll()) * 180.0 / PI;   // Convert roll to degrees
        pitch = (IMU.getPitch()) * 180.0 / PI;  // Convert pitch to degrees
        yaw   = (IMU.getYaw()) * 180.0 / PI;    // Convert yaw / heading to degrees

        // DEBUG
        Serial.print(roll, 1);
        Serial.print(F(","));
        Serial.print(pitch, 1);
        Serial.print(F(","));
        Serial.print(yaw, 1);
    
        Serial.println();
        // END DEBUG

        // !! Only considers the pitch but can be adapted to take everything into account
        // it also depends on the placement of the board
        sprintf(imu, "%g", pitch);
        strcat(imu, "_IMU\n");
        SerialBT.print(imu);

    }
  }

  // ------------------------------------------
  // EMG  
  value = analogRead(EMG_PIN);
  sprintf(emg, "%g", value);
  strcat(emg, "_EMG\n");
  SerialBT.print(emg);

}