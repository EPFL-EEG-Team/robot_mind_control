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


// ==============================================================================
// MACROS and setup

#define SERIAL_BAUDRATE 9600
#define BLUETOOTH_BAUDRATE 9600


BluetoothSerial SerialBT;


//For recording eeg data
//To be used with record_eeg.py
unsigned long now, prev;

float f = 0.0;
char c[50]  = {0}; //size of the number
char type[3]= {0};

int data_points = 0;
//int enable = 1;

// ==============================================================================

/**
 * @brief 
 * 
 */
void setup()
{

    SerialBT.begin("E3K");
    Serial.begin(BLUETOOTH_BAUDRATE);
    prev = micros();
//    enable = 1;
}

/**
 * @brief 
 * 
 */
void loop()
{

//  if (data_points == 0){
//    SerialBT.print("BEGIN_EEG\n");
//  }
//    

  now = micros();
  if(now - prev >= 2000){
    Serial.println("enabled");
//    data_points += 1;

//    if (data_points == 100){
//      data_points = 0;
////      enable = 0;
//      SerialBT.print("END_EEG\n");
//      delay(100);
////      
//    }else{
    f = analogRead(0);
    sprintf(c, "%g", f);
    strcat(c, "_EEG\n");
    SerialBT.print(c);
          
//    }
    
    
    prev = now;

  }
}