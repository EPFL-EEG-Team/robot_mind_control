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

#include <SoftwareSerial.h>

// ==============================================================================
// MACROS

#define TX 3
#define RX 2
#define SERIAL_BAUDRATE 9600
#define BLUETOOTH_BAUDRATE 9600

#define MOTOR_EN_PIN  11 // enable pin of motor controller must have pin with pulse modulation
#define MOTOR_IN1  13
#define MOTOR_IN2  12

SoftwareSerial Bluetooth(TX, RX);

int Data = 0;
// ==============================================================================

/**
 * @brief 
 * 
 */
void setup()
{

    Bluetooth.begin(9600);
    Serial.begin(9600);
    pinMode(MOTOR_EN_PIN, OUTPUT);
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);

}

/**
 * @brief 
 * 
 */
void loop()
{
    digitalWrite(MOTOR_IN1, LOW); //Direction is forward
    digitalWrite(MOTOR_IN2, HIGH); 
    
    if (Bluetooth.available()){ //wait for data received
        Data=Bluetooth.read();
        Serial.println(Data);
        analogWrite(MOTOR_EN_PIN, Data*1.5); 
    }

  
    
}