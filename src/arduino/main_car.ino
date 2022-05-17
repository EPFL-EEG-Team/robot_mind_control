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

}

/**
 * @brief 
 * 
 */
void loop()
{
    if (Bluetooth.available()){ //wait for data received
        Data=Bluetooth.read();
    }
    delay(100);
}



