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
#include "eeg.h"
#include "emg.h"
#include "mcr_blt.h"
#include <SoftwareSerial.h>

// ==============================================================================
// MACROS

#define TX 3
#define RX 2
#define SERIAL_BAUDRATE 9600
#define BLUETOOTH_BAUDRATE 9600

SoftwareSerial Bluetooth(TX, RX)
// ==============================================================================

/**
 * @brief 
 * 
 */
void setup()
{

}

/**
 * @brief 
 * 
 */
void loop()
{

}