/**
 * @file main_headgear.ino
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

#define EEG_PIN 115200

#define BAUDRATE 9600


// ==============================================================================


/**
 * @brief 
 * 
 */
void setup()
{
    // setup the EMG recording
    emg_setup(BAUDRATE); 

    // setup the EEG recording
    eeg_setup(EEG_PIN);

    // setup the bluetooth communication


}

/**
 * @brief 
 * 
 */
void loop()
{

}
