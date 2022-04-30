/**
 * @file mcr_blt.c
 * @author Emle Janho Dit Hreich (emile.janhodithreich@epfl.ch)
 * @brief 
 * 
 * @version 0.1
 * @date 2022-04-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// =====================================================================
// SETUP DESCRIPTION (MASTER/SLAVE)

/**
 * For Option 1, you need to connect the arduinos in a master/slave 
 * relationship.
 * 
 * AT commands: (ATtention -> commands used to control a MoDem)
 * 
 * SLAVE:
 *
 *  Command   | Response   | Description
 *
 *   AT        | OK         | Check if the HC-05 is in AT Command mode
 *   AT+ROLE=0 | OK         | This makes the HC=05 a Slave
 *   AT+ADDR?  | <addr>, OK | Displays the HC-05's address
 *
 * MASTER:
 *
 *  Command          | Response | Description
 *
 *  AT               | OK       | Check if the HC-05 is in AT Command mode
 *  AT+ROLE=1        | OK       | This makes the HC-05 a Master
 *  AT+CMODE=0       | OK       | This allows the HC-05 connect to a specified address
 *  AT+BIND=00<addr> | OK       | When the HC-05 turns on, it will look for this address
 * 
 */

// =====================================================================
// LIBRARIES

#include "mcr_blt.h"
#include <Arduino.h>
#include <SoftwareSerial.h>

// =====================================================================
// FUNCTIONS DEFINITION


void bluetooth_setup(int serial_baudrate, int bluetooth_baudrate, int auto_setup)
{
    if(serial_baudrate <= 0 || bluetooth_baudrate <= 0){
        // TODO add debug macro
        Serial.println("invalid baudrate");
        return;
    }

    if(auto){
    // auto AT command setup
    Serial.begin(serial_baudrate);
    Bluetooth.begin(bluetooth_baudrate);

    if()

    }else{
    // manual user setup

    }

}

void bluetooth_send()
{

}

void bluetooth_recieve()
{

}
