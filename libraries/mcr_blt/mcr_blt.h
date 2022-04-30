/**
 * @file mcr_blt.h
 * @author Emile Janho Dit Hreich (emile.janhodithreich@epfl.ch)
 * 
 * @brief  This libraries defines tools for communication between two arduino boards through HC-05 bluetooth module.
 *         Note that the complete setup requires a manual configuration to Master/Slave in Arduino IDE.
 * 
 * @version 0.1
 * @date 2022-04-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef MCR_BLT_H
#define MCR_BLT_H

#include <Arduino.h>

// =====================================================================
// PROTOTYPES

/**
 * @brief 
 * 
 * @param serial_baudrate 
 * @param bluetooth_baudrate
 * @param auto_setup Specifies if the AT-commands to setup the bluetooth modules
 *                   are entered by the user or automatically entered in the script.
 */
void bluetooth_setup(int serial_baudrate, int bluetooth_baudrate, int auto_setup);

/**
 * @brief 
 * 
 */
void bluetooth_send();

/**
 * @brief 
 * 
 */
void bluetooth_recieve();


#endif