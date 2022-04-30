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


/**
 * @brief Starts serial communication
 * 
 */
void bluetooth_setup();

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