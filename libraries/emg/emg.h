/**
 * @file emg.h
 * @author Emile Janho Dit Hreich (emile.janhodithreich@epfl.ch)
 * @brief 
 * @version 0.1
 * @date 2022-04-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef EMG_H
#define EMG_H

#include <Arduino.h>

/**
 * @brief 
 * 
 * @param serial_pin 
 */
void emg_setup(int serial_pin);

/**
 * @brief 
 * 
 */
void emg_record();

/**
 * @brief 
 * 
 */
void emg_encode();


#endif