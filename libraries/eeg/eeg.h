/**
 * @file eeg.h
 * @author Emile Janho Dit Hreich (emile.janhodithreich@epfl.ch)
 * @brief This library defines functions to read and process EEG signal
 *        using the E3K EEG module.
 * @version 0.1
 * @date 2022-04-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef EEG_H
#define EEG_H

#include <Arduino.h>

/**
 * @brief 
 * 
 * @param pin 
 */
void eeg_setup(int serial pin);

/**
 * @brief 
 * 
 * @param period 
 */
void eeg_record(int period);

/**
 * @brief 
 * 
 */
void eeg_encode();

#endif