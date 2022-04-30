# robot_mind_control

## Description


## Setup

###Bluetooth modules

#### Wiring

#### Software

You will need x2 HC-05 bluetooth modules for this project. One will be connected to the board on the headgear and the other on the car. These modules have two modes: the AT-command mode and the normal communication mode. We will use the AT-command mode to setup both modules in a SLAVE/MASTER relationship. 

The bluetooth module connected to the car will play the role of the slave and the one that is connected to the headgear will have the role of the "Master".

This can be done by inputing specific AT-commands to a Serial Monitor in Arduino IDE or directly through the skecthes.

You will have to specify which option you would like to use upon calling the setup function for the bluetooth module. The commands are detailed below.

## Documentation


## Robot Control

