# curtain_v2
The curtain motor base on stepper motor, belongs to smart home v2

The stepper motor is controled by the timer's interrupt. 

The range of the motor's speed is from 30RPM to 150RPM, converts into frequency is from 100Hz to 500Hz, which means each pulse that the ESP32 generate will be delay 2ms until next pulse is generated.

## About the Project
### Some of the Project's introduction
### A Gif
### A Video Link

## Getting Started
### Hardware Required Material
### Hardware Schematic
### Software Environment
### Project Installing

## Usage Example

## Achitecture

## Contact

## License

## FAQ
### What's relationship between the rotate speed and the frequency of the stepper motor ?
The formular is as follow: $$\frac{(\frac{360 * Divider}{StepAngle})\times RPM}{60} = Frequency$$
RPM means Revolutions Per Minute.
The unit of Frequency is Hz.


## Goal 2: Redesign the project's achitecture
### List the functions that needs to be implemented.

- Communicate with server by mqtt, thus the cummunitaion's link is recommended as follows: 
	- [ ] SmartConfig(If you want to config wifi by smartconfig)
	- [x] **WIFI**(required)
	- [x] **MQTT**(required)

- Stepper motor rotates in the direction of clockwise and counterclockwise
- Read the main circuit's current from the module INA226 in short delay(the unit is ms)

### According to the functions' list, creat the relatively reasonable module
- wifi_unit
- mqtt_unit
- stepper_unit
- main

### Delete redundant and unuseful code
### Optimize the functions and annotations

