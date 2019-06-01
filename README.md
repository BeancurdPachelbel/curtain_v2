# curtain_v2
The curtain motor base on stepper motor, belongs to smart home v2

The stepper motor is controled by the timer's interrupt. 

The range of the motor's speed is from 30RPM to 150RPM, converts into frequency is from 100Hz to 500Hz, which means each pulse that the ESP32 generate will be delay 2ms until next pulse is generated.