# An Arduino-based mapping vehicle

## The Rover

The folder `rover` contains the code for the vehicle. It uses the [inotool](http://inotool.org) build tool. The rover is based on the [Dagu Rover 5 platform](https://www.pololu.com/product/1551) with 4 motors and encoders, an [Arduino Mega](http://arduino.cc/en/pmwiki.php?n=Main/ArduinoBoardMega), a [Redbear BLE Shield](http://redbearlab.com/bleshield/), 3 [SR04 sonar sensors](http://www.amazon.com/SainSmart-HC-SR04-Ranging-Detector-Distance/dp/B004U8TOE6) and the [Polulo MinIMU compass and gyro](https://www.pololu.com/product/2468). See ... for a full description. 

To build the project you need the [L3G](https://github.com/pololu/l3g-arduino), [LSM303 libraries](https://github.com/pololu/lsm303-arduino) for the MinIMU and the [Bluetooth libraries provided by Redbearlab](http://redbearlab.com/getting-started-bleshield).

## The Mac Application

The Mac app receives sensor data from the rover and sends control commands to the vehicle. It builds a [probabilistic occupancy grid map](http://en.wikipedia.org/wiki/Occupancy_grid_mapping) from the sensor data.
