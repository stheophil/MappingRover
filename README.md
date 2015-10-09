# An Arduino-based mapping vehicle

## The Rover

The folder `rover` contains the code for the vehicle. It used the [inotool](http://inotool.org) build tool. _inotool_ has been unmaintained for quite some time and it doesn't work with the latest versions of the Arduino IDE. There is a more current form called [Arturo](https://github.com/scottdarch/Arturo) that I haven't had time to check out yet.

In the meantime, [download version 1.0.5 of the Arduino IDE](https://www.arduino.cc/en/Main/OldSoftwareReleases#1.0.x). inotool works fine with it. You may need to delete the RobotControl Arduino library, [see this thread](http://forum.arduino.cc/index.php?topic=168854.0), if you get compilation errors.

The rover is based on the [Dagu Rover 5 platform](https://www.pololu.com/product/1551) with 4 motors and encoders, an [Arduino Mega](http://arduino.cc/en/pmwiki.php?n=Main/ArduinoBoardMega), a [Redbear BLE Shield](http://redbearlab.com/bleshield/), 3 [SR04 sonar sensors](http://www.amazon.com/SainSmart-HC-SR04-Ranging-Detector-Distance/dp/B004U8TOE6) and the [Polulo MinIMU compass and gyro](https://www.pololu.com/product/2468). See [letsmakerobots.com](http://letsmakerobots.com/robot/project/mapping-rover-the-classic-rover-5-with-improved-3d-printed-axis-adaptors) for a full description. 

To build the project you need the [L3G](https://github.com/pololu/l3g-arduino), [LSM303 libraries](https://github.com/pololu/lsm303-arduino) for the MinIMU and the [Bluetooth libraries provided by Redbearlab](http://redbearlab.com/getting-started-bleshield).

## The Controller

The Arduino is only responsible for collecting sensor data and sending it to a computer. It receives control commands from the computer. The main application consists of two parts:

1. All algorithms have been implemented in C++. See `robot_controller.cpp` for a start. This code requires the boost 1.59 and OpenCV 3.0 libraries. It should be easy to compile it e.g. on a Raspberry Pi or any other single-board computer. It contains no platform-specific input/output or UI code. There is a simple pure C interface in `robot_controller_c.h`.

    - `occupancy_grid.cpp` builds a [probabilistic occupancy grid map](http://en.wikipedia.org/wiki/Occupancy_grid_mapping) from the sensor data. 
    The implementation is based on the description in [_Thrun, S., Burgard, W. and Fox, D. (2005). Probabilistic Robotics. Cambridge, Mass: MIT Press_](http://www.probabilistic-robotics.org)
    - `edge_following_strategy.cpp` implements a simple map making strategy: The robot tries to drive past obstacles at a short distance until all obstacles have been visited. 

2. I've used the C interface to create a simple Mac application in Swift that both communicates with the robot via Bluetooth and visualizes the received data in a simple UI.

![Annotated occupancy grid](https://raw.githubusercontent.com/stheophil/MappingRover/master/example_map.png)

