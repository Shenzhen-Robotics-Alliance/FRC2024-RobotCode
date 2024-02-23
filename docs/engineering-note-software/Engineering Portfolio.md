# FRC 2024 Competition Robot *"PHANTOM"*
Team 5516 *"IRON MAPLE"*, presents.

## Hardware Components


## Self-developed Swerve Drive Library
In recent years of FRC, swerve drivies are so common that the FIRST official have developed a swerve drive code library (named WPI-Swerve-Lib).  This way, teams can power swerves without having to figure out the maths and kinematics behind.  However, this library still has a lot of problems and do not reach our standard. To push the performance, handling and reliability of our chassis to its extreme, we have coded our own swerve drive library, which took us 4 months to complete.

Here are some of the improvements that we have made:
    1. Smart Wheel Direction Control
    2. Wheel Power Constraining
    3. Dynamic Steering PID
    4. Chassis Acceleration Constraining
    5. Curve-Based Position Estimator
    6. X-Formation Lock
    7. xml-based chassis configurator
    8. Enhanced Controller Stick Dead-Banding and Stick Value Curves
    9. Enhanced Path-Planning with Speed Curves 
    10. 
The detailed improvements are too long to be presented in this document, please refer to 

## Vision Co-processor and Cameras
With Navigation Tags provided all over the field and a very recognizable GamePice, it is very obvious that computer vision can bring significant advantages to the performance of robots this year.

Once again, we used a self-developed Computer Vision System over the one Wpi provided. The system is conposed of a "Jetson Nano" AI micro-computer, capable of running complex AI-vision networks. Two cameras are attached to it: the one facing front measures the position of the Navigation Tags, and the one facing back senses the GamePiece.

In order to recognize the GamePieces, we collected datasets on our field to train a custom AI-object-detection network. In comparison to the OpenCV applications running on Resbarry PIs, our Vision Network stands out by its accuracy and reliability. OpenCV applications recognizes the GamePiece mainly by color, and is therefore very dependent to the light intensity of the environment. In comparison, our AI-model recoginizes the GamePiece by its unique shape and appearance, despite lightings and surroundings. 

[Picture: AI-Object-Detection can tell the difference between an orance, an Robot LED status light and the GamePiece, while OpenCV confuses the three for their similarity in color]

The system is connected to the robotRIO via ethernet, a python application we wrote (see here) processes the images and send them through a web server.



## Auto-Pilot System
