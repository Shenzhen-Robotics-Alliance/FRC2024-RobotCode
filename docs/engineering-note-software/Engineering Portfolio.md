# FRC 2024 Competition Robot *"PHANTOM"*
***
Team 5516 *"IRON MAPLE"*, presents.
[合影，机器特写]

## Overview
Our robot this year highlights its self-driving capability. A variety of sensors and algorithms, which will be introduced in detail, are implemented so that robot can complete the majority of tasks on field **BY ITSELF**. This engineering portfolio is highly focused on how we achived these auto-piloting functions.  Other aspects are only introduced in brief,  and in-depth discussions are in *the annexes*.

## Hardware Components (in brief)
Our robot is built upon a -inch, square-shaped chassis with four swerve modules on it. It is customly designed to be smaller for easier shipment.
[底盘照片]
A feeder is installed in the back of the chassis, rollers in the feeder "eat" the Note from the ground and feed it to the intake & shoot module.
Above it is the transformable arm, carring the intake & shoot module.  By setting lining the module up with the feeder, the module act as the intake.  Raising the arm a little higher, the fly wheels will line up with the "Speaker" and can accelerate the Note to shoot.  To amplify, we raise the arm vertically such that it fits into the amplfier.  It is a mechanism that is capable of doing three jobs.  
[机械手cad截图]
Another highlight of our robot is its modular design.  Our robot can be seperated to four parts (namely: the chassis module, the feeder module, the intake & shoot module and the arm module) during shipping, and assemled in the hotel room within 3 hours.  This feature has significant enhancement to the portability of our robot, which is a huge part of our concern as we have to bring the robot to the other side of the plannet. Also, this enables us to carry an identical intake & shoot module as back-up, so that the robot can be recovered from a servere damage in a short time.
[拼装前箱子，拼装后机器]

## Self-Developed Swerve Drive Library (in brief)
Swerve drives are so common that the FIRST official have integrated swerve-drive-code into the basic library of FRC programming (Aka. WPILib). Teams can use swerves without having to figure out the maths and kinematics behind.  

However, the library still has a lot of small problems and do not reach our standard. To push the performance, handling and reliability of our chassis to its extreme, we have coded our own swerve drive library down from the ground, which took us 4 months to complete.

In comparison to WPILib, our code is improved in the following ways:
    1. Smart Wheel Direction Control
    2. Wheel Power Constrain
    3. Dynamic Steering PID
    4. Chassis Acceleration Constrain
    5. Curve-Based Position Estimator
    6. XML-based robot configurator
    7. Enhanced Controller Stick Dead-Banding and Stick Value Curves
    8. Enhanced Path-Planning with Speed Curves 
    9. Delay-Compensation for Auto-Stage-Path-Follwing
Technical details for all the above can be found in *the annexes*
Our swerve-drive-lib is nominated to be better not only by our pilots, but also by Team 6706 "Golem", Team 6414 "Voyager" and Team 8812 "SCIE" from our city, who have choosen our swerve lib over WpiLib. 

## On-Bot Computer Vision System
The first step of developing the auto-driving function is, ofcourse, helping the robot sense its surroundings. Once again, we have used a completely self-developed system over the one Wpi provided.

#### Hardware Setups
The heart of our system is a "Jetson Nano" AI-micro-computer. It is similar to the Resbarry Pi used in the WPILib-version of On-Bot Computer Vision, except that it is equiped with high-performance NPU, that can run complex AI-vision networks. Two cameras are attached to it, watching the most important game pieces in the competition: the one facing front measures the position of the Navigation Tags on both the Speaker and the Amplifier, and the one facing back senses the GamePiece.

[Jetson Nano, Two Cameras]

#### GamePieces Detection through AI-DetectNet

In order to recognize the GamePieces, we collected datasets on our field to train a custom AI-object-detection network. In comparison to OpenCV applications that WPILib provided, which recognizes the GamePiece mainly by its color, our Vision Network stands out for its accuracy and reliability. Since the AI-model recoginizes the GamePiece by its unique shape and appearance, it is not dependent to lighting environment and will not confound the target with objects with similar colors. 

![](./images/AI-DetectNet.png)
<div style="display: flex;">
    <img src="image1.png" alt="Alt Text 1" style="width: 50%;">
    <img src="image2.png" alt="Alt Text 2" style="width: 50%;">
</div>

#### Hardware-Accelerated AprilTags Detection

AprilTags (the navigation tags on field) detection is also running with GPU-acceleration, it is based on this https://github.com/NVIDIA-AI-IOT/isaac_ros_apriltag open-source project by NVIDIA.
While the opencv-based AprilTag detection by WPILib runs at 30fps and takes up a lot of CPU resources of the ResbarryPi, we can run it at up to 90fps and with very low CPU usage rate, thanks to the AI-Micro-Processor.

#### Communicating with robotRIO

In order to communicate with the robotRIO, we link the vision co-processor via ethernet. 
A python application we wrote(see here) sends the results continously through a web server.  The java client in our Robot-Code obtains the results from the server 60 times every second.
The server also streams the images to a webpage, so that we can see the results of the detection live on the computer during tests or practises.

#### Calculating Precise Target Positions

#### Camera Auto-Calibration Process



## Auto-Pilot System
Our Auto-Pilot System is capable of completing three tasks, namely: intake, shoot and amplify, in the click of a button.  We will focus on how the system works, what problems we encountered during the development and how we solved them. For a more illustrative demonstration of our achievemnt, it is highly recommended to watch our video demo [here].

#### Automatic Intake
When the robot has no GamePiece inside, it is in intake mode. During this mode, the robot turns 45-degrees left, this rotation is best for the intake as the pilot can see the robot and the GamePiece on field in the same time
As the pilot moves the robot to approach to the GamePiece, the status light turns blue, indicating that the robot "sees" the target.  
Now the pilot presses the **Auto-Pilot Button**, the robot drives to the GamePiece and grabs it automatically.
Let's take a look at the detailed process.  As the pilot presses The **Auto-Pilot Button**, the robot prepares for the intake process automatically: it starts the rollers on the feeder and lowers the arm such that the intake & shoot module lines up with the feeder.  
Meanwhile, the chassis moves towards the target.  Notice the system does not move the chassis in a straight line. Instead, the system dynamically generates the path leading to the target using Bezier-Curves. This not only makes the movement smoother, it also guarrentees that where-ever the robot starts from, it always end up driving backwards towards the Gamepiece. Which is curcial as it will be significantly easier for the feeder to obtain the Note when the the robot is moving to the Note.
Once the Gamepiece gets "eaten" by the intake, a Rev-2M-Distance-Sensor senses the existence on the note and stops the intake automatically before the Note goes too far.  This is important becuase the shooter cannot accelerate when the Note is in contact with it.

#### Shooter Automatic Aiming
After the GamePiece is detected, the robot switches itself to shooting mode. Under this mode, the robot automatically controls its rotation with the help of the camera such that it always faces the speaker.
Now, the **Auto-Pilot Button** has a different function: it controls the shooter.  When the button is hold, the robot accelerates its shooter and adjust the angle of the arm to aim the target.  We test the ideal shooter speed as well as arm angle at different distances and store them in a look-up-table.  Knowing the precise distance to target, the system automatically adjusts the shooter and the arm to aim the target.

When the **Auto-Pilot Button** is released, the kicker is triggered. The Note flies to the speaker, marking the end of the shooting process.  The robot returns to intake mode.

#### Automatic Target Approaching
If this function is enabled, the chassis will drive automatically to a shooting sweet spot once the **Auto-Pilot Button** is pressed under shooting mode.
It uses an algorthm similar to the one used in the auto-intake to generate a path leading to the sweet-spot.  But when it reaches the sweet spot, it does not stop.  The pilot can decided the direction of the chassis movement by the end of the process using the control stick.  The robot moves to the sweet spot, shoots, and continue moving in that direction until the pilot takes the control back.  This way the robot can move in a smooth and continous process, in which it shoots while moving and finish the process elegantly.
The system also decides the timing to shoot by itself, and start the kicker automatically.
Note that this function is optional and it is the pilot's call as to whether to enable it or not.
#### Problem We Encountered: Camera Motion Blur / Losing Focus

#### Problem We Encountered: Low Shooting Success Rate When Chassis Moving



