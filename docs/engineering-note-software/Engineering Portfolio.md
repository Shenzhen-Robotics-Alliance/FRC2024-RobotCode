# FRC 2024 Competition Robot *"PHANTOM"*
Team 5516 *"IRON MAPLE"*, presents.

## Hardware Components


## Self-developed Swerve Drive Library
In recent years of FRC, swerve drivies are dominating the competitions. And teams can run swerve wheels without having to code it. Wpilib have provided an open-to-use programming library that can drive the swerves. However, we didn't use any of the libraries, we have re-written the whole chassis kinematics library down from the ground. We even coded our own linear algebra library and PID library in order to power the swerves. 


We did this not only becuase we think that we should know how the swerve kinematics work, but also for that the Wpi swerve drive library is not good enough, there are a lot of details that need improvements. 
Here are some of the improvements that we have made:
    1. smart wheel direction control
    2. wheel power constraining system
    3. dynamic swerve PID
    4. chassis acceleration constrain
    5. curve-based position estimator
    6. x-formation lock
    7. xml-based chassis configuration
    8. enhanced controller stick dead-banding and exponential controlling


## Vision Sensors and Co-processor
In this year's competition, AprilTags are provided all over the field to aid navigation and the Gamepiece itself is designed to be very recognizable by cameras. It is clear that computer vision is a crucial part of the game. Most teams uses Resbarry Pi loaded with firmwares that WPILIB have already provided. However, we decided to set up our own Computer Vision System and code it down from the ground. The heart of the system is a Jetson-Nano AI computer by NVIDIA, it is capable of running complex AI-vision networks, that are more accurate, reliable and better in performance than the openCV programs running on Resbarry Pi.  We have trained our own object-detection-network that can recoginize not only the game piece but also opponent robots. This will provide us  

## Auto-Pilot System