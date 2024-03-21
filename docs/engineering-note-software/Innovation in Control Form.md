# Team 6706 "Golem" - Control Award Submission Form
This form provides a concise summary of the control system innovations implemented in the 2024 cooperative robot "PHANTOM" Ⅱ, developed in collaboration by Team 6706 "Golem" and Team 5516 "Iron Maple".

### Sensors Used
The development of our auto-aiming systems was supported by a sophisticated sensor suite, including:

- Rev-2M Distance Sensor: Utilized for the precise detection of game pieces during the intake process.
- On-board Cameras: Two strategically placed cameras facilitate object detection and navigation, aiding both auto-intake and shooting mechanisms.
- Inertial Measurement Units (IMUs): Provide real-time data on the robot's orientation and movement, crucial for dynamic path planning and stabilization.

### Key Algorithms
- Bezier Curve Path Planning: For smooth navigation towards game pieces and optimal positioning for game actions.
- AI-DetectNet for GamePiece Identification: A custom AI model trained to recognize game elements based on their unique shapes, significantly improving detection reliability under varied lighting conditions.
- Hardware-Accelerated AprilTags Detection: Utilizing GPU resources of co-processor for rapid detection of navigational markers, ensuring efficient and responsive robot movement.
- Dynamic Shooter Adjustment Algorithm: Calculates optimal shooter speed and angle based on the robot's distance from the target, stored in a lookup table for quick reference.
- Camera Auto-Calibration Process: Automates the calibration of camera parameters, ensuring high accuracy in object detection and localization.
### Driver Controlled Enhancements 
- Auto-Intake System: Enables the robot to autonomously detect, approach, and intake game pieces, reducing pilot workload and increasing intake speed.
- Shooter Auto-Aiming: Automatically aligns the shooter with the target, adjusting for distance and angle, to ensure high accuracy shots without manual aiming.
- Enhanced Control Feedback: Provides the driver with real-time data on system status and environmental conditions, facilitating informed decision-making and precise control.
### Autonomous Program Diagram
Main AutoStage Diagram
![alt text](<six notes auto path.png>)

### Video Demos
<div style="display: flex;">
    <img src="images/auto intake aim qr.png" alt="Alt Text 2" style="width: 25%; height: 25%">
    <img src="images/shooter aiming demo qr.png" alt="Alt Text 1" style="width: 25%; height: 25%">
    <img src="images/auto calibration video qrcode.png" alt="Alt Text 1" style="width: 25%; height: 25%">
    <img src="images/auto stage demo qr.png" alt="Alt Text 1" style="width: 25%; height: 25%">
</div>
<div style="display: flex;">
    <div style="width: 25%;"><p>Intake Auto-Aim</p></div>
    <div style="width: 25%;"><p>Shooter Auto-Aim</p></div>
    <div style="width: 25%;"><p>Camera Self-Calibrating</p></div>
    <div style="width: 25%;"><p>Auto Stage Testing</p></div>
</div>

For more details, please come visit us at our PIT! 