package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.RobotSystem.RobotShell;

/**
 * The entry of the robot's program
 * 
 * @author Sam
 * @version 0.1
 */
public class Main {
    public static void main(String[] args) {
        System.out.printf("@@@@@@  void main(String[] args)  %n");
        RobotBase.startRobot(RobotShell::new);
    }
}


//@Deprecated
//class TimedRobotShell extends TimedRobot {
//    RobotCore robotCore = null;
//    boolean robotBeenEnabledSinceProgram = false;
//    @Override
//    public void robotInit() {
//        System.out.println("<--main program starts-->");
//    }
//    @Override
//    public void robotPeriodic() {
//        if (robotBeenEnabledSinceProgram && !isTest() && robotCore != null)
//            robotCore.updateRobot(isEnabled());
//    }
//
//    private JetsonDetectionAppClient aprilTagDetectionAppClient = null;
//    private TargetFieldPositionTracker aprilTagPositionTrackingCamera;
//    @Override
//    public void teleopInit() {
//        System.out.println("<-- teleop init -->");
//        robotCore = new RemoteControlledRobot();
//        robotCore.initializeRobot();
//        robotBeenEnabledSinceProgram = true;
//
//        aprilTagDetectionAppClient = new JetsonDetectionAppClient("AprilTagDetector", "10.99.99.109", 8888);
//        final double[] targetHeights = new double[] {100, 100, 100, 100, 100, 100};
//        aprilTagPositionTrackingCamera = new FixedAnglePositionTrackingCamera(
//                aprilTagDetectionAppClient,
//                new FixedAngleCameraProfile(
//                        -0.58288,
//                        -0.001342,
//                        -0.002167
//                ),
//                targetHeights
//        );
//        aprilTagDetectionAppClient.startRecognizing();
//    }
//    @Override
//    public void teleopPeriodic() {
//        if (aprilTagPositionTrackingCamera == null) return;
//        aprilTagPositionTrackingCamera.update(new Vector2D(), new Rotation2D(0));
//        final int targetID = 4;
//        final TargetFieldPositionTracker.TargetOnField target;
//        final double x,y,dis,hdg;
//        if ((target = aprilTagPositionTrackingCamera.getTargetByID(targetID)) == null)
//            x = y = dis = hdg = -114514;
//        else {
//            x = target.fieldPosition.getX() * 100;
//            y = target.fieldPosition.getY() * 100;
//            dis = target.fieldPosition.getMagnitude() * 100;
//            hdg = Math.toDegrees(target.fieldPosition.getHeading()) - 90;
//        }
//        SmartDashboard.putNumber("target relative position to camera X (CM)", x);
//        SmartDashboard.putNumber("target relative position to camera Y (CM)", y);
//        SmartDashboard.putNumber("target distance from camera (CM)", dis);
//        SmartDashboard.putNumber("target Ang From Center Line (DEG)", hdg);
//    }
//
//
//    @Override
//    public void autonomousInit() {
//        this.robotCore = new AutonomousRobot(
//                new AprilTagCameraAutomaticMeasuring(
//                        new JetsonDetectionAppClient("AprilTagDetection", "10.99.99.109", 8888),
//                        4,
//                        100,
//                        130,
//                        200,
//                        40,
//                        new Vector2D(new double[] {0, -130})
//                )
//        ); // TODO use sendable chooser to select auto stage
//        robotCore.initializeRobot();
//        robotBeenEnabledSinceProgram = true;
//    }
//    @Override
//    public void autonomousPeriodic() {
//
//    }
//
//    private SimpleRobotTest robotTest = null;
//    @Override
//    public void testInit() {
//        if (robotTest == null)
//            this.robotTest = new BasicIntakeTest();
//        robotTest.testStart();
//    }
//
//    @Override
//    public void testPeriodic() {
//        robotTest.testPeriodic();
//    }
//
//    @Override
//    public void disabledPeriodic() {
//        if (aprilTagDetectionAppClient != null)
//            aprilTagDetectionAppClient.stopRecognizing();
//        // TODO here, alarm is the robot is unused for too long time
//    }
//}
//
