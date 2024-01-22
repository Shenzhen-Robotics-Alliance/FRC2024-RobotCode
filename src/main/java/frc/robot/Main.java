package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutoStagePrograms.AprilTagCameraAutomaticMeasuring;
import frc.robot.AutoStagePrograms.TestAutoStageProgram;
import frc.robot.Drivers.Encoders.CanCoder;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.Drivers.Visions.FixedAnglePositionTrackingCamera;
import frc.robot.Drivers.Visions.JetsonDetectionAppClient;
import frc.robot.Drivers.Visions.TargetFieldPositionTracker;
import frc.robot.Utils.FixedAngleCameraUtils.FixedAngleCameraHorizontalProfileMeasureProcess;
import frc.robot.Utils.FixedAngleCameraUtils.FixedAngleCameraProfile;
import frc.robot.Utils.FixedAngleCameraUtils.FixedAngleCameraVerticalProfileMeasureProcess;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.RobotConfigReader;

/**
 * The entry of the robot's program
 * 
 * @author Sam
 * @version 0.1
 */
public class Main {
    public static void main(String[] args) {
        System.out.printf("@@@@@@  void main(String[] args)  %n");
        RobotBase.startRobot(TimedRobotShell::new);
    }
}

class TimedRobotShell extends TimedRobot {
    Robot robot = null;
    WheelsCalibration calibration;
    boolean robotBeenEnabledSinceProgram = false;
    @Override
    public void robotInit() {
        System.out.println("<--main program starts-->");
    }
    @Override
    public void robotPeriodic() {
        if (robotBeenEnabledSinceProgram && !isTest() && robot != null)
            robot.updateRobot(isEnabled());
    }

    @Override
    public void teleopInit() {
        System.out.println("<-- teleop init -->");
        robot = new RemoteControlledRobot();
        robot.initializeRobot();
        robotBeenEnabledSinceProgram = true;

        aprilTagDetectionAppClient = new JetsonDetectionAppClient("AprilTagDetector", "10.99.99.109", 8888);
        final double[] targetHeights = new double[] {100, 100, 100, 100, 100, 100};
        aprilTagPositionTrackingCamera = new FixedAnglePositionTrackingCamera(
                aprilTagDetectionAppClient,
                new FixedAngleCameraProfile(
                        -0.58288,
                        -0.001342,
                        -0.002167
                ),
                targetHeights
        );
        aprilTagDetectionAppClient.startRecognizing();
    }
    @Override
    public void teleopPeriodic() {
        if (aprilTagPositionTrackingCamera == null) return;
        aprilTagPositionTrackingCamera.update(new Vector2D(), new Rotation2D(0));
        final int targetID = 4;
        final TargetFieldPositionTracker.TargetOnField target;
        final double x,y,dis,hdg;
        if ((target = aprilTagPositionTrackingCamera.getTargetByID(targetID)) == null)
            x = y = dis = hdg = -114514;
        else {
            x = target.fieldPosition.getX() * 100;
            y = target.fieldPosition.getY() * 100;
            dis = target.fieldPosition.getMagnitude() * 100;
            hdg = Math.toDegrees(target.fieldPosition.getHeading()) - 90;
        }
        SmartDashboard.putNumber("target relative position to camera X (CM)", x);
        SmartDashboard.putNumber("target relative position to camera Y (CM)", y);
        SmartDashboard.putNumber("target distance from camera (CM)", dis);
        SmartDashboard.putNumber("target Ang From Center Line (DEG)", hdg);
    }


    @Override
    public void autonomousInit() {
        this.robot = new AutonomousRobot(
                new AprilTagCameraAutomaticMeasuring(
                        new JetsonDetectionAppClient("AprilTagDetection", "10.99.99.109", 8888),
                        4,
                        100,
                        130,
                        200,
                        40,
                        new Vector2D(new double[] {0, -130})
                )
        ); // TODO use sendable chooser to select auto stage
        robot.initializeRobot();
        robotBeenEnabledSinceProgram = true;
    }
    @Override
    public void autonomousPeriodic() {

    }

    private JetsonDetectionAppClient aprilTagDetectionAppClient = null;
    FixedAngleCameraVerticalProfileMeasureProcess verticalProfileMeasureProcess = null;
    FixedAngleCameraHorizontalProfileMeasureProcess horizontalProfileMeasureProcess = null;
    TargetFieldPositionTracker aprilTagPositionTrackingCamera;
    @Override
    public void testInit() {
        this.calibration = new WheelsCalibration();
        calibration.calibrationStart();

//        aprilTagDetectionAppClient = new JetsonDetectionAppClient("AprilTag Detection", "10.99.99.109", 8888);
//        aprilTagDetectionAppClient.startRecognizing();

//        profileMeasureProcess = new FixedAngleCameraVerticalProfileMeasureProcess(
//                new JetsonDetectionAppClient("AprilTag Detection", "10.99.99.109", 8888),
//                5,
//                -60,
//                40,
//                90,
//                0,
//                new XboxController(1)
//        );
//        /*
//        * results:
//        * r^2 = 0.979
//        * rad/pix = 0.001627
//        * cam install ang (rad) = -0.81585
//        *  */

//        horizontalProfileMeasureProcess = new FixedAngleCameraHorizontalProfileMeasureProcess(
//                new JetsonDetectionAppClient("AprilTagDetector", "10.99.99.109", 8888),
//                new double[] {60, 65, 70, 75},
//                15,
//                0,
//                new XboxController(1)
//        );
//
//        /*
//        * result: -0.001307
//        * */

//        aprilTagDetectionAppClient = new JetsonDetectionAppClient("AprilTagDetector", "10.99.99.109", 8888);
//            final double[] targetHeights = new double[] {100, 100, 100, 100, 100, 100};
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
    }

    @Override
    public void testPeriodic() {
        calibration.calibrationPeriodic();

//        aprilTagDetectionAppClient.startRecognizing();
//        aprilTagDetectionAppClient.update();
//        System.out.println("<-- detection results -->");
//        for (RawObjectDetectionCamera.ObjectTargetRaw target:aprilTagDetectionAppClient.getRawTargets())
//            System.out.println(target);
//        System.out.println("<-- the end -->");

//        if (profileMeasureProcess != null)
//            profileMeasureProcess.measuringPeriodic();

//        horizontalProfileMeasureProcess.measuringPeriodic();

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
    }

    @Override
    public void disabledPeriodic() {
        if (aprilTagDetectionAppClient != null) aprilTagDetectionAppClient.stopRecognizing();
        // TODO here, alarm is the robot is unused for too long time
    }
}

class WheelsCalibration { // calibrate wheel
    private CanCoder testCanCoder;
    private TalonFXMotor testSteerMotor;
    private TalonFXMotor testDriveMotor;
    private Joystick testController;
    private RobotConfigReader config;

    private enum Wheel {
        frontLeft, frontRight, backLeft, backRight
    }

    private final SendableChooser<Wheel> wheelsSendableChooser = new SendableChooser<>();

    public void calibrationStart() {
        try {
            config = new RobotConfigReader("slowerChassis");
        } catch (Exception e) {
            throw new RuntimeException();
        }

        for (Wheel wheel : Wheel.values())
            wheelsSendableChooser.addOption(wheel.name(), wheel);
        wheelsSendableChooser.setDefaultOption(Wheel.frontLeft.name(), Wheel.frontLeft);
        SmartDashboard.putData("select wheel to calibrate", wheelsSendableChooser);

        testController = new Joystick(0);
    }

    public void calibrationPeriodic() {
        testDriveMotor = new TalonFXMotor(
                // new TalonFX((int) config.getConfig("hardware", wheelsSendableChooser.getSelected().name() + "WheelDriveMotor"))
                new TalonFX((int) config.getConfig("hardware", wheelsSendableChooser.getSelected().name() + "WheelDriveMotor"), "ChassisCanivore")
        );
        testDriveMotor.gainOwnerShip(null);

        testSteerMotor = new TalonFXMotor(
                // new TalonFX((int) config.getConfig("hardware", wheelsSendableChooser.getSelected().name() + "WheelSteerMotor"))
                new TalonFX((int) config.getConfig("hardware", wheelsSendableChooser.getSelected().name() + "WheelSteerMotor"), "ChassisCanivore")
        );
        testSteerMotor.gainOwnerShip(null);

        testCanCoder = new CanCoder(
                // new CANCoder((int)config.getConfig("hardware", wheelsSendableChooser.getSelected().name() + "WheelEncoder"))
                new CANcoder((int)config.getConfig("hardware", wheelsSendableChooser.getSelected().name() + "WheelEncoder"), "ChassisCanivore")
        );

        SmartDashboard.putNumber("raw steer encoder reading", testCanCoder.getRawSensorReading());
        SmartDashboard.putNumber("raw steer encoder velocity", testCanCoder.getEncoderVelocity());

        if (testController.getRawButton(7))
            testDriveMotor.setPower(0.3, null);
        else
            testDriveMotor.disableMotor(null);

        if (testController.getRawButton(8))
            testSteerMotor.setPower(0.3, null);
        else
            testSteerMotor.disableMotor(null);
    }
}