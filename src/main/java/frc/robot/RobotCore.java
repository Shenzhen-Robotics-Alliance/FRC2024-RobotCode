package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.DistanceSensors.Rev2mDistanceSensorEncapsulation;
import frc.robot.Drivers.Encoders.CanCoder;
import frc.robot.Drivers.Encoders.DCAbsolutePositionEncoder;
import frc.robot.Drivers.IMUs.PigeonsIMU;
import frc.robot.Drivers.IMUs.SimpleGyro;
import frc.robot.Drivers.Motors.MotorsSet;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.Drivers.Visions.FixedAnglePositionTrackingCamera;
import frc.robot.Drivers.Visions.JetsonDetectionAppClient;
import frc.robot.Drivers.Visions.TargetFieldPositionTracker;
import frc.robot.Modules.PositionReader.SwerveWheelPositionEstimator;
import frc.robot.Modules.PositionReader.SwerveWheelPositionEstimatorCurveOptimized;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Modules.Chassis.SwerveBasedChassis;
import frc.robot.Modules.Chassis.SwerveWheel;
import frc.robot.Modules.UpperStructure.Intake;
import frc.robot.Modules.UpperStructure.IntakeWithDistanceSensor;
import frc.robot.Modules.UpperStructure.Shooter;
import frc.robot.Modules.UpperStructure.TransformableArm;
import frc.robot.Services.RobotServiceBase;
import frc.robot.Utils.*;
import frc.robot.Utils.ComputerVisionUtils.AprilTagReferredTarget;
import frc.robot.Utils.ComputerVisionUtils.FixedAngleCameraProfile;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.MechanismControllers.EncoderMotorMechanism;

/**
 *
 * the core of the robot, including all the modules that powers the module
 * note that services are not included in this field
 * */
public class RobotCore {
        private static final long printTimeIfTimeMillisExceeds = 2;

        public final RobotConfigReader robotConfig;
        public final SwerveWheel frontLeftWheel, backLeftWheel, frontRightWheel, backRightWheel;
        public final SimpleGyro gyro;
        public final SwerveWheelPositionEstimator positionReader;
        public final SwerveBasedChassis chassisModule;
        public final JetsonDetectionAppClient aprilTagDetectionAppClient, noteDetectionAppClient;
        public final TargetFieldPositionTracker aprilTagPositionTrackingCamera, notePositionTrackingCamera;

        public final TransformableArm transformableArm;
        public final Intake intake;
        public final Shooter shooter;
        public final AprilTagReferredTarget speakerTarget, amplifierTarget, noteTarget;

        private final List<String> configsToTune = new ArrayList<>(1);
        private final List<RobotModuleBase> modules;
        private List<RobotServiceBase> services;
        protected boolean wasEnabled;
        private Vector2D chassisCurrentPositionForCameraCalculation = new Vector2D();
        private double chassisCurrentRotationForCameraCalculation = 0;
        /**
         * creates a robot core
         * creates the instances of all the modules, but do not call init functions yet
         * */
        public RobotCore(String configName) {
                System.out.println("<-- Robot Core | creating robot... -->");
                modules = new ArrayList<>();
                services = new ArrayList<>();

                robotConfig = new RobotConfigReader(configName);

                frontLeftWheel = createSwerveWheel("frontLeft", 1, new Vector2D(new double[] { -0.6, 0.6 }));
                modules.add(frontLeftWheel);

                backLeftWheel = createSwerveWheel("backLeft", 2, new Vector2D(new double[] { -0.6, -0.6 }));
                modules.add(backLeftWheel);

                frontRightWheel = createSwerveWheel("frontRight", 3, new Vector2D(new double[] { 0.6, 0.6 }));
                modules.add(frontRightWheel);

                backRightWheel = createSwerveWheel("backRight", 4, new Vector2D(new double[] { 0.6, -0.6 }));
                modules.add(backRightWheel);

                this.gyro = new SimpleGyro(0, false, new PigeonsIMU((int) robotConfig.getConfig("hardware/gyroPort")));

                final SwerveWheel[] swerveWheels = new SwerveWheel[] {frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel};
                positionReader = new SwerveWheelPositionEstimator(swerveWheels, gyro);
                modules.add(positionReader);

                SwerveWheelPositionEstimatorCurveOptimized testPositionEstimator = new SwerveWheelPositionEstimatorCurveOptimized(swerveWheels, gyro);
                modules.add(testPositionEstimator);

                this.chassisModule = new SwerveBasedChassis(swerveWheels, gyro, robotConfig, positionReader);
                modules.add(chassisModule);


                /* TODO: the following into robot config */
                aprilTagDetectionAppClient = new JetsonDetectionAppClient("AprilTagDetector", "10.55.16.109", 8888);
                final double[] targetHeights = new double[] {100, 100, 100, 100, 100, 100};
                aprilTagPositionTrackingCamera = new FixedAnglePositionTrackingCamera(
                        aprilTagDetectionAppClient,
                        new FixedAngleCameraProfile(
                                0.385,
                                -0.0018,
                                -0.00179
                        ),
                        targetHeights
                );
                noteDetectionAppClient = new JetsonDetectionAppClient("NoteDetector", "10.55.16.109", 8889, new double[] {1280, 720});
                notePositionTrackingCamera = new FixedAnglePositionTrackingCamera(
                        noteDetectionAppClient,
                        new FixedAngleCameraProfile(
                                -1.0615,
                                -0.002,
                                -0.00148
                        ),
                        new double[] {-30, -30, -30},
                        new Rotation2D(Math.PI)
                );

                final Map<Integer, Vector2D> speakerTargetAprilTagReferences = new HashMap<>(), amplifierTargetAprilTagReferences = new HashMap<>(), noteTargetReferences = new HashMap<>();
                final DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red); // default to red
                if (alliance == DriverStation.Alliance.Red) {
                        speakerTargetAprilTagReferences.put(4, new Vector2D(new double[] {0.05,0}));
                        speakerTargetAprilTagReferences.put(3, new Vector2D(new double[] {-0.5,0}));
                        amplifierTargetAprilTagReferences.put(5, new Vector2D(new double[] {0, 0}));
                } else {
                        speakerTargetAprilTagReferences.put(7, new Vector2D(new double[] {0.05,0}));
                        speakerTargetAprilTagReferences.put(8, new Vector2D(new double[] {-0.5,0}));
                        amplifierTargetAprilTagReferences.put(6, new Vector2D(new double[] {0, 0}));
                }
                noteTargetReferences.put(1, new Vector2D()); // the id of note is always 0, and the note is itself the reference so the relative position is (0,0)
                speakerTarget = new AprilTagReferredTarget(aprilTagPositionTrackingCamera, speakerTargetAprilTagReferences);
                amplifierTarget = new AprilTagReferredTarget(aprilTagPositionTrackingCamera, amplifierTargetAprilTagReferences);

                noteTarget = new AprilTagReferredTarget(notePositionTrackingCamera, noteTargetReferences); // we call it april tag referred target but it is actually recognized by detect-net app
                final Shooter.AimingSystem aimingSystem = new Shooter.AimingSystem(positionReader, speakerTarget, (long) robotConfig.getConfig("vision-autopilot/aimingTimeUnseenToleranceMS"));
                final EncoderMotorMechanism[] shooterMechanisms = new EncoderMotorMechanism[] {
                        new TalonFXMotor(
                                new TalonFX((int)robotConfig.getConfig("shooter/shooter1Port")),
                                robotConfig.getConfig("shooter/shooter1Reversed") != 0
                        ).toEncoderAndMotorMechanism(),
                        new TalonFXMotor(
                                new TalonFX((int)robotConfig.getConfig("shooter/shooter2Port")),
                                robotConfig.getConfig("shooter/shooter2Reversed") != 0
                        ).toEncoderAndMotorMechanism()
                };
                this.shooter = new Shooter(shooterMechanisms, aimingSystem, robotConfig); modules.add(shooter);


                final TalonFXMotor armMotor = new TalonFXMotor(new TalonFX((int)robotConfig.getConfig("arm/armMotorPort")) ,robotConfig.getConfig("arm/armMotorReversed")!=0);
                final DCAbsolutePositionEncoder armEncoder = new DCAbsolutePositionEncoder(0, robotConfig.getConfig("arm/armEncoderReversed")!=0);
                this.transformableArm = new TransformableArm(armMotor, armEncoder, shooter, robotConfig); modules.add(transformableArm);
                final TalonFXMotor[] intakeMotors = new TalonFXMotor[] {
                        new TalonFXMotor(new TalonFX((int)robotConfig.getConfig("intake/intakeMotor1Port")), robotConfig.getConfig("intake/intakeMotor1Reversed") != 0),
                        new TalonFXMotor(new TalonFX((int)robotConfig.getConfig("intake/intakeMotor2Port")), robotConfig.getConfig("intake/intakeMotor2Reversed") != 0)
                };

                this.intake = new IntakeWithDistanceSensor(new MotorsSet(intakeMotors), intakeMotors[0], new Rev2mDistanceSensorEncapsulation(), robotConfig); modules.add(intake);
        }

        private SwerveWheel createSwerveWheel(String name, int id, Vector2D wheelInstallationPosition) {
                if (robotConfig.getConfig("hardware/chassisOnCanivore") != 0)
                        return createSwerveWheelOnCanivore(name, id, wheelInstallationPosition);
                return new SwerveWheel(
                        new TalonFXMotor(new TalonFX( (int) robotConfig.getConfig("hardware/"+name+"WheelDriveMotor"))),
                        new TalonFXMotor(new TalonFX( (int) robotConfig.getConfig("hardware/"+name+"WheelSteerMotor")), robotConfig.getConfig("hardware/"+name+"WheelSteerMotorReversed") == 1),
                        new TalonFXMotor(new TalonFX((int) robotConfig.getConfig("hardware/"+name+"WheelDriveMotor"))),
                        new CanCoder(new CANcoder((int) robotConfig.getConfig("hardware/"+name+"WheelEncoder")),
                                robotConfig.getConfig("hardware/"+name+"WheelSteerEncoderReversed") == 1),
                        wheelInstallationPosition,
                        robotConfig, 
                        id, 
                        robotConfig.getConfig("hardware/"+name+"WheelZeroPosition") 
                                + (robotConfig.getConfig("hardware/"+name+"WheelSteerEncoderReversed") == 1 ? 
                                        (Math.PI / 2) : (-Math.PI / 2))
                );
        }

        private SwerveWheel createSwerveWheelOnCanivore(String name, int id, Vector2D wheelInstallationPosition) {
                return new SwerveWheel(
                        new TalonFXMotor(new TalonFX( (int) robotConfig.getConfig("hardware/"+name+"WheelDriveMotor"), "ChassisCanivore")),
                        new TalonFXMotor(new TalonFX( (int) robotConfig.getConfig("hardware/"+name+"WheelSteerMotor"), "ChassisCanivore"), robotConfig.getConfig("hardware/"+name+"WheelSteerMotorReversed") == 1),
                        new TalonFXMotor(new TalonFX((int) robotConfig.getConfig("hardware/"+name+"WheelDriveMotor"), "ChassisCanivore")),
                        new CanCoder(new CANcoder((int) robotConfig.getConfig("hardware/"+name+"WheelEncoder"), "ChassisCanivore"),
                                robotConfig.getConfig("hardware/"+name+"WheelSteerEncoderReversed") == 1),
                        wheelInstallationPosition,
                        robotConfig,
                        id,
                        robotConfig.getConfig("hardware/"+name+"WheelZeroPosition")
                                + (robotConfig.getConfig("hardware/"+name+"WheelSteerEncoderReversed") == 1 ?
                                (Math.PI / 2) : (-Math.PI / 2))
                );
        }

        /**
         * initializes the robot
         * note that this will take a little bit of time as it involves creating threads
         * it should be called once each competition, when the driver station connects to the robot
         * */
        public void initializeRobot() {
                System.out.println("<-- Robot | initializing robot... -->");
                /* initialize the modules and services */
                for (RobotModuleBase module:modules)
                        module.init();

                /* start the config tuning */
                addConfigsToTune();
                for (String config:configsToTune)
                        robotConfig.startTuningConfig(config);

                System.out.println("<-- Robot | robot initialized -->");
        }

        private void addConfigsToTune() {
                /* feed forward controller */
//                configsToTune.add("chassis/driveWheelFeedForwardRate");
//                configsToTune.add("chassis/driveWheelFrictionDefaultValue");
//                configsToTune.add("chassis/timeNeededToFullyAccelerate");

                /* steer PID */
//                configsToTune.add("chassis/steerWheelErrorTolerance");
//                configsToTune.add("chassis/steerWheelErrorStartDecelerate");
//                configsToTune.add("chassis/steerWheelMaximumPower");
//                configsToTune.add("chassis/steerWheelMinimumPower");
//                configsToTune.add("chassis/steerWheelFeedForwardTime");
//                configsToTune.add("chassis/steerCorrectionPowerRateAtZeroWheelSpeed");
//                configsToTune.add("chassis/steerCorrectionPowerFullWheelSpeed");

                /* arm pid */
//                configsToTune.add("arm/maximumPower");
//                configsToTune.add("arm/errorStartDecelerate");
//                configsToTune.add("arm/errorTolerance");
//                configsToTune.add("arm/feedForwardTime");
//                configsToTune.add("arm/errorAccumulationProportion");
//                configsToTune.add("arm/maxAcceleration");
//                configsToTune.add("arm/maxVelocity");
//                configsToTune.add("arm/inAdvanceTime");
//                configsToTune.add("arm/errorToleranceAsInPosition");

                /* arm positions */
                configsToTune.add("arm/position-DEFAULT");
                configsToTune.add("arm/position-INTAKE");
                configsToTune.add("arm/position-SHOOT_NOTE");
                configsToTune.add("arm/position-SCORE_AMPLIFIER");

                configsToTune.add("shooter/defaultShootingRPM");
                configsToTune.add("shooter/speedControllerProportionGain");
                configsToTune.add("shooter/speedControllerFeedForwardGain");
//                configsToTune.add("shooter/shooterRPM0");
//                configsToTune.add("shooter/armAngle0");
//                configsToTune.add("shooter/shooterRPM1");
//                configsToTune.add("shooter/armAngle1");
//                configsToTune.add("shooter/shooterRPM2");
//                configsToTune.add("shooter/armAngle2");
//                configsToTune.add("shooter/shooterRPM3");
//                configsToTune.add("shooter/armAngle3");
//                configsToTune.add("shooter/shooterRPM4");
//                configsToTune.add("shooter/armAngle4");
//                configsToTune.add("shooter/shooterRPM5");
//                configsToTune.add("shooter/armAngle5");
        }

        /**
         * resets the robot the current stage
         * this is called once at the start of each stage (auto or teleop)
         * @param services the robot services that will be used this stage
         * */
        public void startStage(List<RobotServiceBase> services) {
                this.services = services;
                System.out.println("<-- Robot Core | starting current stage... -->");
                /* initialize the services */
                for (RobotServiceBase service:services)
                        service.init();
                /* reset the modules and services */
                for (RobotModuleBase module: modules)
                        module.reset();
                for (RobotServiceBase service: services)
                        service.reset();
                /* resume the modules that was paused */
                for (RobotModuleBase module: modules)
                        module.enable();

                aprilTagDetectionAppClient.startRecognizing();
                noteDetectionAppClient.startRecognizing();

                wasEnabled = true;
                System.out.println("<-- Robot Core | current stage started -->");
        }

        /**
         * end the current stage
         * */
        public void stopStage() {
                System.out.println("<-- Robot | pausing robot... -->");
                this.wasEnabled = false;
                for (RobotModuleBase module: modules)
                        module.disable();
                this.services = new ArrayList<>();

                aprilTagDetectionAppClient.stopRecognizing();
                noteDetectionAppClient.stopRecognizing();

                System.out.println("<-- Robot | robot paused... -->");
        }

        /**
         * called when the robot is enabled
         * */
        private long t = System.currentTimeMillis();
        public void updateRobot() {
                chassisCurrentPositionForCameraCalculation = positionReader.getRobotPosition2D();
                chassisCurrentRotationForCameraCalculation = positionReader.getRobotRotation();
                updateAprilTagCamera();
                updateNoteCamera();

                for (RobotServiceBase service: services)
                        service.periodic();
                updateModules();



                printChassisDebugMessagesToDashboard();


                robotConfig.updateTuningConfigsFromDashboard();

                /* monitor the program's performance */
                SmartDashboard.putNumber("robot main thread delay", System.currentTimeMillis()-t);
                t = System.currentTimeMillis();
        }

        public void updateModules() {
                for (RobotModuleBase module:modules)
                        module.periodic();
        }

        private void printChassisDebugMessagesToDashboard() {
                SmartDashboard.putNumber("robot x", positionReader.getRobotPosition2D().getValue()[0]);
                SmartDashboard.putNumber("robot y", positionReader.getRobotPosition2D().getValue()[1]);
                SmartDashboard.putNumber("velocity x", positionReader.getRobotVelocity2D().getValue()[0]);
                SmartDashboard.putNumber("velocity y", positionReader.getRobotVelocity2D().getValue()[1]);
        }

        private void updateAprilTagCamera() {
                long dt = System.currentTimeMillis();
                if (aprilTagPositionTrackingCamera != null)
                        aprilTagPositionTrackingCamera.update(chassisCurrentPositionForCameraCalculation, new Rotation2D(chassisCurrentRotationForCameraCalculation));
                if (System.currentTimeMillis()-dt > printTimeIfTimeMillisExceeds)
                        System.out.println("update april tag camera time (ms): " + (System.currentTimeMillis() - dt));

                printAprilTagCameraResultsToDashboard();


                if (System.currentTimeMillis()-dt > printTimeIfTimeMillisExceeds)
                        System.out.println("print april camera messages time (ms): " + (System.currentTimeMillis() - dt));
        }

        private void updateNoteCamera() {
                long dt = System.currentTimeMillis();
                if (notePositionTrackingCamera != null)
                        notePositionTrackingCamera.update(chassisCurrentPositionForCameraCalculation, new Rotation2D(chassisCurrentRotationForCameraCalculation));
                if (System.currentTimeMillis()-dt > printTimeIfTimeMillisExceeds)
                        System.out.println("update note camera time (ms): " + (System.currentTimeMillis() - dt));

                dt=System.currentTimeMillis();
                printNoteDetectionCameraResultsToDashboard();
                if (System.currentTimeMillis()-dt > printTimeIfTimeMillisExceeds)
                        System.out.println("print note camera messages time (ms): " + (System.currentTimeMillis() - dt));
        }

        private void printAprilTagCameraResultsToDashboard() {
                if (aprilTagPositionTrackingCamera == null)
                        return;

                final Vector2D speakerFieldPosition =  this.speakerTarget.getTargetFieldPositionWithAprilTags(500);
                if (speakerFieldPosition == null) {
                        EasyShuffleBoard.putNumber("apriltag", "target absolute field position X", 0);
                        EasyShuffleBoard.putNumber("apriltag", "target absolute field position Y", 0);
                        EasyShuffleBoard.putNumber("apriltag", "target relative position to robot X", 0);
                        EasyShuffleBoard.putNumber("apriltag", "target relative position to robot Y", 0);
                        EasyShuffleBoard.putNumber("apriltag", "target distance from camera (CM)", 0);
                        return;
                }
                final Vector2D speakerRelativePositionToRobot = Vector2D.displacementToTarget(chassisCurrentPositionForCameraCalculation, speakerFieldPosition);
                EasyShuffleBoard.putNumber("apriltag", "target absolute field position X", speakerFieldPosition.getX());
                EasyShuffleBoard.putNumber("apriltag", "target absolute field position Y", speakerFieldPosition.getY());
                EasyShuffleBoard.putNumber("apriltag", "target relative position to robot X", speakerRelativePositionToRobot.getX());
                EasyShuffleBoard.putNumber("apriltag", "target relative position to robot Y", speakerRelativePositionToRobot.getY());
                EasyShuffleBoard.putNumber("apriltag", "target distance from camera (M)", speakerRelativePositionToRobot.getMagnitude());

                final TargetFieldPositionTracker.TargetOnField aprilTag3 = aprilTagPositionTrackingCamera.getVisibleTargetByID(3),
                        aprilTag4 = aprilTagPositionTrackingCamera.getVisibleTargetByID(4);
                if (aprilTag3 != null) {
                        EasyShuffleBoard.putNumber("apriltag", "Tag 3 position to robot X", aprilTag3.fieldPosition.getX() - chassisCurrentPositionForCameraCalculation.getX());
                        EasyShuffleBoard.putNumber("apriltag", "Tag 3 position to robot Y", aprilTag3.fieldPosition.getY() - chassisCurrentPositionForCameraCalculation.getY());
                }
                if (aprilTag4 != null) {
                        EasyShuffleBoard.putNumber("apriltag", "Tag 4 position to robot X", aprilTag4.fieldPosition.getX() - chassisCurrentPositionForCameraCalculation.getX());
                        EasyShuffleBoard.putNumber("apriltag", "Tag 4 position to robot Y", aprilTag4.fieldPosition.getY() - chassisCurrentPositionForCameraCalculation.getY());
                }

                SmartDashboard.putNumber("target distance from camera (M)", speakerRelativePositionToRobot.getMagnitude());
        }

        private void printNoteDetectionCameraResultsToDashboard() {
                if (notePositionTrackingCamera == null)
                        return;

                final Vector2D notePosition =  this.noteTarget.getTargetFieldPositionWithAprilTags(500);
                if (notePosition == null) {
                        EasyShuffleBoard.putNumber("note-detection", "target absolute field position X", 0);
                        EasyShuffleBoard.putNumber("note-detection", "target absolute field position Y", 0);
                        EasyShuffleBoard.putNumber("note-detection", "target relative position to robot X", 0);
                        EasyShuffleBoard.putNumber("note-detection", "target relative position to robot Y", 0);
                        EasyShuffleBoard.putNumber("note-detection", "target distance from camera (CM)", 0);
                        return;
                }
                final Vector2D noteRelativePositionToRobot = Vector2D.displacementToTarget(chassisCurrentPositionForCameraCalculation, notePosition);
                EasyShuffleBoard.putNumber("note-detection", "target absolute field position X", notePosition.getX());
                EasyShuffleBoard.putNumber("note-detection", "target absolute field position Y", notePosition.getY());
                EasyShuffleBoard.putNumber("note-detection", "target relative position to robot X", noteRelativePositionToRobot.getX());
                EasyShuffleBoard.putNumber("note-detection", "target relative position to robot Y", noteRelativePositionToRobot.getY());
                EasyShuffleBoard.putNumber("note-detection", "target distance from camera (CM)", noteRelativePositionToRobot.getMagnitude());
        }
}