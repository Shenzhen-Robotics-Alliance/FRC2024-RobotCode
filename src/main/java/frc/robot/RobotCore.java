package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.DistanceSensors.Rev2mDistanceSensorEncapsulation;
import frc.robot.Drivers.Encoders.CanCoder;
import frc.robot.Drivers.Encoders.DCAbsolutePositionEncoder;
import frc.robot.Drivers.IMUs.PigeonsIMU;
import frc.robot.Drivers.IMUs.SimpleGyro;
import frc.robot.Drivers.Motors.Motor;
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
import frc.robot.Services.TransformableIntakeAndShooterService;
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
        public final RobotConfigReader robotConfig;
        public final SwerveWheel frontLeftWheel, backLeftWheel, frontRightWheel, backRightWheel;
        public final SimpleGyro gyro;
        public final SwerveWheelPositionEstimator positionReader;
        public final SwerveBasedChassis chassisModule;
        public final JetsonDetectionAppClient aprilTagDetectionAppClient;
        public final TargetFieldPositionTracker aprilTagPositionTrackingCamera;

        public TransformableArm transformableArm;
        public Intake intake;
        public Shooter shooter;

        private final List<String> configsToTune = new ArrayList<>(1);
        private final List<RobotModuleBase> modules;
        private List<RobotServiceBase> services;
        private List<Thread> modulesUpdatingThreads;
        private final boolean useMultiThreads;
        protected boolean wasEnabled;

        public RobotCore(String configNme) {
                this(configNme, true);
        }
        /**
         * creates a robot core
         * creates the instances of all the modules, but do not call init functions yet
         * @param useMultiThreads whether the robot modules should be updated in independent threads
         * */
        public RobotCore(String configName, boolean useMultiThreads) {
                System.out.println("<-- Robot Core | creating robot... -->");
                this.useMultiThreads = useMultiThreads;
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
                final double[] targetHeights = new double[] {130, 130, 130, 130, 130, 130};
                aprilTagPositionTrackingCamera = new FixedAnglePositionTrackingCamera( // TODO load and save to robotConfig
                        aprilTagDetectionAppClient,
                        new FixedAngleCameraProfile(
                                0.6724,
                                -0.00284,
                                0.00208
                        ),
                        targetHeights
                );
                final Map<Integer, Vector2D> speakerTargetAprilTagReferences = new HashMap<>(), amplifierTargetAprilTagReferences = new HashMap<>(), noteTargetReferences = new HashMap<>();
                final DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red); // default to red
                if (alliance == DriverStation.Alliance.Red) {
                        speakerTargetAprilTagReferences.put(4, new Vector2D(new double[] {0,0}));
                        speakerTargetAprilTagReferences.put(3, new Vector2D(new double[] {0.565,0}));
                        amplifierTargetAprilTagReferences.put(5, new Vector2D(new double[] {0, 0}));
                } else {
                        speakerTargetAprilTagReferences.put(7, new Vector2D(new double[] {0,0}));
                        speakerTargetAprilTagReferences.put(8, new Vector2D(new double[] {-0.565,0}));
                        amplifierTargetAprilTagReferences.put(6, new Vector2D(new double[] {0, 0}));
                }
                noteTargetReferences.put(0, new Vector2D()); // the id of note is always 0, and the note is itself the reference so the relative position is (0,0)
                final AprilTagReferredTarget speakerTarget = new AprilTagReferredTarget(aprilTagPositionTrackingCamera, speakerTargetAprilTagReferences),
                        amplifierTarget = new AprilTagReferredTarget(aprilTagPositionTrackingCamera, amplifierTargetAprilTagReferences),
                        /* TODO note target tracking camera */
                        noteTarget = new AprilTagReferredTarget(aprilTagPositionTrackingCamera, noteTargetReferences); // we call it april tag referred target but it is actually recognized by detect-net app
                final Shooter.AimingSystem aimingSystem = new Shooter.AimingSystem(positionReader, speakerTarget, 1000);

                final TalonFXMotor armMotor = new TalonFXMotor(new TalonFX(25) ,robotConfig.getConfig("arm/armMotorReversed")!=0);
                final DCAbsolutePositionEncoder armEncoder = new DCAbsolutePositionEncoder(1, robotConfig.getConfig("arm/armEncoderReversed")!=0);
                this.transformableArm = new TransformableArm(armMotor, armEncoder, robotConfig); modules.add(transformableArm);
                final MotorsSet intakeMotors = new MotorsSet(
                        new Motor[] {
                                new TalonFXMotor(new TalonFX(13), true),
                                new TalonFXMotor(new TalonFX(14), true)
                        });
                this.intake = new IntakeWithDistanceSensor(intakeMotors, new Rev2mDistanceSensorEncapsulation(), robotConfig); modules.add(intake);
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

                if (useMultiThreads)
                        scheduleThreadsAndRun();

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

                System.out.println("<-- Robot | robot paused... -->");
        }

        private void scheduleThreadsAndRun() {
                modulesUpdatingThreads = new ArrayList<>();
                for (RobotModuleBase module: modules)
                        modulesUpdatingThreads.add(module.getPeriodicUpdateThread());
                for (Thread thread: modulesUpdatingThreads)
                        thread.start();
        }

        /**
         * called when the robot is enabled
         * */
        private long t = System.currentTimeMillis();
        public void updateRobot() {
                for (RobotServiceBase service: services)
//                        service.periodic();
                {
                        long dt = System.currentTimeMillis();
                        service.periodic();
//                        if (System.currentTimeMillis()-dt>10)
//                                System.out.println("service " + service.serviceName + " update time (ms): " + (System.currentTimeMillis() - dt));
                }

                if (aprilTagPositionTrackingCamera != null)
                        aprilTagPositionTrackingCamera.update(positionReader.getRobotPosition2D(), new Rotation2D(positionReader.getRobotRotation()));
                if (!useMultiThreads)
                        for (RobotModuleBase module:modules)
                                module.periodic();

                // printChassisDebugMessages();
                long dt = System.currentTimeMillis();
                printAprilTagCameraResultsToDashboard();
                printChassisDebugMessagesToDashboard();
//                if (System.currentTimeMillis()-dt>10)
//                        System.out.println("print april tag camera time (ms): " + (System.currentTimeMillis() - dt));

                dt = System.currentTimeMillis();
                robotConfig.updateTuningConfigsFromDashboard();
//                if (System.currentTimeMillis()-dt>10)
//                        System.out.println("update config time (ms): " + (System.currentTimeMillis() - dt));

                /* monitor the program's performance */
                SmartDashboard.putNumber("robot main thread delay", System.currentTimeMillis()-t);
                t = System.currentTimeMillis();
        }

        private void printChassisDebugMessagesToDashboard() {
                SmartDashboard.putNumber("robot x", positionReader.getRobotPosition2D().getValue()[0]);
                SmartDashboard.putNumber("robot y", positionReader.getRobotPosition2D().getValue()[1]);
                SmartDashboard.putNumber("velocity x", positionReader.getRobotVelocity2D().getValue()[0]);
                SmartDashboard.putNumber("velocity y", positionReader.getRobotVelocity2D().getValue()[1]);
        }

        private void printAprilTagCameraResultsToDashboard() {
                if (aprilTagPositionTrackingCamera == null)
                        return;
                final int targetID = 4;
                final TargetFieldPositionTracker.TargetOnField target;
                final double x,y,dis,hdg;
                if ((target = aprilTagPositionTrackingCamera.getVisibleTargetByID(targetID)) == null)
                        x = y = dis = hdg = -1;
                else {
                        x = target.fieldPosition.getX() * 100;
                        y = target.fieldPosition.getY() * 100;
                        dis = target.fieldPosition.getMagnitude() * 100;
                        hdg = Math.toDegrees(target.fieldPosition.getHeading()) - 90;
                }
                EasyShuffleBoard.putNumber("apriltag", "target relative position to camera X (CM)", x);
                EasyShuffleBoard.putNumber("apriltag", "target relative position to camera Y (CM)", y);
                EasyShuffleBoard.putNumber("apriltag", "target distance from camera (CM)", dis);
                EasyShuffleBoard.putNumber("apriltag", "target Ang From Center Line (DEG)", hdg);
        }
}