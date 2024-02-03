package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.Encoders.CanCoder;
import frc.robot.Drivers.IMUs.PigeonsIMU;
import frc.robot.Drivers.IMUs.SimpleGyro;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.Drivers.Visions.FixedAnglePositionTrackingCamera;
import frc.robot.Drivers.Visions.JetsonDetectionAppClient;
import frc.robot.Drivers.Visions.TargetFieldPositionTracker;
import frc.robot.Modules.PositionReader.SwerveWheelPositionEstimator;
import frc.robot.Modules.PositionReader.SwerveWheelPositionEstimatorCurveOptimized;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Modules.Chassis.SwerveBasedChassis;
import frc.robot.Modules.Chassis.SwerveWheel;
import frc.robot.Services.RobotServiceBase;
import frc.robot.Utils.*;
import frc.robot.Utils.FixedAngleCameraUtils.FixedAngleCameraProfile;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;

/**
 *
 * the core of the robot, including all the modules that powers the module
 * note that services are not included in this field
 * */
// TODO: we have to sort the shuffleboard, it's too messy now
public class RobotCore {
        public final RobotConfigReader robotConfig;
        public final SwerveWheel frontLeftWheel, backLeftWheel, frontRightWheel, backRightWheel;//四驱
        public final SimpleGyro gyro;//陀螺仪
        public final SwerveWheelPositionEstimator positionReader;//舵轮定位
        public final SwerveBasedChassis chassisModule;//底盘的整体module
        public final JetsonDetectionAppClient aprilTagDetectionAppClient;//vision
        public final TargetFieldPositionTracker aprilTagPositionTrackingCamera;
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
                // ^^ 初始化四个轮子,加入module类中
                frontLeftWheel = createSwerveWheel("frontLeft", 1, new Vector2D(new double[] { -0.6, 0.6 }));
                modules.add(frontLeftWheel);

                backLeftWheel = createSwerveWheel("backLeft", 2, new Vector2D(new double[] { -0.6, -0.6 }));
                modules.add(backLeftWheel);

                frontRightWheel = createSwerveWheel("frontRight", 3, new Vector2D(new double[] { 0.6, 0.6 }));
                modules.add(frontRightWheel);

                backRightWheel = createSwerveWheel("backRight", 4, new Vector2D(new double[] { 0.6, -0.6 }));
                modules.add(backRightWheel);
                // initialize gyro
                this.gyro = new SimpleGyro(0, false, new PigeonsIMU((int) robotConfig.getConfig("hardware/gyroPort")));

                final SwerveWheel[] swerveWheels = new SwerveWheel[] {frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel};
                positionReader = new SwerveWheelPositionEstimator(swerveWheels, gyro);// read current position
                modules.add(positionReader);

                SwerveWheelPositionEstimatorCurveOptimized testPositionEstimator = new SwerveWheelPositionEstimatorCurveOptimized(swerveWheels, gyro);//?
                modules.add(testPositionEstimator);

                this.chassisModule = new SwerveBasedChassis(swerveWheels, gyro, robotConfig, positionReader);
                modules.add(chassisModule);// ^^ the whole chassis as a module, used to estimate the robot velocity and postion

                aprilTagDetectionAppClient = new JetsonDetectionAppClient("AprilTagDetector", "10.55.16.109", 8888);
                final double[] targetHeights = new double[] {130, 130, 130, 130, 130, 130};
                aprilTagPositionTrackingCamera = new FixedAnglePositionTrackingCamera( // TODO load and save to robotConfig
                        aprilTagDetectionAppClient,
                        new FixedAngleCameraProfile(
                                0.37725,
                                -0.00284,
                                -0.00203
                        ),
                        targetHeights
                );
        }

        private SwerveWheel createSwerveWheel(String name, int id, Vector2D wheelInstallationPosition) {
                if (robotConfig.getConfig("hardware/chassisOnCanivore") != 0)
                        return createSwerveWheelOnCanivore(name, id, wheelInstallationPosition);// ^^ Canivore is a extended can bus that used for extra wires since a normal can bus can only handle 10 wires
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

        private SwerveWheel createSwerveWheelOnCanivore(String name, int id, Vector2D wheelInstallationPosition) {// ^^ can bus的额外拓展板
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

        private void addConfigsToTune() {//?
                /* feed forward controller */
//                configsToTune.add("chassis/driveWheelFeedForwardRate");
//                configsToTune.add("chassis/driveWheelFrictionDefaultValue");
//                configsToTune.add("chassis/timeNeededToFullyAccelerate");

                /* steer PID */
                configsToTune.add("chassis/steerWheelErrorTolerance");
                configsToTune.add("chassis/steerWheelErrorStartDecelerate");
                configsToTune.add("chassis/steerWheelMaximumPower");
                configsToTune.add("chassis/steerWheelMinimumPower");
                configsToTune.add("chassis/steerWheelFeedForwardTime");
                configsToTune.add("chassis/steerCorrectionPowerRateAtZeroWheelSpeed");
                configsToTune.add("chassis/steerCorrectionPowerFullWheelSpeed");
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
        private long t = System.currentTimeMillis();// ^^ currentTimeMillis is the total time after 1970-1-01
        public void updateRobot() {
                for (RobotServiceBase service: services)
                        service.periodic();

                if (!useMultiThreads)
                        for (RobotModuleBase module:modules)
                                module.periodic();

                // printChassisDebugMessages();
                printAprilTagCameraResultsToDashboard();

                robotConfig.updateTuningConfigsFromDashboard();

                /* monitor the program's performance */
                SmartDashboard.putNumber("robot main thread rate", 1000/(System.currentTimeMillis()-t));// ^^ smart dashboard 是历史的调试软件，现在使用shuffleboard（shuffleboard仍旧支持运行smart dashboard 的语句）
                // ^^ System.currentTimeMillis()-t求出经过的时间
                t = System.currentTimeMillis();
        }

        private void printChassisDebugMessagesToDashboard() {// ^^ 调试
                SmartDashboard.putNumber("robot x", positionReader.getRobotPosition2D().getValue()[0]);
                SmartDashboard.putNumber("robot y", positionReader.getRobotPosition2D().getValue()[1]);
                SmartDashboard.putNumber("velocity x", positionReader.getRobotVelocity2D().getValue()[0]);
                SmartDashboard.putNumber("velocity y", positionReader.getRobotVelocity2D().getValue()[1]);
        }

        private void printAprilTagCameraResultsToDashboard() {// ^^ 调试
                if (aprilTagPositionTrackingCamera == null) return;
                aprilTagPositionTrackingCamera.update(new Vector2D(), new Rotation2D(0));
                final int targetID = 4;
                final TargetFieldPositionTracker.TargetOnField target;
                final double x,y,dis,hdg;
                if ((target = aprilTagPositionTrackingCamera.getTargetByID(targetID)) == null)
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