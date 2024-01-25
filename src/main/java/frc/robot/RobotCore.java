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
import frc.robot.Modules.PositionReader.SwerveWheelPositionEstimator;
import frc.robot.Modules.PositionReader.SwerveWheelPositionEstimatorCurveOptimized;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Modules.SwerveBasedChassis;
import frc.robot.Modules.SwerveWheel;
import frc.robot.Services.RobotServiceBase;
import frc.robot.Utils.*;
import frc.robot.Utils.MathUtils.Vector2D;

/**
 *
 * the core of the robot, including all the modules that powers the module
 * note that services are not included in this field
 * */
// TODO: we have to sort the shuffleboard, it's too messy now
public class RobotCore {
        RobotConfigReader robotConfig;
        public final SwerveWheel frontLeftWheel, backLeftWheel, frontRightWheel, backRightWheel;
        public final SimpleGyro gyro;
        public final SwerveWheelPositionEstimator positionReader;
        public final SwerveBasedChassis chassisModule;
        private final List<String> configsToTune = new ArrayList<>(1);
        private final List<RobotModuleBase> modules;
        private List<RobotServiceBase> services;
        private List<Thread> modulesUpdatingThreads;
        private final boolean useMultiThreads;
        protected boolean wasEnabled;

        public RobotCore() {
                this(true);
        }
        /**
         * creates a robot core
         * creates the instances of all the modules, but do not call init functions yet
         * @param useMultiThreads whether the robot modules should be updated in independent threads
         * */
        public RobotCore(boolean useMultiThreads) {
                System.out.println("<-- Robot Core | creating robot... -->");
                this.useMultiThreads = useMultiThreads;
                modules = new ArrayList<>();
                services = new ArrayList<>();

                try {
                        robotConfig = new RobotConfigReader("fasterChassis");
                } catch (Exception e) {
                        e.printStackTrace();
                        throw new RuntimeException("error while reading robot config");
                }

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
                        service.periodic();

                if (!useMultiThreads)
                        for (RobotModuleBase module:modules)
                                module.periodic();

//                SmartDashboard.putNumber("robot x", positionReader.getRobotPosition2D().getValue()[0]);
//                SmartDashboard.putNumber("robot y", positionReader.getRobotPosition2D().getValue()[1]);
//                SmartDashboard.putNumber("velocity x", positionReader.getRobotVelocity2D().getValue()[0]);
//                SmartDashboard.putNumber("velocity y", positionReader.getRobotVelocity2D().getValue()[1]);

                robotConfig.updateTuningConfigsFromDashboard();

                /* monitor the program's performance */
                SmartDashboard.putNumber("program delay (ms)", System.currentTimeMillis() - t);
                t = System.currentTimeMillis();
        }
}