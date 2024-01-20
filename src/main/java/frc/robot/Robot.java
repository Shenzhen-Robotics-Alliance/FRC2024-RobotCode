package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.*;
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

public class Robot {
        RobotConfigReader robotConfig;
        protected final SwerveWheel frontLeftWheel, backLeftWheel, frontRightWheel, backRightWheel;
        protected final SimpleGyro gyro;
        public final SwerveWheelPositionEstimator positionReader;
        public final SwerveBasedChassis chassisModule;
        protected final List<String> configsToTune = new ArrayList<>(1);
        protected final Timer programDelay = new Timer();
        protected final List<RobotModuleBase> modules;
        protected final List<RobotServiceBase> services;
        protected List<Thread> modulesUpdatingThreads;
        protected final boolean useMultiThreads;
        protected boolean wasEnabled;

        public Robot() {
                this(true);
        }
        public Robot(boolean useMultiThreads) {
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
                // this.gyro = new SimpleGyro(0, true, new NavX2IMU());

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

        public void initializeRobot() {
                System.out.println("<-- Robot | initializing robot... -->");
                /* initialize the modules and services */
                for (RobotModuleBase module:modules)
                        module.init();
                for (RobotServiceBase service:services)
                        service.init();

                /* start the config tuning */
                addConfigsToTune();
                for (String config:configsToTune)
                        robotConfig.startTuningConfig(config);
                restRobot();
                wasEnabled = false; // when the robot just booted, it was not enabled
                System.out.println("<-- Robot | robot initialized -->");
        }



        public void restRobot() {
                for (RobotModuleBase module: modules)
                        module.reset();
                for (RobotServiceBase service: services)
                        service.reset();
                programDelay.start();
                programDelay.reset();
        }

        /** start or re-start the robot */
        public void enableRobot() {
                System.out.println("<-- Robot | enabling robot... -->");
                for (RobotModuleBase module: modules)
                        module.enable();
                if (useMultiThreads) {
                        scheduleThreads();
                        runThreads();
                }

                restRobot();

                wasEnabled = true;
                System.out.println("<-- Robot | robot enabled -->");
        }

        public void pauseRobot() {
                System.out.println("<-- Robot | pausing robot... -->");
                this.wasEnabled = false;
                for (RobotModuleBase module: modules)
                        module.disable();
                System.out.println("<-- Robot | robot paused... -->");
        }

        private void scheduleThreads() {
                modulesUpdatingThreads = new ArrayList<>();
                for (RobotModuleBase module: modules)
                        modulesUpdatingThreads.add(module.getPeriodicUpdateThread());
        }
        private void runThreads() {
                for (Thread thread: modulesUpdatingThreads)
                        thread.start();
        }

        /**
         * always called periodically, no mater if the robot is enabled or not
         * */
        public void updateRobot(boolean isEnabled) {
                if (!isEnabled && wasEnabled)
                        pauseRobot();
                else if (isEnabled && !wasEnabled)
                        enableRobot();

                if (isEnabled)
                        for (RobotServiceBase service: services)
                                service.periodic();

                if (isEnabled && !useMultiThreads)
                        for (RobotModuleBase module:modules)
                                module.periodic();

//                SmartDashboard.putNumber("robot x", positionReader.getRobotPosition2D().getValue()[0]);
//                SmartDashboard.putNumber("robot y", positionReader.getRobotPosition2D().getValue()[1]);
//                SmartDashboard.putNumber("velocity x", positionReader.getRobotVelocity2D().getValue()[0]);
//                SmartDashboard.putNumber("velocity y", positionReader.getRobotVelocity2D().getValue()[1]);

                robotConfig.updateTuningConfigsFromDashboard();

                SmartDashboard.putNumber("program delay(ms)", (programDelay.get() * 1000));
                programDelay.reset();
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
}