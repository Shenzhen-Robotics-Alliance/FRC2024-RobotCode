package frc.robot.Modules.Chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.IMUs.SimpleGyro;
import frc.robot.Modules.Chassis.SwerveWheel;
import frc.robot.Modules.PositionReader.PositionEstimator;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.*;
import frc.robot.Utils.MathUtils.AngleUtils;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.MechanismControllers.EnhancedPIDController;

/**
 * the module that controls the chassis with its four swerves
 */
public class SwerveBasedChassis extends RobotModuleBase {
    private final boolean useProfiledSpeedControl = true;
    /** the amount of changes updatable for the old task before initiating a new task */
    private double translationalTaskUpdatableRange;
    private double robotMaximumSpeed;
    private double timeNeededToFullyAccelerate;
    private double robotSpeedActivateSpeedControl;
    private double wheelsPowerConstrain;
    private double rotationalSpeedMaxSacrifice;
    /** the pid controller that controls the rotation of the robot when needed */
    private EnhancedPIDController goToRotationController;
    private ChassisPositionController chassisPIDController;

    private double positionDifferenceAsTaskFinished;



    /** the current translational task */
    private ChassisTaskTranslation translationalTask;
    /** the current rotational task  */
    private ChassisTaskRotation rotationalTask;
    private boolean locked;

    public enum OrientationMode {
        FIELD,
        ROBOT
    }

    /**
     * the speed controlling mode of each of the independent wheels
     * this is NOT the control mode of the whole chassis
     */
    public enum WheelOutputMode {
        PERCENT_POWER,
        SPEED_CONTROL
    }

    private OrientationMode orientationMode;


    /** the four wheels of the robot */
    private final SwerveWheel[] swerveWheels;
    private final PositionEstimator positionEstimator;
    private final SimpleGyro gyro;
    private final RobotConfigReader robotConfig;
    public SwerveBasedChassis(SwerveWheel[] swerveWheels, SimpleGyro gyro, RobotConfigReader robotConfig, PositionEstimator positionEstimator) {
        super("SwerveBasedChassis", (int)robotConfig.getConfig("system", "chassisModuleUpdateFrequency"));
        this.swerveWheels = swerveWheels;
        this.positionEstimator = positionEstimator;
        this.gyro = gyro;
        this.robotConfig = robotConfig;
    }

    @Override
    public void init() {
        this.reset();
    }

    @Override
    protected void periodic(double dt) {
        Vector2D processedTranslationalSpeed = processTranslationalMotion(dt);
        // SmartDashboard.putNumber("decided vel(x)", processedTranslationalSpeed.getValue()[0]);
        double rotationalSpeed = processRotationalMotion(dt);


        SmartDashboard.putNumber("imu yaw:", gyro.getYaw());

        double highestWheelSpeed = driveWheels(processedTranslationalSpeed, rotationalSpeed);
        // System.out.println("highest wheel speed:" + highestWheelSpeed);
        if (highestWheelSpeed <= wheelsPowerConstrain) return;
        /* if a wheel is asked to run higher than max power, we need to slow everything down to avoid tearing the robot apart */

        /* first we slow down the rotational part */
        final double rotationMinScale = (1-rotationalSpeedMaxSacrifice);

        // System.out.println("sacrificing rotational part by scale: " + Math.sqrt(rotationMinScale));
        rotationalSpeed *= Math.sqrt(rotationMinScale);
        highestWheelSpeed = driveWheels(processedTranslationalSpeed, rotationalSpeed);
        if (highestWheelSpeed <= wheelsPowerConstrain) {
            return;
        }

        /* then we slow it down to max rotationalSpeedMaxSacrifice */
        // System.out.println("highest wheel speed:" + highestWheelSpeed);
        // System.out.println("sacrificing rotational part by scale: " + rotationMinScale);
        rotationalSpeed *= Math.sqrt(rotationMinScale);
        highestWheelSpeed = driveWheels(processedTranslationalSpeed, rotationalSpeed);
        if (highestWheelSpeed <= wheelsPowerConstrain) return;

        /* finally, we start scaling down the translational part */
        // System.out.println("highest wheel speed:" + highestWheelSpeed);
        // System.out.println("scaling down translational speed by factor:" + wheelsPowerConstrain/highestWheelSpeed);
        processedTranslationalSpeed = processedTranslationalSpeed.multiplyBy(wheelsPowerConstrain/highestWheelSpeed);
        rotationalSpeed *= wheelsPowerConstrain/highestWheelSpeed;
        driveWheels(processedTranslationalSpeed, rotationalSpeed);
    }

    /**
     * pass the robot motion params to each wheels
     * @return the highest drive speed among the four wheels
     * */
    private double driveWheels(Vector2D translationalSpeed, double rotationalSpeed) {
        double highestWheelSpeed = 0;
        for (SwerveWheel wheel : swerveWheels)
            highestWheelSpeed = Math.max(highestWheelSpeed,
                    wheel.drive(translationalSpeed, rotationalSpeed, this));
        return highestWheelSpeed;
    }

    @Override
    public void updateConfigs() {
        this.translationalTaskUpdatableRange = robotConfig.getConfig("chassis/translationalTaskUpdatableRange");
        this.robotMaximumSpeed = robotConfig.getConfig("chassis/robotMaximumSpeed");
        this.timeNeededToFullyAccelerate = robotConfig.getConfig("chassis/timeNeededToFullyAccelerate");
        this.robotSpeedActivateSpeedControl = robotConfig.getConfig("chassis/robotSpeedActivateSpeedControl");
        this.wheelsPowerConstrain = robotConfig.getConfig("chassis/wheelsPowerConstrain");
        this.rotationalSpeedMaxSacrifice = robotConfig.getConfig("chassis/rotationalSpeedMaxSacrifice");
        this.ignoredAccelerateTime = robotConfig.getConfig("chassis/ignoredAccelerateTime");

        double robotRotationalErrorTolerance = Math.toRadians(robotConfig.getConfig("chassis/robotRotationalErrorTolerance"));
        double robotRotationalErrorStartDecelerate = Math.toRadians(robotConfig.getConfig("chassis/robotRotationalErrorStartDecelerate"));
        double robotRotationMaximumCorrectionPower = robotConfig.getConfig("chassis/robotRotationMaximumCorrectionPower");
        double robotRotationMinimumCorrectionPower = robotConfig.getConfig("chassis/robotRotationMinimumCorrectionPower");
        double robotRotationFeedForwardTime = robotConfig.getConfig("chassis/robotRotationFeedForwardTime");

        double robotPositionErrorTolerance = robotConfig.getConfig("chassis/robotPositionErrorTolerance");
        double positionToleranceAsTaskFinished = robotConfig.getConfig("chassis/errorToleranceAsCommandFinished");
        this.positionDifferenceAsTaskFinished = robotPositionErrorTolerance * positionToleranceAsTaskFinished;

        double robotPositionErrorStartDecelerate = robotConfig.getConfig("chassis/robotPositionErrorStartDecelerate");
        double robotPositionMaximumCorrectionPower = robotConfig.getConfig("chassis/robotPositionMaximumCorrectionPower");
        double robotPositionMinimumCorrectionPower = robotConfig.getConfig("chassis/robotPositionMinimumCorrectionPower");
        double robotPositionFeedForwardTime = robotConfig.getConfig("chassis/robotPositionFeedForwardTime");

        this.goToRotationController = new EnhancedPIDController(new EnhancedPIDController.StaticPIDProfile(
                Math.PI * 2,
                robotRotationMaximumCorrectionPower,
                robotRotationMinimumCorrectionPower,
                robotRotationalErrorStartDecelerate,
                robotRotationalErrorTolerance,
                robotRotationFeedForwardTime,
                0,
                0
        ));

        final ChassisPositionController.ChassisPIDConfig chassisPIDConfig = new ChassisPositionController.ChassisPIDConfig(
                robotPositionMaximumCorrectionPower,
                robotPositionMinimumCorrectionPower,
                robotPositionErrorStartDecelerate,
                robotPositionErrorTolerance,
                robotPositionFeedForwardTime
        );

        chassisPIDController = new ChassisPositionController(chassisPIDConfig);
    }

    @Override
    public void resetModule() {
        /* reset and recover ownerships to the wheels */
        for (SwerveWheel swerveWheel: swerveWheels) {
            swerveWheel.reset();
            swerveWheel.gainOwnerShip(this);
        }
        /* reset the imu module */
        gyro.reset();
        /* reset the position calculator */
        positionEstimator.reset();
        this.translationalTask = new ChassisTaskTranslation(ChassisTaskTranslation.TaskType.SET_VELOCITY, new Vector2D());
        this.rotationalTask = new ChassisTaskRotation(ChassisTaskRotation.TaskType.SET_VELOCITY, 0);

        locked = false;
        this.translationalTask = new ChassisTaskTranslation(ChassisTaskTranslation.TaskType.SET_VELOCITY,new Vector2D());
        this.translationalTask.initiate(new Vector2D());

        this.decidedVelocity = new Vector2D();
    }

    private Vector2D processTranslationalMotion(double dt) {
        switch (translationalTask.taskType) {
            case SET_VELOCITY:
                return processTranslationalVelocityControl( // process the speed control after
                        processOrientation(this.translationalTask.translationValue), dt
                );
            case GO_TO_POSITION:
                return processOrientation(
                        processTranslationalPositionControl(this.translationalTask.translationValue)
                );
            default:
                throw new UnsupportedOperationException("unsupported translational task type:" + this.translationalTask.taskType.name());
        }
    }

    private Vector2D decidedVelocity;
    /** if the chassis can accelerate to the targeted velocity within this amount time, it just jumps to the target */
    private double ignoredAccelerateTime;
    /**
     * process the desired velocity into actual respond velocity using profiled speed-control
     * @param desiredVelocity the desired velocity, in relationship to the robot itself and in PERCENT OUT OF FULL SPEED
     * @return the amount
     * */
    private Vector2D processTranslationalVelocityControl(Vector2D desiredVelocity, double dt) {
        if (!useProfiledSpeedControl
                || Math.max(desiredVelocity.getMagnitude(), positionEstimator.getRobotVelocity2D().getMagnitude()/robotMaximumSpeed)
                < robotSpeedActivateSpeedControl) // if the desired or current velocity is smaller than the activation speed
            return desiredVelocity; // disable velocity control

        final double maxAcceleration = 1.0 / timeNeededToFullyAccelerate;
        Vector2D velocityDifference = desiredVelocity.addBy(
                decidedVelocity.multiplyBy(-1)
        );

        // SmartDashboard.putNumber("vel ctrl decided", decidedVelocity.getMagnitude());
        Vector2D step = new Vector2D(velocityDifference.getHeading(),
                Math.min(dt * maxAcceleration, velocityDifference.getMagnitude())
        );
        this.decidedVelocity = decidedVelocity.addBy(step);
        if (velocityDifference.getMagnitude() < maxAcceleration * ignoredAccelerateTime)
            return desiredVelocity;
        return decidedVelocity;
    }

    private Vector2D processOrientation(Vector2D desiredVelocity) {
        if (this.orientationMode == OrientationMode.ROBOT)
            return desiredVelocity;
        return desiredVelocity.multiplyBy(
                new Rotation2D(gyro.getYaw())
                        .getReversal());
    }

    /**
     * gets the correction power to move to a given position
     * @param desiredPosition the desired position, field-orientated
     * @return the amount of chassis speed the robot needs at the current time, field-orientated
     */
    public Vector2D processTranslationalPositionControl(Vector2D desiredPosition) {
        final Vector2D chassisPosition2D = positionEstimator.getRobotPosition2D(),
                chassisVelocity2D = positionEstimator.getRobotVelocity2D();

        chassisPIDController.setDesiredPosition(desiredPosition);
        return chassisPIDController.getCorrectionPower(chassisPosition2D, chassisVelocity2D);
    }

    private double processRotationalMotion(double dt) {
        switch (rotationalTask.taskType) {
            case SET_VELOCITY: {
                return processRotationalVelocity(this.rotationalTask.rotationalValue);
            }
            case FACE_DIRECTION: {
                return getRotationalCorrectionSpeed(this.rotationalTask.rotationalValue, dt);
            }
            default: {
                throw new UnsupportedOperationException("bad rotational task type:" + this.rotationalTask.taskType.name());
            }
        }
    }

    private double processRotationalVelocity(double desiredRotationalVelocity) {
        // TODO write the profiled rotational velocity controller here
        return desiredRotationalVelocity;
    }

    /**
     * get the amount of chassis speed needed in order to maintain the desired rotation
     *
     * @param desiredRotation
     * @return
     */
    private double getRotationalCorrectionSpeed(double desiredRotation, double dt) {
        goToRotationController.startNewTask(new EnhancedPIDController.Task(
                EnhancedPIDController.Task.TaskType.GO_TO_POSITION,
                desiredRotation
        ));
        return goToRotationController.getMotorPower(AngleUtils.simplifyAngle(gyro.getYaw()), gyro.getYawVelocity(), dt);
    }

    @Override
    public void onDestroy() {

    }

    @Override
    protected void onEnable() {

    }

    @Override
    protected void onDisable() {

    }

    /**
     * set the translational task of the chassis
     * @param translationalTask the desired task for rotation
     * @param operator the module or service that is calling for the task
     * */
    public void setTranslationalTask(ChassisTaskTranslation translationalTask, RobotModuleOperatorMarker operator) {
        if (!this.isOwner(operator))
                return;

        this.translationalTask = translationalTask;
        this.translationalTask.initiate(
                translationalTask.taskType == ChassisTaskTranslation.TaskType.SET_VELOCITY ?
                        this.positionEstimator.getRobotVelocity2D().multiplyBy(1 / robotMaximumSpeed) : this.positionEstimator.getRobotPosition2D()
        );
    }

    public void setWheelOutputMode(WheelOutputMode selectedMode, RobotModuleOperatorMarker operator) {
        if (!this.isOwner(operator))
            return;
        for(SwerveWheel wheel: swerveWheels)
            wheel.setSpeedControl(
                    selectedMode == WheelOutputMode.SPEED_CONTROL // asked to use speed control
                    && isSpeedControlAllowed()
            );
    }

    private boolean isSpeedControlAllowed() {
        boolean disallowed = this.translationalTask.taskType == ChassisTaskTranslation.TaskType.SET_VELOCITY
                && this.translationalTask.translationValue.getMagnitude() > 0.95; // TODO read from robot config, the amount of power at which the robot should just forget about speed control
        return true;
    }

    public void setOrientationMode(OrientationMode mode, RobotModuleOperatorMarker operator) {
        if (!this.isOwner(operator))
            return;
        this.orientationMode = mode;
    }

    /**
     * sets the rotational task of the chassis
     * @param rotationalTask the desired task for rotation
     * @param operator the module or service that is calling for the task
     */
    public void setRotationalTask(ChassisTaskRotation rotationalTask, RobotModuleOperatorMarker operator) {
        if (!this.isOwner(operator))
            return;
        this.rotationalTask = rotationalTask;
    }

    public void setChassisLocked(boolean locked, RobotModuleOperatorMarker operator) {
        if (!this.isOwner(operator))
            return;
        for (SwerveWheel swerveWheel:this.swerveWheels)
            swerveWheel.setWheelLocked(locked, this);
    }

    public double getChassisHeading() {
        return gyro.getYaw();
    }

    public boolean isCurrentTranslationalTaskFinished() {
        switch (translationalTask.taskType) {
            case SET_VELOCITY:
                return translationalTask.translationValue.getMagnitude() == 0;
            case GO_TO_POSITION:
                return Vector2D.displacementToTarget(positionEstimator.getRobotPosition2D(), translationalTask.translationValue).getMagnitude() < positionDifferenceAsTaskFinished;
            default:
                throw new UnsupportedOperationException("unsupported translational task type:" + this.translationalTask.taskType.name());
        }
    }

    public static class ChassisTaskTranslation {
        public enum TaskType {
            SET_VELOCITY,
            GO_TO_POSITION
        }
        public final TaskType taskType;
        public final Vector2D translationValue;
        public Vector2D initialState;
        public final Timer taskTimer;

        public ChassisTaskTranslation(TaskType type, Vector2D value) {
            this.taskType = type;
            this.translationValue = value;
            this.taskTimer = new Timer();
        }

        /**
         * initiates the translation with initial state
         * @param initialState note it is in PERCENT OUT OF FULL-SPEED, NOT meters/second
         */
        public void initiate(Vector2D initialState) {
            this.taskTimer.start();
            this.initialState = initialState;
        }
    }

    public static class ChassisTaskRotation {
        public enum TaskType {
            SET_VELOCITY,
            FACE_DIRECTION
        }
        public final TaskType taskType;
        public double rotationalValue;
        public final Timer taskTimer;

        /** creates a rotational task
         * @param type choose from this.TaskType
         * @param value in radians, counter-clockwise is positive
         */
        public ChassisTaskRotation(TaskType type, double value) {
            this.taskType = type;
            this.rotationalValue = value;
            this.taskTimer = new Timer();
        }

        public void initiate() {
            taskTimer.start();
        }
    }
}