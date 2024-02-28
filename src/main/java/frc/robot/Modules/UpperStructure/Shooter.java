package frc.robot.Modules.UpperStructure;

import frc.robot.Drivers.Motors.Motor;
import frc.robot.Modules.PositionReader.PositionEstimator;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.ComputerVisionUtils.AprilTagReferredTarget;
import frc.robot.Utils.EasyShuffleBoard;
import frc.robot.Utils.MathUtils.LookUpTable;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.StatisticsUtils;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.MechanismControllers.EncoderMotorMechanism;
import frc.robot.Utils.MechanismControllers.FlyWheelSpeedController;
import frc.robot.Utils.RobotConfigReader;
import frc.robot.Utils.RobotModuleOperatorMarker;
import frc.robot.Utils.TimeUtils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeoutException;

public class Shooter extends RobotModuleBase {
    public enum ShooterMode {
        DISABLED,
        SHOOT,
        AMPLIFY,
        SPECIFIED_RPM
    }

    private final EncoderMotorMechanism[] shooters;
    private final FlyWheelSpeedController flyWheelSpeedController;
    public final AimingSystem aimingSystem;
    private final double encoderVelocityToRPM;
    private final double shooterReadyErrorBound;
    private final RobotConfigReader robotConfig;
    private double specifiedRPM;
    private double aimingSystemDecidedRPM;
    private ShooterMode currentMode;
    public Shooter(EncoderMotorMechanism[] shooters, RobotConfigReader robotConfig) {
        this(shooters, null, robotConfig);
    }
    public Shooter(EncoderMotorMechanism[] shooters, AimingSystem aimingSystem, RobotConfigReader robotConfig) {
        super("Shooter");
        this.shooters = shooters;
        this.aimingSystem = aimingSystem;
        this.robotConfig = robotConfig;
        super.motors.addAll(Arrays.asList(shooters));
        flyWheelSpeedController = new FlyWheelSpeedController(
                new FlyWheelSpeedController.FlyWheelSpeedControllerProfile(0,0,0,0,0,0));
        for (EncoderMotorMechanism shooter:shooters)
            shooter.setController(flyWheelSpeedController);

        this.encoderVelocityToRPM = 60.0 / robotConfig.getConfig("shooter", "shooterMotorEncoderTicksPerRevolution");
        this.shooterReadyErrorBound = robotConfig.getConfig("shooter", "flyWheelSpeedErrorTolerance") * robotConfig.getConfig("shooter", "speedControllerMaximumSpeed");
    }

    /**
     * set the desired speed of the shooter, 0 for stop
     * this is for test only
     * @param desiredSpeedRPM the desired speed of the shooter, in rpm
     * */
    @Deprecated
    public void setDesiredSpeed(double desiredSpeedRPM, RobotModuleOperatorMarker operator) {
        if (!isOwner(operator)) return;

        this.specifiedRPM = desiredSpeedRPM;
    }

    public void setShooterMode(ShooterMode desiredMode, RobotModuleOperatorMarker operator) {
        if (!isOwner(operator)) return;

        this.currentMode = desiredMode;
    }

    public boolean shooterReady() {
        if (this.currentMode != ShooterMode.SHOOT)
            return false;
        final double desiredShootingSpeed = getShooterSpeedWithAimingSystem(0);
        double maxErrorRPM = 0;
        for (EncoderMotorMechanism shooter:shooters)
            maxErrorRPM = Math.max(Math.abs(shooter.getEncoderVelocity() * encoderVelocityToRPM - desiredShootingSpeed), maxErrorRPM);
        System.out.println("shooter speed error (rpm): " + maxErrorRPM + ", tolerance: " + shooterReadyErrorBound);
        return maxErrorRPM < shooterReadyErrorBound;
    }

    @Override
    public void init() {
        for (EncoderMotorMechanism shooter:shooters) {
            shooter.setMotorZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE, this);
            shooter.gainOwnerShip(this);
        }
        this.onReset();
        updateConfigs();
    }


    @Override
    protected void periodic(double dt) {
        aimingSystemDecidedRPM = getShooterSpeedWithAimingSystem(additionalInAdvanceTime);
        final double desiredEncoderVelocity = decideRPM() / encoderVelocityToRPM;

        flyWheelSpeedController.setDesiredSpeed(desiredEncoderVelocity);

        EasyShuffleBoard.putNumber("shooter", "Shooter Desired RPM", desiredEncoderVelocity * encoderVelocityToRPM);
        EasyShuffleBoard.putNumber("shooter", "flyWheel 0 actual RPM", shooters[0].getEncoderVelocity() * encoderVelocityToRPM);

        for (EncoderMotorMechanism shooter : shooters)
            shooter.updateWithController(this);
    }

//    @Override
//    protected void periodic(double dt) {
//        aimingSystemDecidedRPM = getShooterSpeedWithAimingSystem();
//
//        final double desiredEncoderVelocity = decideRPM() / encoderVelocityToRPM;
//
//        flyWheelSpeedController.setDesiredSpeed(desiredEncoderVelocity);
//        for (int shooterID = 0; shooterID < shooters.length; shooterID++)
//            EasyShuffleBoard.putNumber("shooter", "motor " + shooterID + " actual speed", shooters[shooterID].getEncoderVelocity() * encoderVelocityToRPM);
//
//        EasyShuffleBoard.putNumber("shooter", "Shooter Desired RPM", decideRPM());
//
//        for (EncoderMotorMechanism shooter : shooters)
//            shooter.updateWithController(this);
//    }

    private double decideRPM() {
        // System.out.println("shooter current mode: " + currentMode.name());
        return switch (currentMode) {
            case SHOOT -> aimingSystemDecidedRPM;
            case AMPLIFY -> amplifyingRPM;
            case SPECIFIED_RPM -> specifiedRPM;
            default -> idleRPM; // idle
        };
    }

    @Override
    public void onReset() {
        setDesiredSpeed(0, null);
        updateConfigs();
        for (EncoderMotorMechanism shooter : shooters)
            shooter.setMotorZeroPowerBehavior(Motor.ZeroPowerBehavior.RELAX, this);

        currentMode = ShooterMode.DISABLED;
    }

    @Override
    public void onDestroy() {

    }

    @Override
    protected void onEnable() {

    }

    @Override
    protected void onDisable() {
        for (EncoderMotorMechanism shooter:shooters)
            shooter.disableMotor(this);
    }

    private double defaultShootingRPM, amplifyingRPM, idleRPM, projectileSpeed, shootingRange, additionalInAdvanceTime;
    private LookUpTable shooterRPMToTargetDistanceLookUpTable;

    /** the desired arm position for aiming, in degrees and in reference to the default shooting position of the arm, which is specified in the arm configs */
    private LookUpTable armPositionDegreesToTargetDistanceLookUpTable;
    @Override
    public void updateConfigs() {
        final List<Double> speakerTargetDistances = new ArrayList<>(), shooterRPMs = new ArrayList<>(), armAngles = new ArrayList<>();
        int i = 0; while (i < 100) {
            try {
                speakerTargetDistances.add(robotConfig.getConfig("shooter", "targetDistance" + i));
                shooterRPMs.add(robotConfig.getConfig("shooter", "shooterRPM"+ i));
                armAngles.add(robotConfig.getConfig("shooter", "armAngle"+ i));
                i++;
            } catch (NullPointerException end) { break; }
        }
        this.shooterRPMToTargetDistanceLookUpTable = new LookUpTable(StatisticsUtils.toArray(speakerTargetDistances), StatisticsUtils.toArray(shooterRPMs));
        this.armPositionDegreesToTargetDistanceLookUpTable = new LookUpTable(StatisticsUtils.toArray(speakerTargetDistances), StatisticsUtils.toArray(armAngles));

        this.defaultShootingRPM = robotConfig.getConfig("shooter", "defaultShootingRPM");
        this.amplifyingRPM = robotConfig.getConfig("shooter", "amplifyRPM");
        this.idleRPM = robotConfig.getConfig("shooter", "idleRPM");
        this.projectileSpeed = robotConfig.getConfig("shooter", "projectileSpeed");
        this.shootingRange = robotConfig.getConfig("shooter", "shootingRange");
        this.additionalInAdvanceTime = robotConfig.getConfig("shooter", "additionalInAdvanceTime");

        final FlyWheelSpeedController.FlyWheelSpeedControllerProfile speedControllerProfile = new FlyWheelSpeedController.FlyWheelSpeedControllerProfile(
                robotConfig.getConfig("shooter", "speedControllerProportionGain"),
                robotConfig.getConfig("shooter", "speedControllerFeedForwardGain"),
                robotConfig.getConfig("shooter", "speedControllerFrictionGain"),
                robotConfig.getConfig("shooter", "speedControllerFeedForwardDelay"),
                robotConfig.getConfig("shooter", "speedControllerMaximumSpeed") / this.encoderVelocityToRPM,
                robotConfig.getConfig("shooter", "speedControllerTimeNeededToAccelerateToMaxSpeed")
        );
        flyWheelSpeedController.setProfile(speedControllerProfile);
    }

    public double getAdditionalInAdvanceTime() {
        return additionalInAdvanceTime;
    }

    /**
     * gets the aiming angle from the aiming system, so that the arm knows where to go
     * @return the arm position to shoot, in radian and in reference to default shooting position specified in arm configs, positive direction is towards ground.
     * 0, which is the default position, will be returned if target unseen
     * */
    public double getArmPositionWithAimingSystem(double additionalInAdvanceTime) {
        final Vector2D targetRelativePositionToRobot;
        if (aimingSystem == null
                || (targetRelativePositionToRobot = aimingSystem.getRelativePositionToTarget(getProjectileSpeed(), additionalInAdvanceTime)) == null)
            return 0;
        final double distanceToTarget = targetRelativePositionToRobot.getMagnitude();
        return Math.toRadians(armPositionDegreesToTargetDistanceLookUpTable.getYPrediction(distanceToTarget));
    }

    public boolean targetInRange() {
        final Vector2D targetRelativePositionToRobot;
        if (aimingSystem == null
                || (targetRelativePositionToRobot = aimingSystem.getRelativePositionToTarget(getProjectileSpeed())) == null)
            return false;
        return targetRelativePositionToRobot.getMagnitude() < shootingRange;
    }

    public double getShooterSpeedWithAimingSystem(double additionalInAdvanceTime) {
        final Vector2D targetRelativePositionToRobot;
        if (aimingSystem == null
                || (targetRelativePositionToRobot = aimingSystem.getRelativePositionToTarget(getProjectileSpeed())) == null)
            return defaultShootingRPM;
        final double distanceToTarget = targetRelativePositionToRobot.getMagnitude();

        // System.out.println("shooter corresponding (RPM): " + shooterRPMToTargetDistanceLookUpTable.getYPrediction(distanceToTarget));
        EasyShuffleBoard.putNumber("shooter", "target distance", distanceToTarget);
        EasyShuffleBoard.putNumber("shooter", "shooter corresponding RPM",  shooterRPMToTargetDistanceLookUpTable.getYPrediction(distanceToTarget));
        return shooterRPMToTargetDistanceLookUpTable.getYPrediction(distanceToTarget);
    }

    public double getProjectileSpeed() {
        return projectileSpeed;
    }

    public static final class AimingSystem {
        public final PositionEstimator chassisPositionEstimator;
        public final AprilTagReferredTarget target;
        public final long timeUnseenTolerance;
        public Vector2D defaultTargetFieldPosition = null;

        /**
         * @param timeUnseenTolerance the amount of time, in ms, that we can accept the target to be gone
         * */
        public AimingSystem(PositionEstimator chassisPositionEstimator, AprilTagReferredTarget aprilTagReferredTarget, long timeUnseenTolerance) {
            this.chassisPositionEstimator = chassisPositionEstimator;
            this.target = aprilTagReferredTarget;
            this.timeUnseenTolerance = timeUnseenTolerance;
        }

        public Vector2D getRelativePositionToTarget(double projectileSpeed) {
            return getRelativePositionToTarget(projectileSpeed, 0);
        }
        /**
         * with the target position tracker and the position estimator, calculate the target's relative position to the robot
         * @param projectileSpeed the speed of the projectile, in m/s, this is to reduce the deviation due to flight time
         * @return the relative position to target, in meters, and in reference to the robot; if not seen for too long, return null
          */
        public Vector2D getRelativePositionToTarget(double projectileSpeed, double additionalInAdvanceTime) {
            final Vector2D targetFieldPositionByCamera = target.getTargetFieldPositionWithAprilTags(timeUnseenTolerance),
                    targetFieldPosition = targetFieldPositionByCamera == null ? defaultTargetFieldPosition : targetFieldPositionByCamera;
            if (targetFieldPosition == null) return null;
            return getRelativePositionToTarget(projectileSpeed, targetFieldPosition, additionalInAdvanceTime);
        }

        public Vector2D getRelativePositionToTarget(double projectileSpeed, Vector2D targetFieldPosition) {
            return getRelativePositionToTarget(projectileSpeed, targetFieldPosition, 0);
        }

        public Vector2D getRelativePositionToTarget(double projectileSpeed, Vector2D targetFieldPosition, double additionalInAdvanceTime) {
            final double distanceToTarget = Vector2D.displacementToTarget(chassisPositionEstimator.getRobotPosition2D(), targetFieldPosition).getMagnitude(),
                    flightTime = distanceToTarget / projectileSpeed;
            final Vector2D chassisPositionAfterFlightTime = chassisPositionEstimator.getRobotPosition2D().addBy(chassisPositionEstimator.getRobotVelocity2D().multiplyBy(flightTime + additionalInAdvanceTime));
            return Vector2D.displacementToTarget(chassisPositionAfterFlightTime, targetFieldPosition);
        }

        public double getRobotFacing(double projectileSpeed) {
            final Vector2D targetFieldPositionByCamera = target.getTargetFieldPositionWithAprilTags(timeUnseenTolerance),
                    targetFieldPosition = targetFieldPositionByCamera == null ? defaultTargetFieldPosition : targetFieldPositionByCamera;
            return getRelativePositionToTarget(projectileSpeed, targetFieldPosition).getHeading() - Math.toRadians(90);
        }

        public double getRobotFacing(double projectileSpeed, Vector2D targetLastSeenPosition) {
            return getRelativePositionToTarget(projectileSpeed, targetLastSeenPosition, 0).getHeading() - Math.toRadians(90);
        }
    }
}
