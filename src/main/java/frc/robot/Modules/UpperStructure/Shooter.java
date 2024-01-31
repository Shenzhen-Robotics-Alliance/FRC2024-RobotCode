package frc.robot.Modules.UpperStructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Drivers.Visions.TargetFieldPositionTracker;
import frc.robot.Modules.PositionReader.PositionEstimator;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.MathUtils.LookUpTable;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.MechanismControllers.EncoderMotorMechanism;
import frc.robot.Utils.MechanismControllers.FlyWheelSpeedController;
import frc.robot.Utils.RobotConfigReader;
import frc.robot.Utils.RobotModuleOperatorMarker;

import java.util.Arrays;
import java.util.Map;

public class Shooter extends RobotModuleBase {
    private static final boolean inTestMode = false;
    public enum ShooterMode {
        DISABLED,
        SHOOT,
        AMPLIFY,
        SPECIFIED_RPM
    }

    private final EncoderMotorMechanism[] shooters;
    private final FlyWheelSpeedController[] speedControllers;
    private final AimingSystem aimingSystem;
    private final double encoderVelocityToRPM;
    private final double shooterReadyErrorBound;
    private final RobotConfigReader robotConfig;
    private double specifiedRPM;
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
        speedControllers = new FlyWheelSpeedController[shooters.length];
        for (int shooterID = 0; shooterID < shooters.length; shooterID++) {
            speedControllers[shooterID] = new FlyWheelSpeedController(
                    new FlyWheelSpeedController.FlyWheelSpeedControllerProfile(0,0,0,0,0,0)
            );
            shooters[shooterID].setController(speedControllers[shooterID]);
        }
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
        final double decidedRPM = decideRPM();
        if (decidedRPM == 0) return false;
        double maxErrorRPM = 0;
        for (EncoderMotorMechanism shooter:shooters)
            maxErrorRPM = Math.max(Math.abs(shooter.getEncoderVelocity() * encoderVelocityToRPM - decidedRPM), maxErrorRPM);
        System.out.println("shooter speed error (rpm): " + maxErrorRPM);
        System.out.println("shooter speed error tolerance (rpm): " + shooterReadyErrorBound);
        return maxErrorRPM < shooterReadyErrorBound;
    }

    @Override
    public void init() {
        for (EncoderMotorMechanism shooter:shooters) {
            shooter.setMotorZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE, this);
            shooter.gainOwnerShip(this);
        }
        this.resetModule();
        updateConfigs();
    }

    @Override
    protected void periodic(double dt) {
        for (EncoderMotorMechanism shooter : shooters)
            shooter.updateWithController(this);

        final double desiredEncoderVelocity = decideRPM() / encoderVelocityToRPM;
        for (int shooterID = 0; shooterID < shooters.length; shooterID++) {
            speedControllers[shooterID].setDesiredSpeed(desiredEncoderVelocity);
            SmartDashboard.putNumber("Shooter " + shooterID + " actual speed", shooters[shooterID].getEncoderVelocity() * encoderVelocityToRPM);
        }

        SmartDashboard.putNumber("Shooter Desired RPM", specifiedRPM);
    }

    private double decideRPM() {
        return switch (currentMode) {
            case SHOOT -> getShooterSpeedWithAimingSystem();
            case AMPLIFY -> amplifyingRPM;
            case SPECIFIED_RPM -> specifiedRPM;
            default -> idleRPM;
        };
    }

    @Override
    public void resetModule() {
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

    /* TODO the following constants, move then to robotConfig and tune them */
    private static final double defaultShootingRPM = 4500;
    private static final double amplifyingRPM = 3000;
    private static final double idleRPM = 1000;
    private static final double projectileSpeed = 6;
    private static final LookUpTable shooterRPMToTargetDistanceLookUpTable = new LookUpTable(new double[] {1.5, 2, 2.5, 3, 3.5, 4, 5, 6}, new double[] {3800, 3900, 4200, 4500, 4800, 5100, 5500, 6000});
    /** the desired arm position for aiming, in degrees and in reference to the default shooting position of the arm, which is specified in the arm configs */
    private static final LookUpTable armPositionDegreesToTargetDistanceLookUpTable = new LookUpTable(new double[] {1.5, 2, 2.5, 3, 3.5, 4, 5, 6}, new double[] {-10, -5, -5, -5, 0, 0, 0, 0});
    @Override
    public void updateConfigs() {
        final FlyWheelSpeedController.FlyWheelSpeedControllerProfile speedControllerProfile = new FlyWheelSpeedController.FlyWheelSpeedControllerProfile(
                robotConfig.getConfig("shooter", "speedControllerProportionGain"),
                robotConfig.getConfig("shooter", "speedControllerFeedForwardGain"),
                robotConfig.getConfig("shooter", "speedControllerFrictionGain"),
                robotConfig.getConfig("shooter", "speedControllerFeedForwardDelay"),
                robotConfig.getConfig("shooter", "speedControllerMaximumSpeed") / this.encoderVelocityToRPM,
                robotConfig.getConfig("shooter", "speedControllerTimeNeededToAccelerateToMaxSpeed")
        );
        for (FlyWheelSpeedController speedController:speedControllers)
            speedController.setProfile(speedControllerProfile);
    }

    /**
     * gets the aiming angle from the aiming system, so that the arm knows where to go
     * @return the arm position to shoot, in radian and in reference to default shooting position specified in arm configs, positive direction is towards ground.
     * 0, which is the default position, will be returned if target unseen
     * */
    public double getArmPositionWithAimingSystem() {
        final Vector2D targetRelativePositionToRobot;
        if (aimingSystem == null
                || (targetRelativePositionToRobot = aimingSystem.getRelativePositionToTarget(projectileSpeed)) == null)
            return 0;
        final double distanceToTarget = targetRelativePositionToRobot.getMagnitude();
        return Math.toDegrees(armPositionDegreesToTargetDistanceLookUpTable.getYPrediction(distanceToTarget));
    }

    private double getShooterSpeedWithAimingSystem() {
        final Vector2D targetRelativePositionToRobot;
        if (aimingSystem == null
                || (targetRelativePositionToRobot = aimingSystem.getRelativePositionToTarget(projectileSpeed)) == null)
            return defaultShootingRPM;
        final double distanceToTarget = targetRelativePositionToRobot.getMagnitude();
        return shooterRPMToTargetDistanceLookUpTable.getYPrediction(distanceToTarget);
    }

    public static final class AimingSystem {
        public final PositionEstimator chassisPositionEstimator;
        public final TargetFieldPositionTracker targetTracker;
        public final Map<Integer, Vector2D> referenceTagsPositionToTarget;
        public final long timeUnseenTolerance;

        /**
         * @param referenceTagsPositionToTarget the positions of the tags that can be used as references to the target, and (field) their positions in refer to the target
         * @param timeUnseenTolerance the amount of time, in ms, that we can accept the target to be gone
         * */
        public AimingSystem(PositionEstimator chassisPositionEstimator, TargetFieldPositionTracker targetTracker, Map<Integer, Vector2D> referenceTagsPositionToTarget, long timeUnseenTolerance) {
            this.chassisPositionEstimator = chassisPositionEstimator;
            this.targetTracker = targetTracker;
            this.referenceTagsPositionToTarget = referenceTagsPositionToTarget;
            this.timeUnseenTolerance = timeUnseenTolerance;
        }

        /**
         * calculate the target's relative position to the robot, ignore flight time
         * @return the relative position to target, in meters, and in reference to the robot; if not seen for too long, return null
         * */
        public Vector2D getRelativePositionToTarget() {
            return getRelativePositionToTarget(Double.POSITIVE_INFINITY);
        }
        /**
         * with the target position tracker and the position estimator, calculate the target's relative position to the robot
         * @param projectileSpeed the speed of the projectile, in m/s, this is to reduce the deviation due to flight time
         * @return the relative position to target, in meters, and in reference to the robot; if not seen for too long, return null
          */
        public Vector2D getRelativePositionToTarget(double projectileSpeed) {
            int visibleTargetCount = 0;
            Vector2D targetFieldPosition = new Vector2D();
            for (int id:referenceTagsPositionToTarget.keySet()) {
                TargetFieldPositionTracker.TargetOnField target = targetTracker.getTargetByID(id);
                if (target == null || target.timeMillisSinceLastContact() > timeUnseenTolerance)
                    continue;

                /* the target's field position is equal the current reference's position minus the reference's relative position to target */
                final Vector2D targetFieldPositionEstimatedByCurrentReference = target.fieldPosition.addBy(referenceTagsPositionToTarget.get(id).multiplyBy(-1));
                targetFieldPosition = targetFieldPosition.addBy(targetFieldPositionEstimatedByCurrentReference);
                visibleTargetCount++;
            }
            if (visibleTargetCount == 0) return null;

            targetFieldPosition = targetFieldPosition.multiplyBy(1.0/visibleTargetCount);
            final double distanceToTarget = Vector2D.displacementToTarget(chassisPositionEstimator.getRobotPosition2D(), targetFieldPosition).getMagnitude(),
                    flightTime = distanceToTarget / projectileSpeed;
            final Vector2D chassisPositionAfterFlightTime = chassisPositionEstimator.getRobotPosition2D().addBy(chassisPositionEstimator.getRobotVelocity2D().multiplyBy(flightTime)),
                    targetPositionToRobot = Vector2D.displacementToTarget(chassisPositionAfterFlightTime, targetFieldPosition);
            return targetPositionToRobot.multiplyBy(
                    new Rotation2D(chassisPositionEstimator.getRobotRotation()).getReversal()
            );
        }
    }
}
