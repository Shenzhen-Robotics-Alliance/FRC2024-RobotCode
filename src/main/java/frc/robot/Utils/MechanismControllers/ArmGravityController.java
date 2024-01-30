package frc.robot.Utils.MechanismControllers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.EasyShuffleBoard;
import frc.robot.Utils.MathUtils.LookUpTable;

/**
 * a profiled arm controller that controls the arm
 * features:
 *  1. compensates for gravity according to a look-up table
 *  2. schedules a trapezoid movement to the targeted position
 *  3. when the targeted position of the schedule is reached, allows small adjustments of the targeted position, this will not be scheduled and will be executed right away
 * */
public class ArmGravityController implements MechanismController {
    /**
     *  the pid controller lib
     *  notice that it is only used for static control, we have taken over the trapezoid scheduling part
     *  */
    private final EnhancedPIDController enhancedPIDController;
    /**
     * the current schedule of the arm, this is used only to move the arm greatly, small adjustments will not be scheduled
     * */
    private EnhancedPIDController.TrapezoidPathSchedule currentSchedule;
    /**
     * the current desired position, updated if and if only great changes are applied to the desired position
     * */
    private double desiredPosition;
    /**
     * false when initialized,
     * true when the first task is specified
     * */
    private boolean alive;
    /** the profile of the arm, automatically obtained from robotConfig */
    public ArmProfile profile;
    /**
     * previousTimeMillis is to calculate dt
     * current schedule created time is the time when the last big movement is ordered
     * */
    private long previousTimeMillis, currentScheduleCreatedTime;
    public ArmGravityController(ArmProfile armProfile) {
        this.profile = armProfile;
        this.enhancedPIDController = new EnhancedPIDController(armProfile.staticPIDProfile);
    }

    public void updateDesiredPosition(double newDesiredPosition) {
        this.currentSchedule = new EnhancedPIDController.TrapezoidPathSchedule(profile.dynamicalPIDProfile, new EnhancedPIDController.Task(EnhancedPIDController.Task.TaskType.GO_TO_POSITION, newDesiredPosition), currentSchedule.getCurrentPathPosition(0), currentSchedule.getCurrentSpeed(0));
        this.alive = true;
        previousTimeMillis = System.currentTimeMillis();
    }

    public void goToDesiredPosition(double currentPosition, double currentVelocity, double newDesiredPosition) {
        if (newDesiredPosition == desiredPosition) return;
        this.currentSchedule = new EnhancedPIDController.TrapezoidPathSchedule(profile.dynamicalPIDProfile, new EnhancedPIDController.Task(EnhancedPIDController.Task.TaskType.GO_TO_POSITION, newDesiredPosition), currentPosition, currentVelocity);
        this.desiredPosition = newDesiredPosition;
        this.alive = true;
        currentScheduleCreatedTime = previousTimeMillis = System.currentTimeMillis();
    }

    @Override
    public double getMotorPower(double mechanismVelocity, double mechanismPosition) {
        if (!alive) return 0;
        final double scheduleTimer = (System.currentTimeMillis() - currentScheduleCreatedTime) / 1000.0,
                // currentDesiredPositionAccordingToSchedule = currentSchedule.getCurrentPathPosition(scheduleTimer),
                currentDesiredPositionAccordingToSchedule = desiredPosition, // TODO here we disabled the profiled PID control for test
                gravityCorrectionPower = profile.gravityTorqueEquilibriumMotorPowerLookUpTable.getYPrediction(mechanismPosition),
                dt = (System.currentTimeMillis() - previousTimeMillis) / 1000.0,
                pidCorrectionPower = enhancedPIDController.getMotorPowerGoToPositionClassic(mechanismPosition, mechanismVelocity,
                        new EnhancedPIDController.Task(EnhancedPIDController.Task.TaskType.GO_TO_POSITION, currentDesiredPositionAccordingToSchedule),
                        dt),
                overallCorrectionPower = gravityCorrectionPower + pidCorrectionPower;

        final double radianPerEncoderTick = Math.PI * 2 / 133.33 / 2048;
        EasyShuffleBoard.putNumber("arm", "mechanism actual position", Math.toDegrees(mechanismPosition * radianPerEncoderTick));
        EasyShuffleBoard.putNumber("arm", "mechanism actual velocity", Math.toDegrees(mechanismVelocity * radianPerEncoderTick));
        EasyShuffleBoard.putNumber("arm", "current desired position (not updated)", Math.toDegrees(desiredPosition * radianPerEncoderTick));
        EasyShuffleBoard.putNumber("arm", "current desired position (with schedule)", Math.toDegrees(currentDesiredPositionAccordingToSchedule * radianPerEncoderTick));
        EasyShuffleBoard.putNumber("arm", "current desired velocity with schedule",  Math.toDegrees(currentSchedule.getCurrentSpeed(scheduleTimer) * radianPerEncoderTick));
        EasyShuffleBoard.putNumber("arm", "gravity correction power", gravityCorrectionPower);
        EasyShuffleBoard.putNumber("arm", "pid correction power", pidCorrectionPower);
        EasyShuffleBoard.putNumber("arm", "overall correction power: ", overallCorrectionPower);

        previousTimeMillis = System.currentTimeMillis();
        if (Math.abs(overallCorrectionPower) > profile.staticPIDProfile.getMaxPowerAllowed())
            return Math.copySign(profile.staticPIDProfile.getMaxPowerAllowed(), overallCorrectionPower);
        return overallCorrectionPower;
    }

    public void updateArmProfile(ArmProfile newArmProfile) {
        this.profile = newArmProfile;
        this.enhancedPIDController.setPidProfile(newArmProfile.staticPIDProfile);
    }

    public void reset(double initialPosition) {
        this.enhancedPIDController.reset(initialPosition, false);
    }


    public static final class ArmProfile {
        public final LookUpTable gravityTorqueEquilibriumMotorPowerLookUpTable;
        public final EnhancedPIDController.StaticPIDProfile staticPIDProfile;
        public final EnhancedPIDController.DynamicalPIDProfile dynamicalPIDProfile;

        /**
         * Creates a arm PID profile which is another dynamic pid profile
         *
         * @param maxPowerAllowed                       the restriction on power
         * @param errorTolerance                        the amount of error to ignore
         * @param integralCoefficientError              the coefficient of the cumulated error, also known as kI
         * @param maxAcceleration                       the maximum instant acceleration that the mechanism can achieve with the max power
         * @param maxVelocity                           the restriction on the velocity of the mechanism
         */
        public ArmProfile(double maxPowerAllowed, double errorStartDecelerate, double errorTolerance, double feedForwardTime, double integralCoefficient, double maxAcceleration, double maxVelocity, LookUpTable gravityTorqueEquilibriumMotorPowerLookUpTable) {
            this.dynamicalPIDProfile = new EnhancedPIDController.DynamicalPIDProfile(Double.POSITIVE_INFINITY, maxPowerAllowed, 0, errorTolerance, integralCoefficient, 0, maxAcceleration, maxVelocity);
            this.staticPIDProfile = new EnhancedPIDController.StaticPIDProfile(Double.POSITIVE_INFINITY, maxPowerAllowed, 0, errorStartDecelerate, errorTolerance, feedForwardTime, integralCoefficient, 0);
            this.gravityTorqueEquilibriumMotorPowerLookUpTable = gravityTorqueEquilibriumMotorPowerLookUpTable;
        }
    }
}
