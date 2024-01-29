package frc.robot.Utils.MechanismControllers;

import frc.robot.Utils.MathUtils.LookUpTable;

public class ArmGravityController implements MechanismController {
    /**
     * TODO
     *  here we need to achieve:
     *  1. the schedule can be interrupted and altered smoothly in the middle of execution
     *  2. we can update the destination in a small range without scheduling
     *  3. so we might want to add another subclass called armTask
     * */
    private final EnhancedPIDController enhancedPIDController;
    private double desiredPosition;
    private boolean alive;
    public ArmProfile profile;
    private long previousTimeMillis;
    public ArmGravityController(ArmProfile armProfile) {
        this.profile = armProfile;
        this.enhancedPIDController = new EnhancedPIDController(armProfile);
    }

    public void setDesiredPosition(double currentPosition, double currentVelocity, double newDesiredPosition) {
        if (newDesiredPosition == desiredPosition) return;
        this.desiredPosition = newDesiredPosition;
        this.alive = true;
        previousTimeMillis = System.currentTimeMillis();
    }

    @Override
    public double getMotorPower(double mechanismVelocity, double mechanismPosition) {
        if (!alive) return 0;
        final double gravityCorrectionPower = profile.gravityTorqueEquilibriumMotorPowerLookUpTable.getYPrediction(mechanismPosition),
                dt = (System.currentTimeMillis() - previousTimeMillis) / 1000.0,
                overallCorrectionPower = gravityCorrectionPower + enhancedPIDController.getMotorPower(mechanismPosition, mechanismVelocity, dt);
        previousTimeMillis = System.currentTimeMillis();
        if (Math.abs(overallCorrectionPower) > profile.getMaxPowerAllowed())
            return Math.copySign(profile.maxAcceleration, overallCorrectionPower);
        return overallCorrectionPower;
    }

    public void updateArmProfile(ArmProfile newArmProfile) {
        this.profile = newArmProfile;
        this.enhancedPIDController.setPidProfile(newArmProfile);
    }

    public void reset(double initialPosition) {
        this.enhancedPIDController.reset(initialPosition, false);
    }


    public static final class ArmProfile extends EnhancedPIDController.DynamicalPIDProfile {
        public final LookUpTable gravityTorqueEquilibriumMotorPowerLookUpTable;

        /**
         * Creates a arm PID profile which is another dynamic pid profile
         *
         * @param maxPowerAllowed                       the restriction on power
         * @param errorTolerance                        the amount of error to ignore
         * @param integralCoefficientError              the coefficient of the cumulated error, also known as kI
         * @param maxAcceleration                       the maximum instant acceleration that the mechanism can achieve with the max power
         * @param maxVelocity                           the restriction on the velocity of the mechanism
         */
        public ArmProfile(double maxPowerAllowed, double errorTolerance, double integralCoefficientError, double maxAcceleration, double maxVelocity, LookUpTable gravityTorqueEquilibriumMotorPowerLookUpTable) {
            super(Double.POSITIVE_INFINITY, maxPowerAllowed, 0, errorTolerance, integralCoefficientError, 0, maxAcceleration, maxVelocity);
            this.gravityTorqueEquilibriumMotorPowerLookUpTable = gravityTorqueEquilibriumMotorPowerLookUpTable;
        }
    }
}
