package frc.robot.Utils.MechanismControllers;

import frc.robot.Utils.MathUtils.LookUpTable;

public class ArmGravityController {
    private double desiredPosition;
    private boolean activated;
    private ArmProfile profile;
    private double accumulatedError;
    private long previousTimeMillis;
    public ArmGravityController(ArmProfile armProfile) {
        this.profile = armProfile;
        disable();
    }

    public void setDesiredPosition(double desiredPosition) {
        this.desiredPosition = desiredPosition;
        this.activated = true;
        previousTimeMillis = System.currentTimeMillis();
    }

    public void disable() {
        this.activated = false;
        resetErrorAccumulation();
    }

    public void resetErrorAccumulation() {
        this.accumulatedError = 0;
    }

    /**
     * @param armVelocityRadPerSec the velocity of the arm, in radian per second, POSITIVE IS UPWARDS
     * @param armPositionRad the position of the arm, in radian, where ZERO IS HORIZONTAL, POSITIVE IS UPWARDS
     * @return the correction power, in percent output, POSITIVE IS UPWARDS
     * */
    public double getMotorPower(double armVelocityRadPerSec, double armPositionRad) {
        if (!activated)
            return 0;
        final double gravityPower = profile.gravityTorqueAtArmHorizontalState * Math.cos(armPositionRad),
                mechanismPredictedPosition = armPositionRad + armVelocityRadPerSec * profile.feedForwardTime,
                mechanismError = desiredPosition - mechanismPredictedPosition,
                proportionPower = Math.abs(mechanismError) > profile.errorTolerance ?
                        profile.maximumPower * mechanismError / profile.errorStartDecelerate : 0,
                integralPower = accumulatedError * proportionPower,
                dt = (System.currentTimeMillis() - previousTimeMillis) / 1000.0;

        accumulatedError += mechanismError * profile.errorAccumulationProportion * dt;
        previousTimeMillis = System.currentTimeMillis();
        return Math.min(gravityPower + proportionPower + integralPower, profile.maximumPower);
    }

    public void setProfile(ArmProfile newProfile) {
        this.profile = newProfile;
    }

    public static final class ArmProfile {
        public final double maximumPower, errorStartDecelerate, errorTolerance, feedForwardTime, errorAccumulationProportion, gravityTorqueAtArmHorizontalState;

        public ArmProfile(double maximumPower, double errorStartDecelerate, double errorTolerance, double feedForwardTime, double errorAccumulationProportion, double gravityTorqueAtArmHorizontalState) {
            this.maximumPower = maximumPower;
            this.errorStartDecelerate = errorStartDecelerate;
            this.errorTolerance = errorTolerance;
            this.feedForwardTime = feedForwardTime;
            this.gravityTorqueAtArmHorizontalState = gravityTorqueAtArmHorizontalState;
            this.errorAccumulationProportion = errorAccumulationProportion;
        }
    }
}
