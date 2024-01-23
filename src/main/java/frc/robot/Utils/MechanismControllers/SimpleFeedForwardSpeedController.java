package frc.robot.Utils.MechanismControllers;

/**
 * a simple velocity controller
 */
public class SimpleFeedForwardSpeedController {
    private static final double nanoToSec = 1_000_000_000.0;
    private final SimpleFeedForwardControllerProfile profile;
    private double previousSpeed;
    private double previousTimeNano;
    /**
     *  initializes the speed controller with parameters
     * @param profile the profile for the speed controller
     *  */
    public SimpleFeedForwardSpeedController(SimpleFeedForwardControllerProfile profile) {
        this.profile = profile;
    }

    public double getSpeedControlPower(double currentSpeed, double desiredSpeed) {
        double basePower = desiredSpeed * profile.proportionGain;
        double dt = (System.nanoTime() - previousTimeNano) / nanoToSec;
        double currentAcceleration = (currentSpeed - previousSpeed) / dt;
        double predictedSpeed = currentSpeed + currentAcceleration * profile.feedForwardDelay;
        double feedForwardPower = (desiredSpeed - predictedSpeed) * profile.feedForwardGain;

        this.previousSpeed = currentSpeed;
        this.previousTimeNano = System.nanoTime();

        final double correctionPower = basePower + feedForwardPower;
        if (desiredSpeed != 0 && Math.abs(correctionPower) < profile.frictionGain)
            return Math.copySign(profile.frictionGain, correctionPower);

        return correctionPower;
    }

    public SimpleFeedForwardControllerProfile getSpeedControllerProfile() {
        return profile;
    }

    public static class SimpleFeedForwardControllerProfile {
        public final double proportionGain, feedForwardGain, frictionGain, feedForwardDelay;
        public SimpleFeedForwardControllerProfile(double proportionGain, double feedForwardGain, double frictionGain, double feedForwardDelay) {
            this.proportionGain = proportionGain;
            this.feedForwardGain = feedForwardGain;
            this.frictionGain = frictionGain;
            this.feedForwardDelay = feedForwardDelay;
        }
    }
}
