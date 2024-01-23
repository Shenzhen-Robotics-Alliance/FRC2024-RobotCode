package frc.robot.Utils.MechanismControllers;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * controls the speed of fly wheel
 * speed is also positive
 * */
public class FlyWheelSpeedController implements MechanismController {
    private final SimpleFeedForwardSpeedController simpleFeedForwardSpeedController;
    private final FlyWheelSpeedControllerProfile profile;
    private static final double nanoToSec = 1_000_000_000.0;
    /** if the mechanism can reach desired speed within this time, just skip to desired speed */
    private static final double jumpToDesiredSpeedTimeInterval = 0.2;

    private double speedWhenTaskStarted, desiredSpeed;
    private long taskStartTimeNano;
    public FlyWheelSpeedController(FlyWheelSpeedControllerProfile profile) {
        this.profile = profile;
        this.simpleFeedForwardSpeedController = new SimpleFeedForwardSpeedController(profile);
        startNewSpeedControlTask(0, 0);

        Shuffleboard.getTab("Shooter").addDouble("flywheel controller current target speed", this::getCurrentTargetSpeedWithLERP);
    }

    public void setDesiredSpeed(double desiredSpeed, double currentSpeed) {
        // v = at, t = v / a
        if (Math.abs(desiredSpeed - currentSpeed) / profile.maximumAcceleration < jumpToDesiredSpeedTimeInterval)
            this.desiredSpeed = currentSpeed;
        else
            startNewSpeedControlTask(desiredSpeed, currentSpeed);
    }

    public double getCorrectionPower(double currentSpeed) {
        if (desiredSpeed <= profile.maximumSpeed * 0.05) return 0;
        currentSpeed = Math.abs(currentSpeed);
        final double correctionSpeed = simpleFeedForwardSpeedController.getSpeedControlPower(
                currentSpeed / profile.maximumSpeed,
                getCurrentTargetSpeedWithLERP() / profile.maximumSpeed
        );
        return Math.max(correctionSpeed, 0); // do not go negative power
    }

    private void startNewSpeedControlTask(double desiredSpeed, double currentSpeed) {
        this.desiredSpeed = desiredSpeed;
        this.speedWhenTaskStarted = currentSpeed;
        this.taskStartTimeNano = System.nanoTime();
    }

    private double getCurrentTargetSpeedWithLERP() {
        final double speedDifferenceBetweenTaskStartAndEnd = desiredSpeed - speedWhenTaskStarted,
                speedDifferenceMaximumMagnitude = Math.abs(speedDifferenceBetweenTaskStartAndEnd),
                timeSinceTaskStarted = (System.nanoTime() - taskStartTimeNano) / nanoToSec,
                speedDifferenceReached = Math.min(timeSinceTaskStarted * profile.maximumAcceleration, speedDifferenceMaximumMagnitude),
                currentTargetSpeed = speedWhenTaskStarted + Math.copySign(speedDifferenceReached, speedDifferenceBetweenTaskStartAndEnd);
        return Math.max(Math.min(profile.maximumSpeed, currentTargetSpeed),0);
    }

    @Override
    public double getMotorPower(double mechanismVelocity, double mechanismPosition) {
        return this.getCorrectionPower(mechanismVelocity);
    }

    public static class FlyWheelSpeedControllerProfile extends SimpleFeedForwardSpeedController.SimpleFeedForwardControllerProfile {
        public final double maximumSpeed, maximumAcceleration;
        public FlyWheelSpeedControllerProfile(double proportionGain, double feedForwardGain, double frictionGain, double feedForwardDelay, double maximumSpeed, double timeNeededToAccelerateToMaxSpeed) {
            super(proportionGain, feedForwardGain, frictionGain, feedForwardDelay);
            this.maximumSpeed = maximumSpeed;
            this.maximumAcceleration = maximumSpeed/timeNeededToAccelerateToMaxSpeed;
        }
    }

    public FlyWheelSpeedControllerProfile getProfile() {
        return profile;
    }
}
