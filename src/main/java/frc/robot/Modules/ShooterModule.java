package frc.robot.Modules;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Utils.MechanismControllers.EncoderMotorMechanism;
import frc.robot.Utils.MechanismControllers.FlyWheelSpeedController;

import java.util.Arrays;

public class ShooterModule extends RobotModuleBase {
    private final EncoderMotorMechanism[] shooters;
    private final FlyWheelSpeedController[] speedControllers;
    private final double encoderVelocityToRPM;
    private double desiredRPM;
    public ShooterModule(EncoderMotorMechanism[] shooters, FlyWheelSpeedController.FlyWheelSpeedControllerProfile flyWheelSpeedControllerProfile) {
        this(shooters, flyWheelSpeedControllerProfile, 2048);
    }
    public ShooterModule(EncoderMotorMechanism[] shooters, FlyWheelSpeedController.FlyWheelSpeedControllerProfile flyWheelSpeedControllerProfile, int encoderTicksPerRevolution) {
        super("Shooter");
        this.shooters = shooters;
        super.motors.addAll(Arrays.asList(shooters));
        speedControllers = new FlyWheelSpeedController[shooters.length];
        for (int shooterID = 0; shooterID < shooters.length; shooterID++) {
            speedControllers[shooterID] = new FlyWheelSpeedController(flyWheelSpeedControllerProfile);
            shooters[shooterID].setController(speedControllers[shooterID]);
        }
        this.encoderVelocityToRPM = 60.0 / encoderTicksPerRevolution;
        for (int shooterID = 0; shooterID < shooters.length; shooterID++)
            Shuffleboard.getTab("Shooter").addDouble("Shooter " + shooterID + " actual speed", shooters[shooterID]::getRotterRPM);
        Shuffleboard.getTab("Shooter").addDouble("Shooter Desired RPM", () -> desiredRPM);
    }

    /**
     * set the desired speed of the shooter, 0 for stop
     * @param desiredSpeedRPM the desired speed of the shooter, in rpm
     * */
    public void setDesiredSpeed(double desiredSpeedRPM) {
        this.desiredRPM = desiredSpeedRPM;
        final double desiredEncoderVelocity = desiredSpeedRPM / encoderVelocityToRPM;
        for (int shooterID = 0; shooterID < shooters.length; shooterID++)
            speedControllers[shooterID].setDesiredSpeed(desiredEncoderVelocity, Math.abs(shooters[shooterID].getEncoderVelocity()));
    }

    @Override
    public void init() {
        for (EncoderMotorMechanism shooter:shooters) {
            shooter.setMotorZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE, this);
            shooter.gainOwnerShip(this);
        }
        this.resetModule();
    }

    @Override
    protected void periodic(double dt) {
        for (EncoderMotorMechanism shooter : shooters)
            shooter.updateWithController(this);
    }

    @Override
    public void resetModule() {
        setDesiredSpeed(0);
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
}
