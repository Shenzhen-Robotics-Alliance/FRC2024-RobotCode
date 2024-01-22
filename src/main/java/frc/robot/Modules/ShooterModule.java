package frc.robot.Modules;

import frc.robot.Drivers.Encoders.Encoder;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Utils.FlyWheelSpeedController;

public class ShooterModule extends RobotModuleBase {
    private final Motor shooterMotor;
    private final Encoder shooterEncoder;
    private final FlyWheelSpeedController speedController;
    private double desiredSpeed;
    public ShooterModule(Motor shooterMotor, Encoder shooterEncoder, FlyWheelSpeedController speedController) {
        super("Shooter");
        this.shooterMotor = shooterMotor;
        super.motors.add(shooterMotor);
        this.shooterEncoder = shooterEncoder;
        this.speedController = speedController;
    }

    /** set the desired speed of the shooter, 0 for stop */
    public void setDesiredSpeed(double desiredSpeed) {
        speedController.setDesiredSpeed(desiredSpeed, Math.abs(shooterEncoder.getEncoderVelocity()));
        this.desiredSpeed = desiredSpeed;
    }

    @Override
    public void init() {
        shooterMotor.setMotorZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE, this);
        shooterMotor.gainOwnerShip(this);
    }

    @Override
    protected void periodic(double dt) {
        shooterMotor.setPower(getCorrectionPower(), this);
    }

    private double getCorrectionPower() {
        if (desiredSpeed <= speedController.getProfile().maximumSpeed * 0.05)
            return 0;
        return speedController.getCorrectionPower(Math.abs(shooterEncoder.getEncoderVelocity()));
    }

    @Override
    public void resetModule() {
        desiredSpeed = 0;
    }

    @Override
    public void onDestroy() {

    }

    @Override
    protected void onEnable() {

    }

    @Override
    protected void onDisable() {
        shooterMotor.disableMotor(this);
    }
}
