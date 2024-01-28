package frc.robot.Modules.UpperStructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.MechanismControllers.EncoderMotorMechanism;
import frc.robot.Utils.MechanismControllers.FlyWheelSpeedController;
import frc.robot.Utils.RobotConfigReader;

import java.util.Arrays;

public class Shooter extends RobotModuleBase {
    private final EncoderMotorMechanism[] shooters;
    private final FlyWheelSpeedController[] speedControllers;
    private final double encoderVelocityToRPM;
    private final double shooterReadyErrorBound;
    private final RobotConfigReader robotConfig;
    private double desiredRPM;
    public Shooter(EncoderMotorMechanism[] shooters, RobotConfigReader robotConfig) {
        super("Shooter");
        this.shooters = shooters;
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
     * @param desiredSpeedRPM the desired speed of the shooter, in rpm
     * */
    public void setDesiredSpeed(double desiredSpeedRPM) {
        this.desiredRPM = desiredSpeedRPM;
        final double desiredEncoderVelocity = desiredSpeedRPM / encoderVelocityToRPM;
        for (int shooterID = 0; shooterID < shooters.length; shooterID++)
            speedControllers[shooterID].setDesiredSpeed(desiredEncoderVelocity, Math.abs(shooters[shooterID].getEncoderVelocity()));
    }

    public boolean shooterReady() {
        if (desiredRPM == 0) return false;
        double maxError = 0;
        for (EncoderMotorMechanism shooter:shooters)
            maxError = Math.max(Math.abs(shooter.getEncoderVelocity() - this.desiredRPM/ encoderVelocityToRPM), maxError);
        System.out.println("shooter max error (rpm): " + maxError * encoderVelocityToRPM);
        return maxError < shooterReadyErrorBound;
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

        for (int shooterID = 0; shooterID < shooters.length; shooterID++)
            SmartDashboard.putNumber("Shooter " + shooterID + " actual speed", shooters[shooterID].getEncoderVelocity() * encoderVelocityToRPM);
        SmartDashboard.putNumber("Shooter Desired RPM", desiredRPM);
    }

    @Override
    public void resetModule() {
        setDesiredSpeed(0);
        updateConfigs();
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

    @Override
    public void updateConfigs() {
        final FlyWheelSpeedController.FlyWheelSpeedControllerProfile speedControllerProfile = new FlyWheelSpeedController.FlyWheelSpeedControllerProfile(
                robotConfig.getConfig("shooter", "speedControllerProportionGain"),
                robotConfig.getConfig("shooter", "speedControllerFeedForwardGain"),
                robotConfig.getConfig("shooter", "speedControllerFrictionGain"),
                robotConfig.getConfig("shooter", "speedControllerFeedForwardDelay"),
                robotConfig.getConfig("shooter", "speedControllerMaximumSpeed"),
                robotConfig.getConfig("shooter", "speedControllerTimeNeededToAccelerateToMaxSpeed")
        );
        for (FlyWheelSpeedController speedController:speedControllers) {
            speedController.setProfile(speedControllerProfile);
            System.out.println("speed controller p in shooter module:" + speedController.getProfile().proportionGain);
        }
    }
}
