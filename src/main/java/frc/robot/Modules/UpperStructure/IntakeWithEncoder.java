package frc.robot.Modules.UpperStructure;

import frc.robot.Drivers.Motors.Motor;
import frc.robot.Utils.MechanismControllers.EncoderMotorMechanism;
import frc.robot.Utils.RobotConfigReader;

/**
 * I don't think this will work
 * */
@Deprecated
public class IntakeWithEncoder extends Intake {
    private final EncoderMotorMechanism intakeEncoderMotorMechanism;
    private final RobotConfigReader robotConfig;

    public IntakeWithEncoder(EncoderMotorMechanism intakeMotorAndEncoder, RobotConfigReader robotConfig) {
        super();

        this.intakeEncoderMotorMechanism = intakeMotorAndEncoder;
        this.robotConfig = robotConfig;
    }
    @Override
    public void init() {
        onReset();
    }

    @Override
    protected void periodic(double dt) {

    }

    @Override
    public void onReset() {
        updateConfigs();
        this.currentStatus = IntakeModuleStatus.YIELD;
        intakeEncoderMotorMechanism.gainOwnerShip(this);
        intakeEncoderMotorMechanism.setMotorZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE, this);
    }

    @Override
    public void onDestroy() {

    }

    @Override
    protected void onEnable() {

    }

    @Override
    protected void onDisable() {

    }

    @Override
    public boolean isNoteInsideIntake() {
        return false;
    }
}
