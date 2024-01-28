package frc.robot.Modules.UpperStructure;

import frc.robot.Drivers.Motors.Motor;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.RobotConfigReader;

/**
 * TODO here we need to add a distance sensor or color sensor or whatever that senses the motion of the Note
 * */
public class Intake extends RobotModuleBase {

    private final Motor intakeMotor;
    private final RobotConfigReader robotConfig;
    public enum IntakeModuleTask {
        DISABLED,
        RUNNING,
        REVERTING
    }
    private IntakeModuleTask task;
    public Intake(Motor intakeMotor, RobotConfigReader robotConfig) {
        super("Intake");
        this.intakeMotor = intakeMotor;
        super.motors.add(intakeMotor);
        this.robotConfig = robotConfig;
    }

    @Override
    public void init() {
        intakeMotor.gainOwnerShip(this);
        updateConfigs();
        reset();
    }

    private double intakePower;
    private double revertPower;
    @Override
    protected void periodic(double dt) {
        switch (this.task) {
            case RUNNING -> intakeMotor.setPower(intakePower, this);
            case REVERTING -> intakeMotor.setPower(revertPower, this);
            case DISABLED -> intakeMotor.setPower(0, this);
        }
    }

    @Override
    public void updateConfigs() {
        this.intakePower = robotConfig.getConfig("intake", "intakePower");
        this.revertPower = robotConfig.getConfig("intake", "revertPower");
    }

    @Override
    public void resetModule() {
        this.task = IntakeModuleTask.DISABLED;
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

    public void setTask(IntakeModuleTask task, RobotModuleBase operatorModule) {
        if (!isOwner(operatorModule))
            return;
        this.task = task;
    }
}
