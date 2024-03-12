package frc.robot.Modules.UpperStructure;

import frc.robot.Modules.RobotModuleBase;
import frc.robot.Services.RobotServiceBase;

public abstract class Intake extends RobotModuleBase {
    // TODO handle the situation when the distance sensor is disconnected
    protected Intake() {
        super("Intake");
    }

    public enum IntakeModuleStatus {
        YIELD,
        HOLDING,
        GRABBING,
        LAUNCHING,
        SPLITTING
    }
    protected IntakeWithDistanceSensor.IntakeModuleStatus currentStatus;

    /**
     * @return the intake motor power at disabled status, which is always 0.0
     * */
    protected double updateStatusToDisabled() {
        this.currentStatus = IntakeWithDistanceSensor.IntakeModuleStatus.YIELD;
        return 0;
    }

    protected double updateStatusToHolding() {
        this.currentStatus = IntakeModuleStatus.HOLDING;
        return 0;
    }

    /**
     * turn off the intake
     * this will terminate the current intake process and stop the motors, until the next time startIntake() or
     * */
    public void turnOffIntake(RobotServiceBase operatorService) {
        if (!isOwner(operatorService) || currentStatus == IntakeModuleStatus.HOLDING)
            return;
        this.currentStatus = IntakeModuleStatus.YIELD;
    }
    /**
     *  start the intake process
     *  the intake module will run the motors to suck the note in, and will automatically stop
     *  */
    public void startIntake(RobotServiceBase operatorService) {
        if (!isOwner(operatorService))
            return;

        this.currentStatus = IntakeModuleStatus.GRABBING;
    }

    public void startLaunch(RobotServiceBase operatorService) {
        if (!isOwner(operatorService))
            return;

        this.currentStatus = IntakeModuleStatus.LAUNCHING;
    }

    public void startSplit(RobotServiceBase operatorService) {
        if (!isOwner(operatorService))
            return;

        this.currentStatus = IntakeModuleStatus.SPLITTING;
    }

    public boolean isCurrentTaskComplete() {
        return currentStatus == IntakeModuleStatus.YIELD;
    }

    public IntakeModuleStatus getCurrentStatus() {
        return this.currentStatus;
    }

    public abstract boolean isNoteInsideIntake();

    public boolean malFunctioning() {return true;}
}
