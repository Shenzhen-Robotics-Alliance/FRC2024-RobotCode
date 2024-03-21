package frc.robot.Modules.UpperStructure;

import frc.robot.Modules.RobotModuleBase;
import frc.robot.Services.RobotServiceBase;
import frc.robot.Utils.RobotModuleOperatorMarker;

public abstract class Intake extends RobotModuleBase {
    // TODO handle the situation when the distance sensor is disconnected
    protected Intake() {
        super("Intake");
    }

    public enum IntakeModuleStatus {
        OFF,
        HOLDING,
        GRABBING,
        LAUNCHING,
        SPLITTING,
        SPECIFY_POWER
    }
    protected IntakeWithDistanceSensor.IntakeModuleStatus currentStatus;
    protected double specifiedPower = 0.0;

    /**
     * @return the intake motor power at disabled status, which is always 0.0
     * */
    protected double updateStatusToDisabled() {
        this.currentStatus = IntakeWithDistanceSensor.IntakeModuleStatus.OFF;
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
        this.currentStatus = IntakeModuleStatus.OFF;
    }
    /**
     *  start the intake process
     *  the intake module will run the motors to suck the note in, and will automatically stop
     *  */
    public void startIntake(RobotModuleOperatorMarker operator) {
        if (!isOwner(operator))
            return;

        this.currentStatus = IntakeModuleStatus.GRABBING;
    }

    public void startLaunch(RobotModuleOperatorMarker operator) {
        if (!isOwner(operator))
            return;

        this.currentStatus = IntakeModuleStatus.LAUNCHING;
    }

    public void startSplit(RobotModuleOperatorMarker operator) {
        if (!isOwner(operator))
            return;

        this.currentStatus = IntakeModuleStatus.SPLITTING;
    }

    public void specifyPower(double specifiedPower, RobotModuleOperatorMarker operator) {
        if (!isOwner(operator)) return;

        this.currentStatus = IntakeModuleStatus.SPECIFY_POWER;
        this.specifiedPower = specifiedPower;
    }

    public boolean isCurrentTaskComplete() {
        return currentStatus == IntakeModuleStatus.OFF;
    }

    public IntakeModuleStatus getCurrentStatus() {
        return this.currentStatus;
    }

    public abstract boolean isNoteInsideIntake();

    public boolean malFunctioning() {return true;}

    public void setCurrentStatusAsHolding() {
        this.currentStatus = IntakeModuleStatus.HOLDING;
    }
}
