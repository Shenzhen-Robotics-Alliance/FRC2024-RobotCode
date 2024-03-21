package frc.robot.Modules.UpperStructure;

import frc.robot.Drivers.DistanceSensors.DistanceSensor;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Services.RobotServiceBase;
import frc.robot.Utils.EasyShuffleBoard;
import frc.robot.Utils.RobotConfigReader;
import frc.robot.Utils.RobotModuleOperatorMarker;

public class IntakeWithLowerDistanceSensor extends Intake {
    private final Motor intakeMotor, intakeAidMotor;
    private final DistanceSensor intakeDistanceSensor;
    private final RobotConfigReader robotConfig;
    private boolean noteAlreadyInIntake, noteSensedByDistanceSensorDuringCurrentIntakeTask, noteAlreadyInPositionDuringCurrentIntakeTask;
    private double timeSinceNoteInPosition;

    public IntakeWithLowerDistanceSensor(Motor intakeMotor, Motor intakeAidMotor, DistanceSensor intakeDistanceSensor, RobotConfigReader robotConfig) {
        super();
        this.intakeAidMotor = intakeAidMotor;
        this.intakeMotor = intakeMotor;
        this.intakeDistanceSensor = intakeDistanceSensor;
        super.motors.add(intakeMotor);
        super.motors.add(intakeAidMotor);
        this.robotConfig = robotConfig;
    }

    @Override
    public void init() {
        onReset();
    }

    @Override
    protected void periodic(double dt) {
        intakeMotor.setPower(decidedIntakeMotorPower(dt), this);
        intakeAidMotor.setPower(switch(currentStatus) {
            case GRABBING -> intakeAidingMotorPower;
            case SPLITTING -> -intakeAidingMotorPower;
            default -> 0;
        }, this);
        EasyShuffleBoard.putNumber("intake", "note sensor reading (CM)", intakeDistanceSensor.getDistanceCM());
    }

    public double decidedIntakeMotorPower(double dt) {
        System.out.println("intake current status: " + currentStatus);
        System.out.println("is note inside intake: " + isNoteInsideIntake());
        switch (currentStatus) {
            case GRABBING -> {
                /* wait for the note to come up from the feeder and get sensed by the sensor for the first time */
                if (!noteSensedByDistanceSensorDuringCurrentIntakeTask && noteSensedBySensor()) {
                    noteSensedByDistanceSensorDuringCurrentIntakeTask = true;
                    noteAlreadyInPositionDuringCurrentIntakeTask = false;
                }

                /* spin the intake slowly up and wait for the note to go so high that the sensor can't see */
                if (noteSensedByDistanceSensorDuringCurrentIntakeTask) {
                    if (!noteAlreadyInPositionDuringCurrentIntakeTask && !noteSensedBySensor()) {
                        noteAlreadyInPositionDuringCurrentIntakeTask = true;
                        timeSinceNoteInPosition = 0;
                    }
                    /* now that the sensor can't see the note, we back it up for 0.2 */
                    if (noteAlreadyInPositionDuringCurrentIntakeTask) {
                        timeSinceNoteInPosition += dt;
                        if (timeSinceNoteInPosition > moveBackToPositionTime) {
                            noteAlreadyInIntake = true;
                            return updateStatusToHolding();
                        }
                        return moveNoteDownInsideIntakePower;
                    }
                    return moveNoteUpInsideIntakePower;
                }
                return intakePower;
            }
            case HOLDING -> {
                return 0;
            }
            case LAUNCHING -> {
                timeSinceSplitOrShootProcessStarted += dt;
                if (timeSinceSplitOrShootProcessStarted > launchTime) {
                    this.noteAlreadyInIntake = false;
                    return updateStatusToDisabled();
                }
                return launchPower;
            }
            case SPLITTING -> {
                timeSinceSplitOrShootProcessStarted += dt;
                if (timeSinceSplitOrShootProcessStarted > splitTime) {
                    this.noteAlreadyInIntake = false;
                    return updateStatusToDisabled();
                }
                return revertPower;
            }
        }
        return 0;
    }

    private double intakePower, intakeAidingMotorPower, moveNoteDownInsideIntakePower, moveNoteUpInsideIntakePower, launchPower, revertPower, distanceSensorThreshold, splitTime, launchTime, moveBackToPositionTime;
    @Override
    public void updateConfigs() {
        // this.intakeAidingMotorPower = robotConfig.getConfig("intake", "intakeAidPower");
        this.intakePower = robotConfig.getConfig("intake", "intakePower");
        this.intakeAidingMotorPower = robotConfig.getConfig("intake", "intakeAidPower");
        this.moveNoteDownInsideIntakePower = robotConfig.getConfig("intake", "moveNoteDownInsideIntakePower");
        this.moveNoteUpInsideIntakePower = robotConfig.getConfig("intake", "moveNoteUpInsideIntakePower");
        this.revertPower = robotConfig.getConfig("intake", "revertPower");
        this.launchPower = robotConfig.getConfig("intake", "launchPower");
        this.distanceSensorThreshold = robotConfig.getConfig("intake", "distanceSensorThreshold");
        this.splitTime = robotConfig.getConfig("intake", "splitTime");
        this.launchTime = robotConfig.getConfig("intake", "launchTime");
        moveBackToPositionTime = robotConfig.getConfig("intake", "moveBackToPositionTime");
    }

    @Override
    public void onReset() {
        updateConfigs();
        this.currentStatus = IntakeModuleStatus.OFF;
        intakeMotor.gainOwnerShip(this);
        intakeMotor.setMotorZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE, this);
        intakeAidMotor.gainOwnerShip(this);
        intakeAidMotor.setMotorZeroPowerBehavior(Motor.ZeroPowerBehavior.RELAX, this);

        timeSinceSplitOrShootProcessStarted = 0;
        noteAlreadyInIntake = false;
    }

    private double timeSinceSplitOrShootProcessStarted;

    @Override
    public void turnOffIntake(RobotServiceBase operatorService) {
        if (
                this.currentStatus == IntakeModuleStatus.SPLITTING
                        || this.currentStatus == IntakeModuleStatus.LAUNCHING
                        || (this.currentStatus == IntakeModuleStatus.GRABBING && this.noteSensedByDistanceSensorDuringCurrentIntakeTask)
        ) return;
        super.turnOffIntake(operatorService);
    }

    @Override
    public void startLaunch(RobotModuleOperatorMarker operator) {
        if (!isOwner(operator) || this.currentStatus == IntakeModuleStatus.LAUNCHING)
            return;

        this.timeSinceSplitOrShootProcessStarted = 0;
        super.startLaunch(operator);
    }

    @Override
    public void startIntake(RobotModuleOperatorMarker operator) {
        if (!isOwner(operator) || currentStatus == IntakeModuleStatus.GRABBING || isNoteInsideIntake())
            return;
        noteSensedByDistanceSensorDuringCurrentIntakeTask = false;
        super.startIntake(operator);
    }

    @Override
    public void startSplit(RobotModuleOperatorMarker operator) {
        if (!isOwner(operator) || this.currentStatus == IntakeModuleStatus.SPLITTING)
            return;

        this.timeSinceSplitOrShootProcessStarted = 0;
        super.startSplit(operator);
    }

    @Override
    public boolean isNoteInsideIntake() {
        return noteAlreadyInIntake;
    }

    private boolean noteSensedBySensor() {
        return intakeDistanceSensor.getDistanceCM() <= distanceSensorThreshold;
    }

    @Override
    public boolean malFunctioning() {
        return intakeDistanceSensor.errorDetected();
    }

    @Override
    public void setCurrentStatusAsHolding() {
        this.noteAlreadyInIntake = true;
        super.setCurrentStatusAsHolding();
    }
}
