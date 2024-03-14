package frc.robot.Modules.UpperStructure;

import frc.robot.Drivers.DistanceSensors.DistanceSensor;
import frc.robot.Drivers.Encoders.Encoder;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Services.RobotServiceBase;
import frc.robot.Utils.EasyShuffleBoard;
import frc.robot.Utils.MechanismControllers.EnhancedPIDController;
import frc.robot.Utils.RobotConfigReader;
import frc.robot.Utils.RobotModuleOperatorMarker;

public class IntakeWithLowerDistanceSensor extends Intake {
    private final Motor intakeMotor, intakeAidMotor;
    private final Encoder intakeEncoder;
    private final DistanceSensor intakeDistanceSensor;
    private final RobotConfigReader robotConfig;
    private boolean noteAlreadyInIntake;

    public IntakeWithLowerDistanceSensor(Motor intakeMotor, Motor intakeAidMotor, Encoder intakeEncoder, DistanceSensor intakeDistanceSensor, RobotConfigReader robotConfig) {
        super();
        this.intakeAidMotor = intakeAidMotor;
        this.intakeMotor = intakeMotor;
        this.intakeEncoder = intakeEncoder;
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
        switch (currentStatus) {
            case GRABBING -> {
                this.noteAlreadyInIntake |= isNoteInsideIntake();

                if (noteAlreadyInIntake && !isNoteInsideIntake())
                    return updateStatusToHolding();
                return noteAlreadyInIntake ? moveNoteInsideIntakeFastPower : intakePower;
            }
            case HOLDING -> {
                return noteSensedBySensor() ? moveNoteInsideIntakePower : 0;
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

    private double intakePower, intakeAidingMotorPower, moveNoteInsideIntakePower, moveNoteInsideIntakeFastPower, launchPower, revertPower, distanceSensorThreshold, splitTime, launchTime;
    @Override
    public void updateConfigs() {
        // this.intakeAidingMotorPower = robotConfig.getConfig("intake", "intakeAidPower");
        this.intakePower = robotConfig.getConfig("intake", "intakePower");
        this.intakeAidingMotorPower = robotConfig.getConfig("intake", "intakeAidPower");
        this.moveNoteInsideIntakePower = robotConfig.getConfig("intake", "moveNoteInsideIntakePower");
        this.moveNoteInsideIntakeFastPower = robotConfig.getConfig("intake", "moveNoteInsideIntakeFastPower");
        this.revertPower = robotConfig.getConfig("intake", "revertPower");
        this.launchPower = robotConfig.getConfig("intake", "launchPower");
        this.distanceSensorThreshold = robotConfig.getConfig("intake", "distanceSensorThreshold");
        this.splitTime = robotConfig.getConfig("intake", "splitTime");
        this.launchTime = robotConfig.getConfig("intake", "launchTime");
    }

    @Override
    public void onReset() {
        updateConfigs();
        this.currentStatus = IntakeModuleStatus.YIELD;
        intakeMotor.gainOwnerShip(this);
        intakeMotor.setMotorZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE, this);
        intakeAidMotor.gainOwnerShip(this);
        intakeAidMotor.setMotorZeroPowerBehavior(Motor.ZeroPowerBehavior.RELAX, this);

        timeSinceSplitOrShootProcessStarted = 0;
        noteAlreadyInIntake = false;
    }

    private double timeSinceSplitOrShootProcessStarted;

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

        super.startIntake(operator);
    }

    @Override
    public void startSplit(RobotModuleOperatorMarker operator) {
        if (!isOwner(operator) || this.currentStatus == IntakeModuleStatus.SPLITTING)
            return;

        this.timeSinceSplitOrShootProcessStarted = 0;
        super.startLaunch(operator);
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
}
