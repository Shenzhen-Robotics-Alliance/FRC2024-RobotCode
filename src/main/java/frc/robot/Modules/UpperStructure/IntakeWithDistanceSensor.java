package frc.robot.Modules.UpperStructure;

import frc.robot.Drivers.DistanceSensors.DistanceSensor;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Services.RobotServiceBase;
import frc.robot.Utils.RobotConfigReader;

public class IntakeWithDistanceSensor extends Intake {
    private final Motor intakeMotor;
    private final DistanceSensor intakeDistanceSensor;
    private final RobotConfigReader robotConfig;
    public IntakeWithDistanceSensor(Motor intakeMotor, DistanceSensor intakeDistanceSensor, RobotConfigReader robotConfig) {
        super();
        this.intakeMotor = intakeMotor;
        this.intakeDistanceSensor = intakeDistanceSensor;
        super.motors.add(intakeMotor);
        this.robotConfig = robotConfig;
    }

    @Override
    public void init() {
        resetModule();
    }

    private double intakePower, launchPower, revertPower, distanceSensorThreshold, splitTime;
    @Override
    protected void periodic(double dt) {
        intakeMotor.setPower(decidedIntakeMotorPower(dt), this);
    }

    public double decidedIntakeMotorPower(double dt) {
        switch (currentStatus) {
            case GRABBING -> {
                if (isNoteInsideIntake())
                    return updateStatusToDisabled();
                return intakePower;
            }
            case LAUNCHING -> {
                if (!isNoteInsideIntake())
                    return updateStatusToDisabled();
                return launchPower;
            }
            case REVERTING -> {
                timeSinceSplitProcessStarted += dt;
                if (timeSinceSplitProcessStarted > splitTime)
                    return updateStatusToDisabled();
                return revertPower;
            }
        }
        return 0;
    }

    @Override
    public void updateConfigs() {
        this.intakePower = robotConfig.getConfig("intake", "intakePower");
        this.revertPower = robotConfig.getConfig("intake", "revertPower");
        this.launchPower = robotConfig.getConfig("intake", "launchPower");
        this.distanceSensorThreshold = robotConfig.getConfig("intake", "distanceSensorThreshold");
        this.splitTime = robotConfig.getConfig("intake", "splitTime");
    }

    @Override
    public void resetModule() {
        updateConfigs();
        this.currentStatus = IntakeModuleStatus.DISABLED;
        intakeMotor.gainOwnerShip(this);
        intakeMotor.setMotorZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE, this);
        timeSinceSplitProcessStarted = 0;
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

    private double timeSinceSplitProcessStarted;

    @Override
    public void startSplit(RobotServiceBase operatorService) {
        if (!isOwner(operatorService))
            return;
        timeSinceSplitProcessStarted = 0;
        super.startSplit(operatorService);
    }

    @Override
    public boolean isNoteInsideIntake() {
        return intakeDistanceSensor.getDistanceCM() <= distanceSensorThreshold;
    }
}
