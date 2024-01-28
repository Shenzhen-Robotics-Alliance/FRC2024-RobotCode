package frc.robot.Modules.UpperStructure;

import frc.robot.Drivers.DistanceSensors.DistanceSensor;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Services.RobotServiceBase;
import frc.robot.Utils.RobotConfigReader;

public class Intake extends RobotModuleBase {
    private final Motor intakeMotor;
    private final DistanceSensor intakeDistanceSensor;
    private final RobotConfigReader robotConfig;
    public enum IntakeModuleStatus {
        DISABLED,
        GRABBING,
        LAUNCHING,
        REVERTING
    }
    private IntakeModuleStatus currentStatus;
    public Intake(Motor intakeMotor, DistanceSensor intakeDistanceSensor, RobotConfigReader robotConfig) {
        super("Intake");
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
        final double intakeMotorPower = decidedIntakeMotorPower(dt);
        intakeMotor.setPower(intakeMotorPower, this);
    }

    public double decidedIntakeMotorPower(double dt) {
        switch (currentStatus) {
            case GRABBING -> {
                if (intakeDistanceSensor.getDistanceCM() <= distanceSensorThreshold)
                    return updateStatusToDisabled();
                return intakePower;
            }
            case LAUNCHING -> {
                if (intakeDistanceSensor.getDistanceCM() >= distanceSensorThreshold)
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

    /**
     * @return the intake motor power at disabled status, which is always 0.0
     * */
    private double updateStatusToDisabled() {
        this.currentStatus = IntakeModuleStatus.DISABLED;
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
        timeSinceSplitProcessStarted  = 0;
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

    /**
     * turn off the intake
     * this will terminate the current intake process and stop the motors, until the next time startIntake() or
     * */
    public void turnOffIntake(RobotServiceBase operatorService) {
        if (!isOwner(operatorService))
            return;
        this.currentStatus = IntakeModuleStatus.DISABLED;
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

    private double timeSinceSplitProcessStarted;
    public void startSplit(RobotServiceBase operatorService) {
        if (!isOwner(operatorService))
            return;

        timeSinceSplitProcessStarted = 0;
        this.currentStatus = IntakeModuleStatus.REVERTING;
    }

    public boolean isCurrentTaskComplete() {
        return currentStatus == IntakeModuleStatus.DISABLED;
    }
}
