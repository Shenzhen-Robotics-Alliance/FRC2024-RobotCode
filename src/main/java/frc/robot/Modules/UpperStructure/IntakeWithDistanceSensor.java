package frc.robot.Modules.UpperStructure;

import frc.robot.Drivers.DistanceSensors.DistanceSensor;
import frc.robot.Drivers.Encoders.Encoder;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Services.RobotServiceBase;
import frc.robot.Utils.EasyShuffleBoard;
import frc.robot.Utils.MechanismControllers.EnhancedPIDController;
import frc.robot.Utils.RobotConfigReader;

public class IntakeWithDistanceSensor extends Intake {
    private final Motor intakeMotor, intakeAidMotor;
    private final Encoder intakeEncoder;
    private final DistanceSensor intakeDistanceSensor;
    private final RobotConfigReader robotConfig;

    public IntakeWithDistanceSensor(Motor intakeMotor, Motor intakeAidMotor, Encoder intakeEncoder, DistanceSensor intakeDistanceSensor, RobotConfigReader robotConfig) {
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

    private double intakeWheelHoldingPosition = 0;
    public double decidedIntakeMotorPower(double dt) {
        switch (currentStatus) {
            case GRABBING -> {
                if (isNoteInsideIntake())
                    return updateStatusToHolding();
                return intakePower;
            }
            case HOLDING -> {
                if (!isNoteInsideIntake()) {
                    System.out.println("<-- Intake | note gone when holding, updating to disabled... -->");
                    return updateStatusToDisabled();
                }
                intakeWheelPositionController.startNewTask(new EnhancedPIDController.Task(EnhancedPIDController.Task.TaskType.GO_TO_POSITION, intakeWheelHoldingPosition));
                final double holdPower = intakeWheelPositionController.getMotorPower(intakeEncoder.getEncoderPosition(), intakeEncoder.getEncoderVelocity(), 0); // dt does not matter
                EasyShuffleBoard.putNumber("intake", "holding power", holdPower);
                return holdPower;
            }
            case LAUNCHING -> {
                if (!isNoteInsideIntake())
                    return updateStatusToDisabled();
                return launchPower;
            }
            case SPLITTING -> {
                timeSinceSplitProcessStarted += dt;
                if (timeSinceSplitProcessStarted > splitTime)
                    return updateStatusToDisabled();
                return revertPower;
            }
        }
        return 0;
    }

    @Override
    protected double updateStatusToHolding() {
        super.updateStatusToHolding();
        intakeWheelPositionController.startNewTask(new EnhancedPIDController.Task(EnhancedPIDController.Task.TaskType.GO_TO_POSITION,
                this.intakeWheelHoldingPosition = this.intakeEncoder.getEncoderPosition() + intakeSensorToReadyPositionDifference));
        return intakeWheelPositionController.getMotorPower(intakeEncoder.getEncoderPosition(), intakeEncoder.getEncoderVelocity(), 0); // dt does not matter
    }

    private EnhancedPIDController intakeWheelPositionController;
    private double intakePower, intakeAidingMotorPower, launchPower, revertPower, distanceSensorThreshold, splitTime,
    /** the difference, in encoder ticks, between the position at which the note sensor is triggered and the desired note position */
            intakeSensorToReadyPositionDifference;
    @Override
    public void updateConfigs() {
        // this.intakeAidingMotorPower = robotConfig.getConfig("intake", "intakeAidPower");
        this.intakePower = robotConfig.getConfig("intake", "intakePower");
        this.intakeAidingMotorPower = robotConfig.getConfig("intake", "intakeAidPower");
        this.revertPower = robotConfig.getConfig("intake", "revertPower");
        this.launchPower = robotConfig.getConfig("intake", "launchPower");
        this.distanceSensorThreshold = robotConfig.getConfig("intake", "distanceSensorThreshold");
        this.splitTime = robotConfig.getConfig("intake", "splitTime");

        // TODO put the following in robotConfig
        final double intakeMotorEncoderTicksPerSecondAtNormalPower = 74000;
        EnhancedPIDController.PIDProfile intakeMotorPIDProfile = new EnhancedPIDController.StaticPIDProfile(
                Double.POSITIVE_INFINITY,
                0.45,
                0.02,
                intakeMotorEncoderTicksPerSecondAtNormalPower * 0.3,
                intakeMotorEncoderTicksPerSecondAtNormalPower * 0.01,
                0.1,
                0,
                0);
        intakeWheelPositionController = new EnhancedPIDController(intakeMotorPIDProfile);
        intakeSensorToReadyPositionDifference = intakeMotorEncoderTicksPerSecondAtNormalPower * -0.02;
    }

    @Override
    public void onReset() {
        updateConfigs();
        this.currentStatus = IntakeModuleStatus.YIELD;
        intakeMotor.gainOwnerShip(this);
        intakeMotor.setMotorZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE, this);
        intakeAidMotor.gainOwnerShip(this);
        intakeAidMotor.setMotorZeroPowerBehavior(Motor.ZeroPowerBehavior.RELAX, this);

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
    public void startIntake(RobotServiceBase operatorService) {
        if (isNoteInsideIntake())
            this.currentStatus = IntakeModuleStatus.HOLDING;
        else
            super.startIntake(operatorService);
    }

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

    @Override
    public boolean malFunctioning() {
        return intakeDistanceSensor.errorDetected();
    }
}
