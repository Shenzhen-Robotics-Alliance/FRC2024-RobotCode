package frc.robot.Modules.UpperStructure;

import frc.robot.Drivers.DistanceSensors.DistanceSensor;
import frc.robot.Drivers.Encoders.Encoder;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Utils.EasyShuffleBoard;
import frc.robot.Utils.MechanismControllers.EnhancedPIDController;
import frc.robot.Utils.RobotConfigReader;
import frc.robot.Utils.RobotModuleOperatorMarker;

public class IntakeWithDistanceSensor extends Intake {
    private final Motor intakeMotor, intakeAidMotor;
    private final Encoder intakeEncoder;
    private final DistanceSensor intakeDistanceSensor;
    private final RobotConfigReader robotConfig;
    private final TransformableArm arm;

    public IntakeWithDistanceSensor(Motor intakeMotor, Motor intakeAidMotor, Encoder intakeEncoder, DistanceSensor intakeDistanceSensor, TransformableArm arm, RobotConfigReader robotConfig) {
        super();
        this.intakeAidMotor = intakeAidMotor;
        this.intakeMotor = intakeMotor;
        this.intakeEncoder = intakeEncoder;
        this.intakeDistanceSensor = intakeDistanceSensor;
        super.motors.add(intakeMotor);
        super.motors.add(intakeAidMotor);
        this.robotConfig = robotConfig;
        this.arm = arm;
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
            case SPLITTING ->-intakeAidingMotorPower;
            default -> 0;
        }, this);
        EasyShuffleBoard.putNumber("intake", "note sensor reading (CM)", intakeDistanceSensor.getDistanceCM());
    }

    private double intakeWheelHoldingPosition = 0;
    public double decidedIntakeMotorPower(double dt) {
        // System.out.println("<-- intake current status" + this.currentStatus + " -->");
        return switch (currentStatus) {
            case GRABBING -> {
                if (intakeDistanceSensor.getDistanceCM(100) <= distanceSensorThreshold)
                    yield updateStatusToHolding();
                yield intakePower;
            }
            case HOLDING -> {
                if (!isNoteInsideIntake()) {
                    System.out.println("<-- Intake | note gone when holding, updating to disabled... -->");
                    yield updateStatusToDisabled();
                }
                intakeWheelPositionController.startNewTask(new EnhancedPIDController.Task(EnhancedPIDController.Task.TaskType.GO_TO_POSITION, intakeWheelHoldingPosition));
                final double holdPower = intakeWheelPositionController.getMotorPower(intakeEncoder.getEncoderPosition(), intakeEncoder.getEncoderVelocity(), 0); // dt does not matter
                EasyShuffleBoard.putNumber("intake", "holding power", holdPower);
                yield holdPower;
            }
            case LAUNCHING -> {
                timeSinceSplitOrShootProcessStarted += dt;
                if (timeSinceSplitOrShootProcessStarted > launchTime)
                    yield updateStatusToDisabled();
                yield launchPower;
            }
            case SPLITTING -> {
                timeSinceSplitOrShootProcessStarted += dt;
                if (timeSinceSplitOrShootProcessStarted > splitTime)
                    yield updateStatusToDisabled();
                yield arm.transformerInPosition() ? revertPower : 0;
            }
            case SPECIFY_POWER -> super.specifiedPower;
            case OFF -> 0.0;
        };
    }

    @Override
    protected double updateStatusToHolding() {
        super.updateStatusToHolding();
        intakeWheelPositionController.startNewTask(new EnhancedPIDController.Task(EnhancedPIDController.Task.TaskType.GO_TO_POSITION,
                this.intakeWheelHoldingPosition = this.intakeEncoder.getEncoderPosition() + intakeSensorToReadyPositionDifference));
        return intakeWheelPositionController.getMotorPower(intakeEncoder.getEncoderPosition(), intakeEncoder.getEncoderVelocity(), 0); // dt does not matter
    }

    private EnhancedPIDController intakeWheelPositionController;
    private double intakePower, intakeAidingMotorPower, launchPower, revertPower, distanceSensorThreshold, splitTime, launchTime,
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
        this.launchTime = robotConfig.getConfig("intake", "launchTime");

        // TODO put the following in robotConfig and make this CM
        final double intakeMotorEncoderTicksPerSecondAtNormalPower = 74000;
        EnhancedPIDController.PIDProfile intakeMotorPIDProfile = new EnhancedPIDController.StaticPIDProfile(
                Double.POSITIVE_INFINITY,
//                0.12,
//                0.02,
                0, 0,
                intakeMotorEncoderTicksPerSecondAtNormalPower * 0.15,
                intakeMotorEncoderTicksPerSecondAtNormalPower * 0.01,
                0.04,
                0,
                0);
        intakeWheelPositionController = new EnhancedPIDController(intakeMotorPIDProfile);
        intakeSensorToReadyPositionDifference = intakeMotorEncoderTicksPerSecondAtNormalPower * -0.025;
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
    }

    private double timeSinceSplitOrShootProcessStarted;

    @Override
    public void startIntake(RobotModuleOperatorMarker operator) {
        if (isNoteInsideIntake())
            this.currentStatus = IntakeModuleStatus.HOLDING;
        else
            super.startIntake(operator);
    }

    @Override
    public void startSplit(RobotModuleOperatorMarker operator) {
        if (!isOwner(operator))
            return;
        timeSinceSplitOrShootProcessStarted = 0;
        super.startSplit(operator);
    }

    @Override
    public void startLaunch(RobotModuleOperatorMarker operator) {
        if (!isOwner(operator))
            return;
        timeSinceSplitOrShootProcessStarted = 0;
        super.startLaunch(operator);
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
