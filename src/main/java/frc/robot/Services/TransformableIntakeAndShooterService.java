package frc.robot.Services;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Modules.UpperStructure.Intake;
import frc.robot.Modules.UpperStructure.IntakeTransformer;
import frc.robot.Modules.UpperStructure.Shooter;
import frc.robot.Utils.RobotConfigReader;

/**
 * TODO communicate with vision-aided chassis
 * */
public class TransformableIntakeAndShooterService extends RobotServiceBase {
    private final Intake intakeModule;
    private final Shooter shooterModule;
    private final IntakeTransformer transformerModule;
    private final RobotConfigReader robotConfig;
    private final XboxController copilotController;

    public enum IntakeAndShooterStatus {
        /** the transformer is at standby position */
        AT_DEFAULT_POSITION,
        /** the transformer is at intake standby position, which does not hit the floor, waiting for intake command */
        AT_INTAKE_STANDBY_POSITION,
        /** the transformer is at intake active position, the intake wheels spin to grab the note */
        PROCEEDING_INTAKE,
        /** the transformer is at intake standby position, and the intake is splitting the note out */
        PROCEEDING_SPLIT,
        /** the transformer is at the shooting position, standing by for further instruction */
        AT_SHOOTING_STANDBY_POSITION_HOLDING_NOTE,
        /** the transformer is at the shooting position and the note is being launched */
        LAUNCHING_NOTE,
        /** the transformer is at amplifier position and is standing by */
        AT_AMPLIFIER_POSITION_HOLDING_NOTE,
        /** the transformer is at amplifier position and is splitting the note */
        SPLITTING_TO_AMPLIFIER
    }
    private IntakeAndShooterStatus currentStatus;
    private boolean currentTaskComplete;
    /**
     * initialization of intake and shooter service
     * a transformable mechanism that can intake and move up to shoot
     */
    protected TransformableIntakeAndShooterService(RobotConfigReader robotConfig, Intake intakeModule, Shooter shooterModule, IntakeTransformer transformerModule, XboxController copilotController) {
        super("Intake-And-Shooter-Service");
        this.intakeModule = intakeModule;
        this.shooterModule = shooterModule;
        this.transformerModule = transformerModule;
        this.robotConfig = robotConfig;
        this.copilotController = copilotController;
    }

    @Override
    public void init() {
        reset();
    }

    @Override
    public void reset() {
        this.currentStatus = IntakeAndShooterStatus.AT_DEFAULT_POSITION;
    }

    private void gainOwnerShips() {
        this.intakeModule.clearOwners();
        this.intakeModule.gainOwnerShip(this);
        this.shooterModule.clearOwners();
        this.shooterModule.gainOwnerShip(this);
        this.transformerModule.clearOwners();
        this.transformerModule.gainOwnerShip(this);
    }

    @Override
    public void periodic() {
        /* read the xbox input */
        final boolean
                /** set the current service as the activated  */
                SET_ACTIVATE_BUTTON = copilotController.getBackButton(), // "back" for manual control, "start" for auto control (in vision aided chassis)
                MOVE_TO_GRAB_STANDBY_POSITION_BUTTON = copilotController.getAButton(),
                START_GRAB_BUTTON = copilotController.getRightTriggerAxis() > 0.75,
                STOP_GRAB_BUTTON = copilotController.getRightTriggerAxis() < 0.25,
                START_SPLIT_BUTTON = copilotController.getRightBumper(),
                TOGGLE_AMPLIFIER_BUTTON = copilotController.getLeftBumper(),
                START_SHOOTER_BUTTON = copilotController.getLeftTriggerAxis() > 0.75,
                LAUNCH_BUTTON = copilotController.getLeftTriggerAxis() < 0.25,
                CANCEL_ACTION_BUTTON = copilotController.getXButton();

        switch (currentStatus) {
            // TODO finish this part of the code
            case AT_DEFAULT_POSITION -> {
                // the transformer is at standby position
                break;
            }
            case AT_INTAKE_STANDBY_POSITION -> {
                // the transformer is at intake standby position, which does not hit the floor, waiting for intake command
                break;
            }
            case PROCEEDING_INTAKE -> {
                // the transformer is at intake active position, the intake wheels spin to grab the note
                break;
            }
            case PROCEEDING_SPLIT -> {
                // the transformer is at intake standby position, and the intake is splitting the note out
                break;
            }
            case AT_SHOOTING_STANDBY_POSITION_HOLDING_NOTE -> {
                // the transformer is at the shooting position, standing by for further instruction
                break;
            }
            case LAUNCHING_NOTE -> {
                // the transformer is at the shooting position and the note is being launched
                break;
            }
            case AT_AMPLIFIER_POSITION_HOLDING_NOTE -> {
                // the transformer is at amplifier position and is standing by
                break;
            }
            case SPLITTING_TO_AMPLIFIER -> {
                // the transformer is at amplifier position and is splitting the note
                break;
            }
            default -> {
                // the current status is unknown
                break;
            }
        }
    }

    @Override
    public void onDestroy() {

    }

    /**
     * @return whether the current task is satisfied
     * */
    public boolean isCurrentTaskComplete() {
        return currentTaskComplete;
    }
}
