package frc.robot.Services;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Modules.UpperStructure.Intake;
import frc.robot.Modules.UpperStructure.TransformableArm;
import frc.robot.Modules.UpperStructure.Shooter;
import frc.robot.Utils.RobotConfigReader;

/**
 * TODO communicate with vision-aided chassis
 * */
public class TransformableIntakeAndShooterService extends RobotServiceBase {
    private final Intake intakeModule;
    private final Shooter shooterModule;
    private final TransformableArm transformerModule;
    private final RobotConfigReader robotConfig;
    private final XboxController copilotController;

    public enum IntakeAndShooterStatus {
        /** the transformer is at standby position */
        AT_DEFAULT_POSITION,
        /** the transformer is at standby position, and holding a note */
        AT_DEFAULT_POSITION_HOLDING_NOTE,
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
    protected TransformableIntakeAndShooterService(RobotConfigReader robotConfig, Intake intakeModule, Shooter shooterModule, TransformableArm transformerModule, XboxController copilotController) {
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
                CANCEL_GRAB_BUTTON = copilotController.getRightTriggerAxis() < 0.25,
                START_SPLIT_BUTTON = copilotController.getRightBumper(),
                TOGGLE_AMPLIFIER_BUTTON = copilotController.getLeftBumper(),
                START_SHOOTER_BUTTON = copilotController.getLeftTriggerAxis() > 0.75,
                LAUNCH_BUTTON = copilotController.getLeftTriggerAxis() < 0.25,
                CANCEL_ACTION_BUTTON = copilotController.getXButton();

        switch (currentStatus) {
            case AT_DEFAULT_POSITION -> {
                /* the transformer is at standby position not holding a note */
                transformerModule.setTransformerDesiredPosition(TransformableArm.TransformerPosition.DEFAULT, this);
                intakeModule.turnOffIntake(this);
                if (START_GRAB_BUTTON)
                    startIntakeProcess();
                else if (MOVE_TO_GRAB_STANDBY_POSITION_BUTTON)
                    this.currentStatus = IntakeAndShooterStatus.AT_INTAKE_STANDBY_POSITION;
            }
            case AT_DEFAULT_POSITION_HOLDING_NOTE -> {
                /* the transformer is at standby position, and holding a note */
                transformerModule.setTransformerDesiredPosition(TransformableArm.TransformerPosition.DEFAULT, this);
                if (!intakeModule.isNoteInsideIntake())
                    this.currentStatus = IntakeAndShooterStatus.AT_DEFAULT_POSITION;
                if (START_SPLIT_BUTTON)
                    startSplitProcess();
            }
            case AT_INTAKE_STANDBY_POSITION -> {
                /* the transformer is at intake standby position, which does not hit the floor, waiting for intake command */
                transformerModule.setTransformerDesiredPosition(TransformableArm.TransformerPosition.INTAKE_STANDBY, this);
                intakeModule.turnOffIntake(this);
                if (START_GRAB_BUTTON)
                    startIntakeProcess();
            }
            case PROCEEDING_INTAKE -> {
                /* the transformer is at intake active position, the intake is already set to be spinning, so that they can grab the note */
                transformerModule.setTransformerDesiredPosition(TransformableArm.TransformerPosition.INTAKE, this);
                if (CANCEL_GRAB_BUTTON)
                    cancelIntakeProcess();
                if (intakeModule.isCurrentTaskComplete())
                    this.currentStatus = IntakeAndShooterStatus.AT_DEFAULT_POSITION_HOLDING_NOTE;
            }
            case PROCEEDING_SPLIT -> {
                /*
                * the transformer is at intake standby position, and the intake is already set to be splitting
                * the intake module will just split for 0.5s so we just wait for it to finish
                * */
                transformerModule.setTransformerDesiredPosition(TransformableArm.TransformerPosition.INTAKE_STANDBY, this);
                if (intakeModule.isCurrentTaskComplete())
                    this.currentStatus = IntakeAndShooterStatus.AT_INTAKE_STANDBY_POSITION;
            }
            case AT_SHOOTING_STANDBY_POSITION_HOLDING_NOTE -> {
                /*
                * the transformer is at the shooting position, standing by for further instruction
                * TODO
                *  1. aiming logic should go here, pass the arm position calculated by the aiming system to the arm module
                *  2. maybe also request the chassis to face to the target
                *  3. shooter module should be set as activated, so that it automatically obtains aiming data and update its shooter speed accordingly
                *  */

                transformerModule.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SHOOT_NOTE, this);
                if (START_SPLIT_BUTTON)
                    launchProcessFailed();
                if (START_SHOOTER_BUTTON && shooterModule.shooterReady() && transformerModule.transformerInPosition())
                    startLaunchProcess();
            }
            case LAUNCHING_NOTE -> {
                /* the transformer is at the shooting position and the note is kicker are spinning to launch the note */

                transformerModule.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SHOOT_NOTE, this);
                if (START_SPLIT_BUTTON)
                    launchProcessFailed();
                if (intakeModule.isCurrentTaskComplete())
                    launchProcessSucceeded();
            }
            case AT_AMPLIFIER_POSITION_HOLDING_NOTE -> {
                /* the transformer is at amplifier position and is standing by */
                transformerModule.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SCORE_AMPLIFIER, this);
                if (START_SHOOTER_BUTTON)
                    startAmplifyProcess();

            }
            case SPLITTING_TO_AMPLIFIER -> {
                /* the transformer is at amplifier position and is splitting the note */
                transformerModule.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SCORE_AMPLIFIER, this);

                if (START_SPLIT_BUTTON)
                    launchProcessFailed();
                if (intakeModule.isCurrentTaskComplete())
                    this.currentStatus = IntakeAndShooterStatus.AT_DEFAULT_POSITION;
            }
            default -> throw new IllegalStateException("unknown status for intake and shooter:" + currentStatus);
        }
        System.out.println("intake and shooter service current status: " + currentStatus);
    }

    private void startIntakeProcess() {
        intakeModule.startIntake(this);
        this.currentStatus = IntakeAndShooterStatus.PROCEEDING_INTAKE;
    }

    private void startSplitProcess() {
        intakeModule.startSplit(this);
        this.currentStatus = IntakeAndShooterStatus.PROCEEDING_SPLIT;
    }

    private void cancelIntakeProcess() {
        intakeModule.turnOffIntake(this);
        this.currentStatus = IntakeAndShooterStatus.AT_INTAKE_STANDBY_POSITION;
    }

    private void startLaunchProcess() {
        intakeModule.startLaunch(this);
        this.currentStatus = IntakeAndShooterStatus.LAUNCHING_NOTE;
    }

    private void launchProcessSucceeded() {
        shooterModule.setDesiredSpeed(0);
        this.currentStatus = IntakeAndShooterStatus.AT_DEFAULT_POSITION;
    }

    /** in case the shooter gets stuck */
    private void launchProcessFailed() {
        shooterModule.setDesiredSpeed(0);
        startSplitProcess();
    }

    private void startAmplifyProcess() {
        shooterModule.setDesiredSpeed(2000); // TODO find this value in robot config as "amplifier scoring shooter speed"
        intakeModule.startSplit(this);
        this.currentStatus = IntakeAndShooterStatus.SPLITTING_TO_AMPLIFIER;
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
