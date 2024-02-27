package frc.robot.Services;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Modules.UpperStructure.Intake;
import frc.robot.Modules.UpperStructure.TransformableArm;
import frc.robot.Modules.UpperStructure.Shooter;
import frc.robot.Utils.RobotConfigReader;

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
        AMPLIFYING,
        ACTION_CANCELLED
    }
    private IntakeAndShooterStatus currentStatus;
    private boolean currentTaskComplete;
    /**
     * initialization of intake and shooter service
     * a transformable mechanism that can intake and move up to shoot
     */
    public TransformableIntakeAndShooterService(Intake intakeModule, Shooter shooterModule, TransformableArm transformerModule, RobotConfigReader robotConfig, XboxController copilotController) {
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
        gainOwnerShipsToModules(); // TODO sendable chooser to choose between this and vision chassis
    }

    private void gainOwnerShipsToModules() {
        this.intakeModule.gainOwnerShip(this);
        this.shooterModule.gainOwnerShip(this);
        this.transformerModule.gainOwnerShip(this);
    }

    @Override
    public void periodic() {
        /* read the xbox input */
        final boolean
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
                shooterModule.setShooterMode(Shooter.ShooterMode.DISABLED, this);
                if (START_GRAB_BUTTON)
                    startIntakeProcess();
                else if (START_SPLIT_BUTTON)
                    startSplitProcess();
            }
            case AT_DEFAULT_POSITION_HOLDING_NOTE -> {
                /* the transformer is at standby position, and holding a note */
                shooterModule.setShooterMode(Shooter.ShooterMode.DISABLED, this);
                transformerModule.setTransformerDesiredPosition(TransformableArm.TransformerPosition.DEFAULT, this);
                if (!intakeModule.isNoteInsideIntake())
                    this.currentStatus = IntakeAndShooterStatus.AT_DEFAULT_POSITION;
                else if (START_SHOOTER_BUTTON && !CANCEL_ACTION_BUTTON)
                    this.currentStatus = IntakeAndShooterStatus.AT_SHOOTING_STANDBY_POSITION_HOLDING_NOTE;
                else if (TOGGLE_AMPLIFIER_BUTTON && !CANCEL_ACTION_BUTTON)
                    this.currentStatus = IntakeAndShooterStatus.AT_AMPLIFIER_POSITION_HOLDING_NOTE;
                else if (START_SPLIT_BUTTON)
                    startSplitProcess();
            }
            case PROCEEDING_INTAKE -> {
                /* the transformer is at intake active position, the intake is already set to be spinning, so that they can grab the note */
                shooterModule.setShooterMode(Shooter.ShooterMode.DISABLED, this);
                transformerModule.setTransformerDesiredPosition(TransformableArm.TransformerPosition.INTAKE, this);
                if (CANCEL_GRAB_BUTTON)
                    cancelIntakeProcess();
                else if (intakeModule.isNoteInsideIntake())
                    this.currentStatus = IntakeAndShooterStatus.AT_DEFAULT_POSITION_HOLDING_NOTE;
            }
            case PROCEEDING_SPLIT -> {
                /*
                * the transformer is at intake standby position, and the intake is already set to be splitting
                * the intake module will just split for 0.5s so we just wait for it to finish
                * */
                shooterModule.setShooterMode(Shooter.ShooterMode.DISABLED, this);
                transformerModule.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SPLIT, this);
                if (intakeModule.isCurrentTaskComplete())
                    this.currentStatus = IntakeAndShooterStatus.AT_DEFAULT_POSITION;
            }
            case AT_SHOOTING_STANDBY_POSITION_HOLDING_NOTE -> {
                transformerModule.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SHOOT_NOTE, this);
                shooterModule.setShooterMode(Shooter.ShooterMode.SHOOT, this);
                if (START_SPLIT_BUTTON)
                    launchProcessFailed();
                else if (CANCEL_ACTION_BUTTON)
                    launchProcessCancelled();
                else if (LAUNCH_BUTTON && shooterModule.shooterReady() && transformerModule.transformerInPosition())
                    startLaunchProcess();
            }
            case LAUNCHING_NOTE -> {
                /* the transformer is at the shooting position and the note is kicker are spinning to launch the note */
                shooterModule.setShooterMode(Shooter.ShooterMode.SHOOT, this);
                transformerModule.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SHOOT_NOTE, this);
                if (START_SPLIT_BUTTON)
                    launchProcessFailed();
                else if (CANCEL_ACTION_BUTTON)
                    launchProcessCancelled();
                else if (!intakeModule.isNoteInsideIntake())
                    launchProcessSucceeded();
            }
            case AT_AMPLIFIER_POSITION_HOLDING_NOTE -> {
                /* the transformer is at amplifier position and is standing by */
                transformerModule.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SCORE_AMPLIFIER, this);
                shooterModule.setShooterMode(Shooter.ShooterMode.AMPLIFY, this);
                if (!TOGGLE_AMPLIFIER_BUTTON)
                    startAmplifyProcess();
                else if (CANCEL_ACTION_BUTTON)
                    launchProcessCancelled();
            }
            case AMPLIFYING -> {
                /* the transformer is at amplifier position and is splitting the note */
                transformerModule.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SCORE_AMPLIFIER, this);
                shooterModule.setShooterMode(Shooter.ShooterMode.AMPLIFY, this);
                if (START_SPLIT_BUTTON)
                    launchProcessFailed();
                if (!intakeModule.isNoteInsideIntake())
                    this.currentStatus = IntakeAndShooterStatus.AT_DEFAULT_POSITION;
            }
            case ACTION_CANCELLED -> {
                if (!TOGGLE_AMPLIFIER_BUTTON && LAUNCH_BUTTON)
                    this.currentStatus = IntakeAndShooterStatus.AT_DEFAULT_POSITION_HOLDING_NOTE; // wait for the copilot to release the buttons
            }
            default -> throw new IllegalStateException("unknown status for intake and shooter:" + currentStatus);
        }
        // System.out.println("intake and shooter service current status: " + currentStatus);
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
        this.currentStatus = IntakeAndShooterStatus.AT_DEFAULT_POSITION;
    }

    private void startLaunchProcess() {
        intakeModule.startLaunch(this);
        this.currentStatus = IntakeAndShooterStatus.LAUNCHING_NOTE;
    }

    private void launchProcessSucceeded() {
        shooterModule.setShooterMode(Shooter.ShooterMode.DISABLED, this);
        this.currentStatus = IntakeAndShooterStatus.AT_DEFAULT_POSITION;
    }

    /** in case the shooter gets stuck */
    private void launchProcessFailed() {
        shooterModule.setShooterMode(Shooter.ShooterMode.DISABLED, this);
        startSplitProcess();
    }

    private void launchProcessCancelled() {
        shooterModule.setShooterMode(Shooter.ShooterMode.DISABLED, this);
        this.currentStatus = IntakeAndShooterStatus.ACTION_CANCELLED;
    }

    private void startAmplifyProcess() {
        intakeModule.startLaunch(this);
        this.currentStatus = IntakeAndShooterStatus.AMPLIFYING;
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
