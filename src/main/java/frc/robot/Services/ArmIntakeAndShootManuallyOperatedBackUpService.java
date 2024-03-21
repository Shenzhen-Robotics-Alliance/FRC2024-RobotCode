package frc.robot.Services;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Modules.UpperStructure.Intake;
import frc.robot.Modules.UpperStructure.Shooter;
import frc.robot.Modules.UpperStructure.TransformableArm;
import frc.robot.Utils.RobotConfigReader;

public class ArmIntakeAndShootManuallyOperatedBackUpService extends RobotServiceBase {

    private final Intake intakeModule;
    private final Shooter shooterModule;
    private final TransformableArm transformerModule;
    private final RobotConfigReader robotConfig;
    private final XboxController copilotController;

    public ArmIntakeAndShootManuallyOperatedBackUpService(Intake intakeModule, Shooter shooterModule, TransformableArm transformerModule, RobotConfigReader robotConfig, XboxController copilotController) {
        super("Arm Intake and Shoot Manually Operated BackUp Service");
        this.intakeModule = intakeModule;
        this.shooterModule = shooterModule;
        this.transformerModule = transformerModule;
        this.robotConfig = robotConfig;
        this.copilotController = copilotController;
    }

    @Override
    public void init() {

    }

    @Override
    public void reset() {

    }

    /**
     * logic:
     * (gaining control)
     * when anything malfunctions, this module gains ownership to upper structures automatically; It will also do of copilot presses button "back"
     * (split)
     * pressing button b, arm raises to split angle, intake splits whenever arm is ready
     * (intake process)
     * pressing right bumper, arm moves to intake angle and intake spins at intake speed to grab note, intake will NOT stop automatically when note is inside
     * so, after the note gets inside, the copilot pulls the right stick down to make the intake move reversely, notice it moves very slowly
     * (scoring process)
     * By pulling the left stick down, the arm raises to shoot angle, and the shooter accelerates (if aiming system is available, they will be automatically adjusted according to distance as usual)
     * By pulling the right stick up, the arm raise to amplify angle, and the shooter accelerates to amplifying RPM
     * Notice the intake will not push the note towards fly wheels in the both situations above, it will instead remain in still. To move it, the copilot moves the right stick up, this time the intake moves up and in a faster speed.
     * */
    @Override
    public void periodic() {
        if (copilotController.getBackButton() || intakeModule.malFunctioning() || transformerModule.malFunctioning()) gainOwnerShipsToUpperStructure();
        if (copilotController.getLeftStickButton() && copilotController.getRightStickButton()) intakeModule.reset();

        /* normal ops */
        final double leftStick = -copilotController.getLeftY(), rightStick = -copilotController.getRightY();
        if (copilotController.getRightBumper()) {
            transformerModule.setTransformerDesiredPosition(TransformableArm.TransformerPosition.INTAKE, this);
            shooterModule.setShooterMode(Shooter.ShooterMode.DISABLED, this);
            if (transformerModule.transformerInPosition())
                intakeModule.startIntake(this);
        } else if (leftStick > 0.2) {
            transformerModule.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SCORE_AMPLIFIER, this);
            shooterModule.setShooterMode(Shooter.ShooterMode.AMPLIFY, this);
        } else if (leftStick < -0.2) {
            transformerModule.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SHOOT_NOTE, this);
            shooterModule.setShooterMode(Shooter.ShooterMode.SHOOT, this);
        }  else {
            transformerModule.setTransformerDesiredPosition(TransformableArm.TransformerPosition.DEFAULT, this);
            shooterModule.setShooterMode(Shooter.ShooterMode.DISABLED, this);
            intakeModule.turnOffIntake(this);
        }

        if (rightStick > 0.05)
            intakeModule.specifyPower(rightStick * robotConfig.getConfig("intake", "moveNoteUpInsideIntakePower"), this);
        else if (rightStick < -0.05)
            intakeModule.specifyPower(rightStick * robotConfig.getConfig("intake", "moveNoteDownInsideIntakePower"), this);


        /* in case of stuck */
        if (copilotController.getYButton()) {
            shooterModule.setDesiredSpeed(copilotController.getBButton() ? -6000: 6000, this);
            shooterModule.setShooterMode(Shooter.ShooterMode.SPECIFIED_RPM, this);
        }
        if (copilotController.getBButton()) {
            transformerModule.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SPLIT, this);
            transformerModule.periodic();
            if (transformerModule.transformerInPosition())
                intakeModule.startSplit(this); // in case if the Note is stuck
        }
    }

    @Override
    public void onDestroy() {

    }

    private void gainOwnerShipsToUpperStructure() {
        intakeModule.gainOwnerShip(this);
        transformerModule.gainOwnerShip(this);
        shooterModule.gainOwnerShip(this);

        System.out.println("<-- WARNING!!! distance sensor malfunction detected, switching to manual control -->");
    }

    private void setGamePadStatus() {

    }
}