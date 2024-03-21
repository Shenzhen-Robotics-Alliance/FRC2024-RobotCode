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

    protected ArmIntakeAndShootManuallyOperatedBackUpService(Intake intakeModule, Shooter shooterModule, TransformableArm transformerModule, RobotConfigReader robotConfig, XboxController copilotController) {
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

    @Override
    public void periodic() {
        if (copilotController.getBackButton() || intakeModule.malFunctioning())
            gainOwnerShipsToUpperStructure();

    }

    @Override
    public void onDestroy() {

    }

    private void gainOwnerShipsToUpperStructure() {
        intakeModule.gainOwnerShip(this);
        transformerModule.gainOwnerShip(this);
        shooterModule.gainOwnerShip(this);

        // TODO rumble gamepads
        // copilotController.setRumble(GenericHID.RumbleType.kBothRumble, );
    }

    private void setGamePadStatus() {

    }
}