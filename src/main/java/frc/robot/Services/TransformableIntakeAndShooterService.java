package frc.robot.Services;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Modules.UpperStructure.Intake;
import frc.robot.Modules.UpperStructure.IntakeTransformer;
import frc.robot.Modules.UpperStructure.Shooter;
import frc.robot.Utils.RobotConfigReader;
import org.cef.network.CefURLRequest;

public class TransformableIntakeAndShooterService extends RobotServiceBase {
    private final Intake intakeModule;
    private final Shooter shooterModule;
    private final IntakeTransformer transformerModule;
    private final RobotConfigReader robotConfig;
    private final XboxController copilotController;

    public enum IntakeAndShooterTask {
        SET_AWAIT_INTAKE,
        GRAB_NOTE,
        SET_AWAIT_SHOOT,
        SHOOT_NOTE
    }
    private IntakeAndShooterTask currentTask;
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
        this.intakeModule.gainOwnerShip(this);
        this.shooterModule.gainOwnerShip(this);
        this.transformerModule.gainOwnerShip(this);

        currentTask = IntakeAndShooterTask.SET_AWAIT_INTAKE;
    }

    @Override
    public void periodic() {

    }

    @Override
    public void onDestroy() {

    }
}
