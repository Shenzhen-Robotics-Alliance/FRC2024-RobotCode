package frc.robot.Modules.UpperStructure;

import frc.robot.Modules.RobotModuleBase;
import frc.robot.RobotShell;
import frc.robot.Services.RobotServiceBase;

public class IntakeTransformer extends RobotModuleBase {
    public enum TransformerMode {
        INTAKE,
        INTAKE_STANDBY,
        SHOOT,
        SCORE_AMPLIFIER
    }
    private TransformerMode currentMode;
    private static final TransformerMode defaultMode = TransformerMode.INTAKE; // on startup, we are at intake position

    private boolean transformerInPosition;

    /*
    * TODO we do not know yet , what the intake transformer is made of, so we first construct the APIs
    * */
    public IntakeTransformer() {
        super("Transformable-Intake");
    }

    @Override
    public void init() {
        this.resetModule();
    }

    @Override
    protected void periodic(double dt) {

    }

    @Override
    public void resetModule() {
        this.currentMode = defaultMode;
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

    public void setTransformerStatus(TransformerMode desiredMode, RobotServiceBase operatorService) {
        if (!isOwner(operatorService))
            return;

        this.currentMode = desiredMode;
    }

    public boolean transformerInPosition() {
        return this.transformerInPosition;
    }
}
