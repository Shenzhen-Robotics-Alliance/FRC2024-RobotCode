package frc.robot.Modules.UpperStructure;

import frc.robot.Modules.RobotModuleBase;
import frc.robot.Services.RobotServiceBase;

public class IntakeTransformer extends RobotModuleBase {
    public enum TransformerPosition {
        INTAKE,
        STANDBY,
        SHOOT,
        SCORE_AMPLIFIER
    }
    private TransformerPosition desiredPosition;
    private static final TransformerPosition defaultMode = TransformerPosition.INTAKE; // on startup, we are at intake position

    private boolean transformerInPosition;

    /*
    * TODO we do not know yet , what the intake transformer is made of, so we first construct the APIs
    *  today we have to finish coding it
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
        this.desiredPosition = defaultMode;
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

    public void setTransformerDesiredPosition(TransformerPosition desiredMode, RobotServiceBase operatorService) {
        if (!isOwner(operatorService))
            return;

        this.desiredPosition = desiredMode;
    }

    public boolean transformerInPosition() {
        return this.transformerInPosition;
    }
}
