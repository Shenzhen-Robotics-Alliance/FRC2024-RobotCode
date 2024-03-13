package frc.robot.Services;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Modules.UpperStructure.Climb;
import frc.robot.RobotShell;
import frc.robot.Utils.RobotConfigReader;

public class ClimbService extends RobotServiceBase {
    private final XboxController copilotGamePad;
    private final Climb climb;
    private final RobotConfigReader robotConfig;

    public ClimbService(XboxController copilotGamePad, Climb climb, RobotConfigReader robotConfig) {
        super("climb");
        this.copilotGamePad = copilotGamePad;
        this.climb = climb;
        this.robotConfig = robotConfig;
    }

    @Override
    public void init() {
        reset();
    }

    @Override
    public void reset() {
        climb.gainOwnerShip(this);
    }

    @Override
    public void periodic() {
        climb.setLeftClimbTask(
                copilotGamePad.getAButton() ?
                new Climb.ClimbTask(Climb.ClimbTask.TaskType.SET_POSITION, robotConfig.getConfig("climb", "upMostPointEncoderPosition"))
                        : new Climb.ClimbTask(Climb.ClimbTask.TaskType.SET_POWER, copilotGamePad.getLeftTriggerAxis())
                , this);

        climb.setRightClimbTask(
                copilotGamePad.getAButton() ?
                        new Climb.ClimbTask(Climb.ClimbTask.TaskType.SET_POSITION, robotConfig.getConfig("climb", "upMostPointEncoderPosition"))
                        : new Climb.ClimbTask(Climb.ClimbTask.TaskType.SET_POWER, copilotGamePad.getRightTriggerAxis())
                , this);
    }

    @Override
    public void onDestroy() {

    }
}
