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
        climb.onReset();
    }

    @Override
    public void periodic() {
        climb.setLeftClimbTask(new Climb.ClimbTask(Climb.ClimbTask.TaskType.SET_POWER, (copilotGamePad.getAButton() ? -1:1) * copilotGamePad.getLeftTriggerAxis()), this);

        climb.setRightClimbTask(new Climb.ClimbTask(Climb.ClimbTask.TaskType.SET_POWER, (copilotGamePad.getAButton() ? -1:1) * copilotGamePad.getRightTriggerAxis()), this);
    }

    @Override
    public void onDestroy() {

    }
}
