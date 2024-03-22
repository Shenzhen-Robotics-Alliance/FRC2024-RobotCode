package frc.robot.Utils.Tests;

import frc.robot.AutoStagePrograms.FourNoteLower;
import frc.robot.RobotCore;
import frc.robot.Utils.CommandSequenceGenerator;

public class AutoInitializationCheck implements SimpleRobotTest {
    private final CommandSequenceGenerator autoStage = new FourNoteLower();
    @Override
    public void testStart() {
        autoStage.getCommandSegments(new RobotCore("6706-2024"));
    }

    @Override
    public void testPeriodic() {

    }
}
