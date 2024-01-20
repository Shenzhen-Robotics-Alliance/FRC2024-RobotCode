package frc.robot;

import frc.robot.AutoStagePrograms.AutoStageProgram;
import frc.robot.Modules.SwerveBasedChassis;
import frc.robot.Services.AutoProgramRunner;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.List;

public class AutonomousRobot extends Robot {
    private final AutoProgramRunner autoProgramRunner;
    public AutonomousRobot(AutoStageProgram autoStageProgram) {
        super();

        final List<SequentialCommandSegment> commandSegments = autoStageProgram.getCommandSegments(this);
        autoProgramRunner = new AutoProgramRunner(commandSegments, super.chassisModule, super.robotConfig);
        super.services.add(autoProgramRunner);
    }
}
