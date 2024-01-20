package frc.robot.AutoStagePrograms;

import frc.robot.Modules.SwerveBasedChassis;
import frc.robot.Robot;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.List;

/**
 * write auto stages in subclasses that implements this abstract class
 * */
public interface AutoStageProgram {
    List<SequentialCommandSegment> getCommandSegments(Robot robot);
}
