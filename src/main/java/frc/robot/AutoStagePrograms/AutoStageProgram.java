package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.List;

/**
 * write auto stages in subclasses that implements this abstract class
 * */
public interface AutoStageProgram {
    List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore);
}
