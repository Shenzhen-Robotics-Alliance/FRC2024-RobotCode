package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.MathUtils.BezierCurve;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;
import frc.robot.Utils.MathUtils.Vector2D;

import java.util.ArrayList;
import java.util.List;

public class TestAutoStageProgram implements AutoStageProgram {
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore);

        commandSegments.add(commandFactory.moveFromPointToPointAndStop(
                new Vector2D(),
                new Vector2D(new double[] {0, 3}),
                new Vector2D(new double[] {2,3}))
        );
        return commandSegments;
    }
}
