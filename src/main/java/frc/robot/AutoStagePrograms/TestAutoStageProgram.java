package frc.robot.AutoStagePrograms;

import frc.robot.Robot;
import frc.robot.Utils.MathUtils.BezierCurve;
import frc.robot.Utils.SequentialCommandSegment;
import frc.robot.Utils.MathUtils.Vector2D;

import java.util.ArrayList;
import java.util.List;

public class TestAutoStageProgram implements AutoStageProgram {
    @Override
    public List<SequentialCommandSegment> getCommandSegments(Robot robot) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();

        BezierCurve path = new BezierCurve(new Vector2D(), new Vector2D(new double[] {0, 3}), new Vector2D(new double[] {2, 3}));
        commandSegments.add(new SequentialCommandSegment(
                path,
                () -> {},
                () -> {},
                () -> {},
                () -> true,
                0, Math.toRadians(0)
                ));
        return commandSegments;
    }
}
