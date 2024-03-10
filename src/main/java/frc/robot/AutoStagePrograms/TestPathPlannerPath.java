package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.CommandSequenceGenerator;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.SpeedCurves;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.List;

public class TestPathPlannerPath implements CommandSequenceGenerator {
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, "test path", new Rotation2D(0));
        commandSegments.add(commandFactory.calibratePositionEstimator());
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> SequentialCommandFactory.getBezierCurvesFromPathFile("test path").get(0),
                ()->{}, ()->{}, ()->{},
                () -> true,
                () -> new Rotation2D(Math.toRadians(0)),
                () -> new Rotation2D(Math.toRadians(0)),
                SpeedCurves.easeIn,
                1
        ));

        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> SequentialCommandFactory.getBezierCurvesFromPathFile("test path").get(1),
                ()->{}, ()->{}, ()->{},
                () -> true,
                () -> new Rotation2D(Math.toRadians(0)),
                () -> new Rotation2D(Math.toRadians(90)),
                SpeedCurves.easeOut,
                1
        ));
        return commandSegments;
    }
}
