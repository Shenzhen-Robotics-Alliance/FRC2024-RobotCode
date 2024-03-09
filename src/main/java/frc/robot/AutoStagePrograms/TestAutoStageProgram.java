package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.CommandSequenceGenerator;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;
import frc.robot.Utils.MathUtils.Vector2D;

import java.util.ArrayList;
import java.util.List;

public class TestAutoStageProgram implements CommandSequenceGenerator {
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, new Vector2D(new double[] {0, 0.4}), new Rotation2D(0));

        commandSegments.add(commandFactory.moveFromPointToPoint(new Vector2D(new double[] {0, 0.4}), new Vector2D(new double[] {0, 3}), new Rotation2D(0), new Rotation2D(Math.toDegrees(90))));

//        commandSegments.add(new SequentialCommandSegment(
//                () -> true,
//                () -> commandFactory.getBezierCurvesFromPathFile("test path").get(0),
//                ()->{}, ()->{}, ()->{},
//                () -> true,
//                () -> new Rotation2D(Math.toRadians(0)),
//                () -> new Rotation2D(Math.toRadians(0))
//        ));
//
//        commandSegments.add(new SequentialCommandSegment(
//                () -> true,
//                () -> commandFactory.getBezierCurvesFromPathFile("test path").get(1),
//                ()->{}, ()->{}, ()->{},
//                () -> true,
//                () -> new Rotation2D(Math.toRadians(0)),
//                () -> new Rotation2D(Math.toRadians(90))
//        ));
        return commandSegments;
    }
}
