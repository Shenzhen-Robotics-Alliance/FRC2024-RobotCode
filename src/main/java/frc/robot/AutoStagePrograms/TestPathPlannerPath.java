package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.CommandSequenceGenerator;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.SpeedCurves;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TestPathPlannerPath implements CommandSequenceGenerator {
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, "test path", new Rotation2D(0));
        commandSegments.add(commandFactory.calibratePositionEstimator());

        commandSegments.addAll(Arrays.asList(commandFactory.followPathFacing(
                "test path",
                new Rotation2D(Math.toRadians(90)),
                () -> System.out.println("<-- beginning -->"),
                () -> System.out.println("<-- periodic -->"),
                () -> System.out.println("<-- ending -->")
        )));
        return commandSegments;
    }
}
