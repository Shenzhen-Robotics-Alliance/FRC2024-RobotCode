package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.CommandSequenceGenerator;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class FiveNotesUpperTestRoute implements CommandSequenceGenerator {
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, SequentialCommandFactory.getRobotStartingPosition("shoot first move to second upper"), new Rotation2D(Math.toRadians(90)));

        commandSegments.add(commandFactory.calibratePositionEstimator());

        /* shoot first move to second */
        commandSegments.addAll(Arrays.asList(commandFactory.followPath(
                "shoot first move to second upper",
                new Rotation2D[] {
                        new Rotation2D(Math.toRadians(170)),
                        new Rotation2D(Math.toRadians(180)),
                        new Rotation2D(Math.toRadians(180))
                },
                ()->{}, ()->{}, ()->{}
        )));

        /* shoot second move to third */
        commandSegments.addAll(Arrays.asList(commandFactory.followPath(
                "shoot second move to third upper",
                new Rotation2D[] {
                        new Rotation2D(Math.toRadians(160)),
                        new Rotation2D(Math.toRadians(160)),
                        new Rotation2D(Math.toRadians(-160))
                },
                ()->{}, ()->{}, ()->{}
        )));

        /* shoot third move to fourth */
        commandSegments.addAll(Arrays.asList(commandFactory.followPath(
                "shoot third move to fourth upper",
                new Rotation2D[] {
                        new Rotation2D(Math.toRadians(150)),
                        new Rotation2D(Math.toRadians(180)),
                        new Rotation2D(Math.toRadians(180))
                },
                ()->{}, ()->{}, ()->{}
        )));

        /* shoot fourth move to fifth */
        commandSegments.addAll(Arrays.asList(commandFactory.followPath(
                "shoot fourth move to fifth upper",
                new Rotation2D[] {
                        new Rotation2D(Math.toRadians(-160)),
                        new Rotation2D(Math.toRadians(-150)),
                        new Rotation2D(Math.toRadians(180))
                },
                ()->{}, ()->{}, ()->{}
        )));

        /* shoot fifth */
        commandSegments.addAll(Arrays.asList(commandFactory.followPathFacing(
                "shoot fifth upper",
                new Rotation2D(Math.toRadians(180))
        )));

        return commandSegments;
    }
}
