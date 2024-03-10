package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.CommandSequenceGenerator;
import frc.robot.Utils.ComputerVisionUtils.AutoStageVisionAimBot;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class RedAutoStageSixNote implements CommandSequenceGenerator {
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, SequentialCommandFactory.getRobotStartingPosition("first note and grab"), new Rotation2D(Math.toRadians(180)));
        final AutoStageVisionAimBot aimBot = new AutoStageVisionAimBot(robotCore, 6000);

        commandSegments.add(commandFactory.calibratePositionEstimator());

        commandSegments.addAll(Arrays.asList(commandFactory.followPathFacing(
                "first note and grab",
                new Rotation2D(Math.toRadians(180))
        )));

        commandSegments.addAll(Arrays.asList(commandFactory.followPathFacing(
                "shoot second and grab third",
                new Rotation2D(Math.toRadians(-135))
        )));

        commandSegments.addAll(Arrays.asList(commandFactory.followPathFacing(
                "shoot third grab fourth",
                new Rotation2D(Math.toRadians(135))
        )));

        commandSegments.addAll(Arrays.asList(commandFactory.followPathFacing(
                "shoot fourth move to fifth",
                new Rotation2D(Math.toRadians(180))
        )));

        commandSegments.addAll(Arrays.asList(commandFactory.followPath(
                "shoot fifth grab sixth",
                new Rotation2D[] {
                        new Rotation2D(Math.toRadians(180)),
                        new Rotation2D(Math.toRadians(180)),
                        new Rotation2D(Math.toRadians(135)),
                        new Rotation2D(Math.toRadians(135))
                },
                () -> {}, () -> {}, () -> {}
        )));

        commandSegments.addAll(Arrays.asList(commandFactory.followPath(
                "shoot sixth",
                new Rotation2D[] {
                        new Rotation2D(Math.toRadians(135)),
                        new Rotation2D(Math.toRadians(180))
                },
                () -> {}, () -> {}, () -> {}
        )));

        return commandSegments;
    }
}
