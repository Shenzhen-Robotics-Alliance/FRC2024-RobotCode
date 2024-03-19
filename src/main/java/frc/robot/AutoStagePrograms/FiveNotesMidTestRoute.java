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

public class FiveNotesMidTestRoute implements CommandSequenceGenerator {
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, SequentialCommandFactory.getRobotStartingPosition("first note"), new Rotation2D(Math.toRadians(180)));

        commandSegments.add(commandFactory.calibratePositionEstimator());

        /* shoot the preloaded and move to the second */
        commandSegments.addAll(Arrays.asList(commandFactory.followPathFacing(
                "first note",
                new Rotation2D(Math.toRadians(180))
        )));

        commandSegments.addAll(Arrays.asList(commandFactory.followPathFacing(
                "move to second note",
                new Rotation2D(Math.toRadians(135))
        )));

        /* shoot second and move to third */
        commandSegments.addAll(Arrays.asList(commandFactory.followPathFacing(
                "shoot second and grab third",
                new Rotation2D(Math.toRadians(-150))
        )));

        /* shoot third and move to fourth */
        commandSegments.addAll(Arrays.asList(commandFactory.followPath(
                "shoot third grab fourth",
                new Rotation2D[] {
                        new Rotation2D(Math.toRadians(150)),
                        new Rotation2D(Math.toRadians(-150))
                },
                ()->{}, ()->{}, ()->{}
        )));

        /* shoot fourth and move to fifth */
        commandSegments.addAll(Arrays.asList(commandFactory.followPath(
                "shoot fourth move to fifth",
                new Rotation2D[] {
                        new Rotation2D(Math.toRadians(-150)),
                        new Rotation2D(Math.toRadians(-150)),
                        new Rotation2D(Math.toRadians(180))
                },
                ()->{}, ()->{}, ()->{}
        )));

        /* shoot fifth move to sixth */
        commandSegments.addAll(Arrays.asList(commandFactory.followPath(
                "shoot fifth grab sixth",
                new Rotation2D[] {
                        new Rotation2D(Math.toRadians(150)),
                        new Rotation2D(Math.toRadians(-120))
                },
                ()->{}, ()->{}, ()->{}
        )));

        commandSegments.addAll(Arrays.asList(commandFactory.followPathFacing(
                "move closer to speaker holding sixth",
                new Rotation2D(Math.toRadians(-135))
        )));

        return commandSegments;
    }
}

