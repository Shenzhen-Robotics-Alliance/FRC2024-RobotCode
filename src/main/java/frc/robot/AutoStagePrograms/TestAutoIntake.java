package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.ComputerVisionUtils.AutoStageVisionAimBot;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.List;

public class TestAutoIntake implements AutoStageProgram {
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, new Vector2D(new double[] {0, 0}), new Rotation2D(Math.toRadians(180)));
        final AutoStageVisionAimBot aimBot = new AutoStageVisionAimBot(robotCore, 2000);
        commandSegments.add(commandFactory.calibratePositionEstimator());

        commandSegments.add(commandFactory.moveFromPointToPointAndStop(
                new Vector2D(new double[] {0, 0}),
                new Vector2D(new double[] {0, 1}),
                new Rotation2D(Math.toRadians(180)),
                new Rotation2D(Math.toRadians(180))
        ));

        commandSegments.add(aimBot.grabNote(
                () -> true,
                new Vector2D(new double[] {0, 1.3}),
                new Rotation2D(Math.toRadians(180)),
                2000
                ));
        return commandSegments;
    }
}
