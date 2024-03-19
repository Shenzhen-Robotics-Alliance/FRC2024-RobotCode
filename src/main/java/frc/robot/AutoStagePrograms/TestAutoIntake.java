package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.CommandSequenceGenerator;
import frc.robot.Utils.ComputerVisionUtils.AutoStageVisionAimBot;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.List;

public class TestAutoIntake implements CommandSequenceGenerator {
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, new Vector2D(new double[]{0, 0}), new Rotation2D(Math.toRadians(0)));
        final AutoStageVisionAimBot aimBot = new AutoStageVisionAimBot(robotCore, 2000);
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();

        commandSegments.add(commandFactory.calibratePositionEstimator());
        commandSegments.add(aimBot.grabNote(new Vector2D(new double[] {0, -0.5}), new Rotation2D(0), 5000));

        return commandSegments;
    }
}