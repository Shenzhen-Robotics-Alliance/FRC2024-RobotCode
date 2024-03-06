package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.CommandSequenceGenerator;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.List;

public class HardCodedLeaveCommunityRight implements CommandSequenceGenerator {
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, new Vector2D(new double[] {0, 0.4}), new Rotation2D(Math.toRadians(-90)));

        commandSegments.add(commandFactory.moveFromPointToMidPointToPoint(
                new Vector2D(new double[] {0, 0}),
                new Vector2D(new double[] {0.5, 0}),
                new Vector2D(new double[] {0.5, 2})
        ));
        return commandSegments;
    }
}