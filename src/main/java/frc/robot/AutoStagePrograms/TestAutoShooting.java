package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.ComputerVisionUtils.AutoStageVisionAimBot;
import frc.robot.Utils.MathUtils.BezierCurve;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.List;

public class TestAutoShooting implements AutoStageProgram {
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, new Vector2D(new double[] {0, 0}), new Rotation2D(Math.toRadians(-90)));
        final AutoStageVisionAimBot aimBot = new AutoStageVisionAimBot(robotCore, 2000);
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();

        commandSegments.add(commandFactory.calibratePositionEstimator());
        commandSegments.add(commandFactory.moveFromPointToMidPointToPoint(
                new Vector2D(new double[] {0, 0}),
                new Vector2D(new double[] {0.2, 2.7}),
                new Vector2D(new double[] {0.4, 2.7-0.4})
        ));
        commandSegments.add(aimBot.shootWhileMoving(
                new BezierCurve(new Vector2D(new double[] {0.4, 2.7-0.4}), new Vector2D(new double[] {1.2, 2.5-0.8}), new Vector2D(new double[] {1.7, 2.5})),
                new Vector2D(new double[] {-0.5, -0.8}),
                4000
        ));
        return commandSegments;
    }
}
