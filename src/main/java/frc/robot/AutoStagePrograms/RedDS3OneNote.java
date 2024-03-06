package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.CommandSequenceGenerator;
import frc.robot.Utils.ComputerVisionUtils.AutoStageVisionAimBot;
import frc.robot.Utils.MathUtils.BezierCurve;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.List;

public class RedDS3OneNote implements CommandSequenceGenerator {
    public static final Vector2D startingPosition = new Vector2D(new double[] {2.9, 0.3}),
            position1 = new Vector2D(new double[] {2.9, 1.8}),
            assumedSpeakerPosition = new Vector2D(new double[] {1.45, 0});
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, startingPosition, new Rotation2D(Math.toRadians(90)));
        final AutoStageVisionAimBot aimBot = new AutoStageVisionAimBot(robotCore, 6000);

        commandSegments.add(commandFactory.calibratePositionEstimator());

        /* shoot the first note */
        commandSegments.add(aimBot.shootWhileMoving(
                new BezierCurve(startingPosition, position1),
                assumedSpeakerPosition,
                3000
        ));

        commandSegments.add(commandFactory.faceDirection(new Rotation2D(0)));
        commandSegments.add(commandFactory.moveToPoint(new Vector2D(new double[] {3.8, 1.8})));
        return commandSegments;
    }
}
