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

public class TestAutoShooting implements CommandSequenceGenerator {
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, new Vector2D(new double[] {0, 0}), new Rotation2D(Math.toRadians(-90)));
        final AutoStageVisionAimBot aimBot = new AutoStageVisionAimBot(robotCore, 2000);
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();

        commandSegments.add(commandFactory.calibratePositionEstimator());
        commandSegments.add(commandFactory.moveFromPointToMidPointToPoint(
                new Vector2D(new double[] {0, 0}),
                new Vector2D(new double[] {0, 2.3}),
                new Vector2D(new double[] {0.8, 2.3-0.8}),
                aimBot.prepareToShoot(new Vector2D(new double[] {2, -0.8})), ()->{}, () ->{},
                new Rotation2D(Math.toRadians(-90)),
                new Rotation2D(Math.toRadians(-150))
        ));

//        commandSegments.add(new SequentialCommandSegment(
//                () -> true,
//                () -> new BezierCurve(new Vector2D(new double[] {0.8, 2.3-0.8}), new Vector2D(new double[] {1.2, 2.3-1.2}), new Vector2D(new double[] {2, 2.5}), new Vector2D(new double[] {2, 3.1})),
//                () -> {}, () -> {}, () -> {},
//                robotCore.chassisModule::isCurrentRotationalTaskFinished,
//                () -> new Rotation2D(Math.toRadians(-180)), () -> new Rotation2D(Math.toRadians(-180))
//        ));

        commandSegments.add(aimBot.shootWhileMoving(
                new BezierCurve(new Vector2D(new double[] {0.8, 2.3-0.8}), new Vector2D(new double[] {1.2, 2.3-1.2}), new Vector2D(new double[] {2, 2.5}), new Vector2D(new double[] {2, 3.1})),
                new Vector2D(new double[] {2, -0.8}),
                6000
        ));
        return commandSegments;
    }
}
