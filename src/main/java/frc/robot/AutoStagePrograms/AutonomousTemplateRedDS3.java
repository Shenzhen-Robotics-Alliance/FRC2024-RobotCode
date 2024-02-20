package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.MathUtils.BezierCurve;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.List;

public class AutonomousTemplateRedDS3 implements AutoStageProgram {
    private static final Vector2D startingPosition = new Vector2D(new double[] {2.9, 0.5}),
            position1 = new Vector2D(new double[] {2.9, 2.9}),
            position2 = new Vector2D(new double[] {3.36, 8}),
            position3 = new Vector2D(new double[]{3.5, 5.8}),
            position4 = new Vector2D(new double[]{2.9, 4.3}),
            position5 = new Vector2D(new double[]{2.2, 3.5}),
            position6 = new Vector2D(new double[]{1.65, 4.3}),
            position7 = new Vector2D(new double[] {2.2, 5.1}),
            position8 = new Vector2D(new double[] {2.2+0.25, 5.1+0.4}),
            position9 = new Vector2D(new double[] {1.68, 8})
    ;
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, startingPosition, new Rotation2D(Math.toRadians(180)));
        commandSegments.add(commandFactory.calibratePositionEstimator());

        /* first note beside starting position */
        commandSegments.add(commandFactory.moveFromPointToPointAndStop(
                startingPosition,
                position1,
                new Rotation2D(Math.toRadians(180)),
                new Rotation2D(Math.toRadians(135))
        ));

        /* note on the middle line */
        commandSegments.add(commandFactory.moveFromPointToPointAndStop(
                position1,
                position2,
                new Rotation2D(Math.toRadians(135)),
                new Rotation2D(Math.toRadians(180))
        ));

        /* drive back */
        commandSegments.add(commandFactory.moveFromPointToPoint(
                position2,
//                position3
                position4,
                new Rotation2D(Math.toRadians(180)),
                new Rotation2D(Math.toRadians(150))
        ));

        /* shooting curve */
        commandSegments.add(new SequentialCommandSegment(
                ()->true,
                () -> new BezierCurve(
                        position4,
                        position5,
                        position6,
                        position7),
                () ->{}, ()->{}, ()->{},
                () -> true,
                () -> new Rotation2D(Math.toRadians(180)),
                () -> new Rotation2D(Math.toRadians(150))
        ));

        /* second note on middle line */
        commandSegments.add(commandFactory.moveFromPointToMidPointToPointAndStop(
                position7,
                position8,
                position9,
                new Rotation2D(Math.toRadians(150)),
                new Rotation2D(Math.toRadians(180))
        ));

        return commandSegments;
    }
}
