package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.CommandSequenceGenerator;
import frc.robot.Utils.MathUtils.BezierCurve;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.List;

public class AutonomousTemplateRedDS2 implements CommandSequenceGenerator {
    public static final Vector2D startingPosition = new Vector2D(new double[] {0, 0.4}),
            position1 = new Vector2D(new double[] {-0.1, 1.3}),
            position2 = new Vector2D(new double[] {0.8, 1.7}),
            position3 = new Vector2D(new double[] {1.2, 2.8}),
            position4 = new Vector2D(new double[] {1.5, 4.8}),
            position5 = new Vector2D(new double[] {-0.2, 5.6}),
            position6 = new Vector2D(new double[] {-0.7, 6.5}),
            position7 = new Vector2D(new double[] {-1, 8.2}),
            position8 = RedDS2.midLineLefterNotePosition.addBy(new Vector2D(new double[] {0.3, -0.3})),
            position10 = new Vector2D(new double[] {0.8, 0.8}),
            position11 = new Vector2D(new double[] {2.2, 1}),
            position12 = new Vector2D(new double[] {1.9, 2.5}),
            position13 = new Vector2D(new double[] {0, 5.4}),
            position14 = new Vector2D(new double[] {0.3, 7.6}),
            position15 = new Vector2D(new double[] {0.1, 8.1}),
            position16 = position4.addBy(Vector2D.displacementToTarget(position4, position12).multiplyBy(0.5)),
            position17 = new Vector2D(new double[] {2.8, 3}),
            position18 = new Vector2D(new double[] {3.2, 7}),
            position19 = new Vector2D(new double[] {1.8, 7.5}),
            position20 = new Vector2D(new double[] {2.2, 9}),
            position21 = new Vector2D(new double[] {3, 6}),
            position22 = new Vector2D(new double[] {3, 2});

    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, startingPosition, new Rotation2D(Math.toRadians(-90)));
        commandSegments.add(commandFactory.calibratePositionEstimator());

        /* the note in front */
        commandSegments.add(commandFactory.moveFromPointToPointAndStop(
                startingPosition, position1,
                new Rotation2D(Math.toRadians(-90)), new Rotation2D(Math.toRadians(-135))
        ));

        /* shoot the first note */
        commandSegments.add(commandFactory.moveFromPointToMidPointToPoint(
                position1, position2, position3,
                new Rotation2D(Math.toRadians(-135)), new Rotation2D(Math.toRadians(-180))
        ));

        /* go over the stage zone */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(position3, position4, position5, position6),
                () -> {}, () -> {}, () -> {},
                () -> true,
                () -> new Rotation2D(Math.toRadians(-180)), () -> new Rotation2D(Math.toRadians(-180))
        ));

        /* the lefter note on the middle line */
        commandSegments.add(commandFactory.moveFromPointToMidPointToPointAndStop(
                position6, position7, position8,
                new Rotation2D(Math.toRadians(-180)), new Rotation2D(Math.toRadians(-135))
        ));

        /* same way back */
        commandSegments.add(commandFactory.moveFromPointToMidPointToPoint(
                position8, position7, position6,
                new Rotation2D(Math.toRadians(-135)), new Rotation2D(Math.toRadians(-180))
        ));
        /* stage zone (back) */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(position6, position5, position4, position3),
                () -> {}, () -> {}, () -> {},
                () -> true,
                () -> new Rotation2D(Math.toRadians(-180)), () -> new Rotation2D(Math.toRadians(-180))
        ));

        /* shoot and grab another from beside */
        commandSegments.add(commandFactory.moveFromPointToMidPointToPointAndStop(
                position3, position10, position11,
                new Rotation2D(Math.toRadians(-180)), new Rotation2D(Math.toRadians(-180))
        ));

        /* move back, crossing stage */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(position12, position4, position13, position14),
                () -> {}, () -> {}, () -> {},
                () -> true,
                () -> new Rotation2D(Math.toRadians(-180)), () -> new Rotation2D(Math.toRadians(-180))
        ));

        /* grab the center note on middle line */
        commandSegments.add(commandFactory.moveFromPointToPointAndStop(
                position14, position15,
                new Rotation2D(Math.toRadians(-180)), new Rotation2D(Math.toRadians(-180))
        ));

        /* drive back in the same way */
        commandSegments.add(commandFactory.moveFromPointToPoint(
                position15, position14,
                new Rotation2D(Math.toRadians(-180)), new Rotation2D(Math.toRadians(-180))
        ));
        /* crossing stage */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(position14, position13, position4, position16),
                () -> {}, () -> {}, () -> {},
                () -> true,
                () -> new Rotation2D(Math.toRadians(-180)), () -> new Rotation2D(Math.toRadians(-180))
        ));

        /* shoot */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(position16, position12, position17, position18),
                () -> {}, () -> {}, () -> {},
                () -> true,
                () -> new Rotation2D(Math.toRadians(-180)), () -> new Rotation2D(Math.toRadians(-180))
        ));

        /* right note on middle line */
        commandSegments.add(commandFactory.moveFromPointToMidPointToPointAndStop(
                position18, position19, position20, new Rotation2D(Math.toRadians(-180)), new Rotation2D(Math.toRadians(-180))
        ));

        commandSegments.add(commandFactory.moveFromPointToMidPointToPointAndStop(
                position20, position21, position22, new Rotation2D(Math.toRadians(180)), new Rotation2D(Math.toRadians(135))
        ));

        return commandSegments;
    }
}
