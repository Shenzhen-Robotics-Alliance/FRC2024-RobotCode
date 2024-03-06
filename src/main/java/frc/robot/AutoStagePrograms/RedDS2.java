package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.CommandSequenceGenerator;
import frc.robot.Utils.ComputerVisionUtils.AutoStageVisionAimBot;
import frc.robot.Utils.MathUtils.BezierCurve;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.SpeedCurves;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.List;

public class RedDS2 implements CommandSequenceGenerator {
    public static final Vector2D
            assumedSpeakerPosition = new Vector2D(new double[] {1.45, 0}),
            allianceFrontNotePosition = new Vector2D(new double[] {0, 2.9}),
            allianceRighterNotePosition = new Vector2D(new double[] {1.45, 2.9}),
            midLineLefterNotePosition = new Vector2D(new double[] {-1.68, 8.27}).addBy(new Vector2D(new double[] {-0.3, 0.3})),
            midLineCenterNotePosition = new Vector2D(new double[] {0.3, 8.27}),
            midLineRighterNotePosition = new Vector2D(new double[] {1.68, 8.27});
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final double timeScaleAutoIntake = 0.5;
        final long intakeTimeOut = 1750;
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, AutonomousTemplateRedDS2.startingPosition, new Rotation2D(Math.toRadians(-90)));
        final AutoStageVisionAimBot aimBot = new AutoStageVisionAimBot(robotCore, 6000);
        final SequentialCommandSegment waitForArmToBeInPosition = new SequentialCommandSegment(
                () -> true,
                () -> null,
                () -> {}, () -> {}, () -> {},
                robotCore.transformableArm::transformerInPosition,
                robotCore.positionReader::getRobotRotation2D, robotCore.positionReader::getRobotRotation2D
        );

        commandSegments.add(commandFactory.calibratePositionEstimator());

        /* shoot the first note */
        commandSegments.add(aimBot.shootWhileMoving(
                new BezierCurve(AutonomousTemplateRedDS2.startingPosition, AutonomousTemplateRedDS2.position1),
                assumedSpeakerPosition,
                3000
        ));

        /* the note in front */
        commandSegments.add(commandFactory.justDoIt(aimBot.prepareToIntake()));
        commandSegments.add(commandFactory.moveToPointIf(
                () -> true, AutonomousTemplateRedDS2.position1, ()->{}, () -> {}, () ->{}, new Rotation2D(Math.toRadians(180)))
        );
        commandSegments.add(aimBot.grabNote(
                () -> true,
                allianceFrontNotePosition,
                new Rotation2D(Math.toRadians(180)),
                intakeTimeOut,
                false));
        /* shoot the gabbed note */
        commandSegments.add(aimBot.shootWhileMoving(
                new BezierCurve(AutonomousTemplateRedDS2.position1, AutonomousTemplateRedDS2.position2, AutonomousTemplateRedDS2.position3),
                assumedSpeakerPosition,
                3000
        ));

        /* wait for the transformer to get in position */
        commandSegments.add(waitForArmToBeInPosition);
        /* go over the stage zone */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(AutonomousTemplateRedDS2.position3, AutonomousTemplateRedDS2.position4, AutonomousTemplateRedDS2.position5, AutonomousTemplateRedDS2.position6),
                () -> {}, () -> {}, () -> {},
                () -> true,
                () -> new Rotation2D(Math.toRadians(-180)), () -> new Rotation2D(Math.toRadians(-180))
        ));
        /* the lefter note on the middle line */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(AutonomousTemplateRedDS2.position6, AutonomousTemplateRedDS2.position7, AutonomousTemplateRedDS2.position8),
                aimBot.prepareToIntake(), ()->{}, ()->{},
                () -> true,
                () -> new Rotation2D(Math.toRadians(-180)), () -> new Rotation2D(Math.toRadians(-135)),
                SpeedCurves.easeOut,timeScaleAutoIntake
        ));
        commandSegments.add(aimBot.grabNote(midLineLefterNotePosition, new Rotation2D(Math.toRadians(-135)), intakeTimeOut));

        /* same way back */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(AutonomousTemplateRedDS2.position8, AutonomousTemplateRedDS2.position7, AutonomousTemplateRedDS2.position6),
                () -> {}, () -> {}, () -> {},
                robotCore.transformableArm::transformerInPosition,
                () -> new Rotation2D(Math.toRadians(-135)), () -> new Rotation2D(Math.toRadians(-180))
        ));
        /* stage zone (back) */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(AutonomousTemplateRedDS2.position6, AutonomousTemplateRedDS2.position5, AutonomousTemplateRedDS2.position4, AutonomousTemplateRedDS2.position3),
                () -> {}, () -> {}, aimBot.prepareToShoot(),
                () -> true,
                () -> new Rotation2D(Math.toRadians(-180)), () -> new Rotation2D(Math.toRadians(-180))
        ));
        /* shoot the note */
        commandSegments.add(aimBot.shootWhileMoving(
                new BezierCurve(AutonomousTemplateRedDS2.position3, AutonomousTemplateRedDS2.position10, AutonomousTemplateRedDS2.position11),
                assumedSpeakerPosition,
                3000
        ));

        /* grab the righter note of this alliance */
        commandSegments.add(aimBot.grabNote(() -> true, allianceRighterNotePosition, new Rotation2D(Math.toRadians(180)), intakeTimeOut, true));
        /* shoot still right here */
        commandSegments.add(aimBot.shootWhileMoving(
                new BezierCurve(AutonomousTemplateRedDS2.position12, AutonomousTemplateRedDS2.position12.addBy(new Vector2D(new double[] {0, 1}))),
                assumedSpeakerPosition,
                4000
        ));

        // it ends here

//        /* move back, crossing stage */
//        commandSegments.add(waitForArmToBeInPosition);
//        commandSegments.add(new SequentialCommandSegment(
//                () -> true,
//                () -> new BezierCurve(AutonomousTemplateRedDS2.position12.addBy(new Vector2D(new double[] {0, 1})), AutonomousTemplateRedDS2.position4, AutonomousTemplateRedDS2.position13, AutonomousTemplateRedDS2.position14),
//                () -> {}, () -> {}, () -> {},
//                () -> true,
//                () -> new Rotation2D(Math.toRadians(-180)), () -> new Rotation2D(Math.toRadians(-180))
//        ));
//        /* grab the center note on middle line */
//        commandSegments.add(new SequentialCommandSegment(
//                () -> true,
//                () -> new BezierCurve(AutonomousTemplateRedDS2.position14, AutonomousTemplateRedDS2.position15),
//                ()->{}, ()->{}, ()->{},
//                () -> true,
//                () -> new Rotation2D(Math.toRadians(-180)), () -> new Rotation2D(Math.toRadians(-180)),
//                SpeedCurves.easeOut,
//                timeScaleAutoIntake
//                ));
//        commandSegments.add(aimBot.grabNote(midLineCenterNotePosition, new Rotation2D(Math.toRadians(-180)), intakeTimeOut));
//
//        /* drive back in the same way */
//        commandSegments.add(new SequentialCommandSegment(
//                () -> true,
//                () -> new BezierCurve(AutonomousTemplateRedDS2.position15, AutonomousTemplateRedDS2.position14),
//                () -> {}, () -> {}, () -> {},
//                robotCore.transformableArm::transformerInPosition,
//                () -> new Rotation2D(Math.toRadians(-135)), () -> new Rotation2D(Math.toRadians(-180))
//        ));
//        /* crossing stage */
//        commandSegments.add(new SequentialCommandSegment(
//                () -> true,
//                () -> new BezierCurve(AutonomousTemplateRedDS2.position14, AutonomousTemplateRedDS2.position13, AutonomousTemplateRedDS2.position4, AutonomousTemplateRedDS2.position16),
//                () -> {}, () -> {}, aimBot.prepareToShoot(),
//                () -> true,
//                () -> new Rotation2D(Math.toRadians(-180)), () -> new Rotation2D(Math.toRadians(-180))
//        ));
//        /* shoot */
//        commandSegments.add(aimBot.shootWhileMoving(
////                new BezierCurve(AutonomousTemplateRedDS2.position16, AutonomousTemplateRedDS2.position12, AutonomousTemplateRedDS2.position17),
//                new BezierCurve(AutonomousTemplateRedDS2.position16, AutonomousTemplateRedDS2.position12, AutonomousTemplateRedDS2.position12),
//                assumedSpeakerPosition,
//                3000
//        ));

//
//        /* grab the right note on middle line */
//        commandSegments.add(new SequentialCommandSegment(
//                () -> true,
//                () -> new BezierCurve(AutonomousTemplateRedDS2.position17, AutonomousTemplateRedDS2.position18, AutonomousTemplateRedDS2.position19, AutonomousTemplateRedDS2.position20),
//                ()->{}, ()->{}, ()->{},
//                () -> true,
//                () -> new Rotation2D(Math.toRadians(-180)), () -> new Rotation2D(Math.toRadians(-180)),
//                SpeedCurves.easeOut, timeScaleAutoIntake
//        ));
//        commandSegments.add(commandFactory.moveFromPointToMidPointToPoint(
//                AutonomousTemplateRedDS2.position18, AutonomousTemplateRedDS2.position19, AutonomousTemplateRedDS2.position20, new Rotation2D(Math.toRadians(-180)), new Rotation2D(Math.toRadians(-180))
//        ));
//        commandSegments.add(aimBot.grabNote(midLineRighterNotePosition, new Rotation2D(Math.toRadians(135)), intakeTimeOut));
//        /* shoot the last note and stop */
//        commandSegments.add(aimBot.shootWhileMoving(
//                new BezierCurve(AutonomousTemplateRedDS2.position20, AutonomousTemplateRedDS2.position21, AutonomousTemplateRedDS2.position22),
//                assumedSpeakerPosition,
//                6000
//        ));
//
//        commandSegments.add(commandFactory.lockChassis());
        return commandSegments;
    }
}
