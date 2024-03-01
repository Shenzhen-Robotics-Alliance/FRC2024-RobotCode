package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.ComputerVisionUtils.AutoStageVisionAimBot;
import frc.robot.Utils.Flip;
import frc.robot.Utils.MathUtils.BezierCurve;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.SpeedCurves;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.List;

public class BlueDS2 implements AutoStageProgram {
    public static final Vector2D
            assumedSpeakerPosition = Flip.flipHorizontally(RedDS2.assumedSpeakerPosition),
            allianceFrontNotePosition = Flip.flipHorizontally(RedDS2.assumedSpeakerPosition),
            allianceRighterNotePosition = Flip.flipHorizontally(RedDS2.assumedSpeakerPosition),
            midLineLefterNotePosition = Flip.flipHorizontally(RedDS2.assumedSpeakerPosition),
            midLineCenterNotePosition = Flip.flipHorizontally(RedDS2.assumedSpeakerPosition),
            midLineRighterNotePosition = Flip.flipHorizontally(RedDS2.assumedSpeakerPosition);
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final double timeScaleAutoIntake = 0.5;
        final long intakeTimeOut = 3000;
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, AutonomousTemplateBlueDS2.startingPosition, new Rotation2D(Math.toRadians(-90)));
        final AutoStageVisionAimBot aimBot = new AutoStageVisionAimBot(robotCore, intakeTimeOut);
        final SequentialCommandSegment waitForArmToBeInPosition = new SequentialCommandSegment(
                () -> true,
                () -> null,
                () -> {
                }, () -> {
        }, () -> {
        },
                robotCore.transformableArm::transformerInPosition,
                robotCore.positionReader::getRobotRotation2D, robotCore.positionReader::getRobotRotation2D
        );

        commandSegments.add(commandFactory.calibratePositionEstimator());

        /* shoot the first note */
        commandSegments.add(aimBot.shootWhileMoving(
                new BezierCurve(AutonomousTemplateBlueDS2.startingPosition, AutonomousTemplateBlueDS2.position1),
                assumedSpeakerPosition,
                4000
        ));

        /* the note in front */
        commandSegments.add(commandFactory.justDoIt(aimBot.prepareToIntake()));
        commandSegments.add(commandFactory.moveToPointIf(
                () -> true, AutonomousTemplateBlueDS2.position1, () -> {
                }, () -> {
                }, () -> {
                }, new Rotation2D(Math.toRadians(180)))
        );
        commandSegments.add(aimBot.grabNote(
                () -> true,
                allianceFrontNotePosition,
                new Rotation2D(Math.toRadians(180)),
                2000,
                false));
        /* shoot the gabbed note */
        commandSegments.add(aimBot.shootWhileMoving(
                new BezierCurve(AutonomousTemplateBlueDS2.position1, AutonomousTemplateBlueDS2.position2, AutonomousTemplateBlueDS2.position3),
                assumedSpeakerPosition,
                4000
        ));

        /* wait for the transformer to get in position */
        commandSegments.add(waitForArmToBeInPosition);
        /* go over the stage zone */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(AutonomousTemplateBlueDS2.position3, AutonomousTemplateBlueDS2.position4, AutonomousTemplateBlueDS2.position5, AutonomousTemplateBlueDS2.position6),
                () -> {
                }, () -> {
        }, () -> {
        },
                () -> true,
                () -> new Rotation2D(Math.toRadians(-180)), () -> new Rotation2D(Math.toRadians(-180))
        ));
        /* the lefter note on the middle line */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(AutonomousTemplateBlueDS2.position6, AutonomousTemplateBlueDS2.position7, AutonomousTemplateBlueDS2.position8),
                aimBot.prepareToIntake(), () -> {
        }, () -> {
        },
                () -> true,
                () -> new Rotation2D(Math.toRadians(-180)), () -> new Rotation2D(Math.toRadians(-135)),
                SpeedCurves.easeOut, timeScaleAutoIntake
        ));
        commandSegments.add(aimBot.grabNote(midLineLefterNotePosition, new Rotation2D(Math.toRadians(-135)), 2000));

        /* same way back */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(AutonomousTemplateBlueDS2.position8, AutonomousTemplateBlueDS2.position7, AutonomousTemplateBlueDS2.position6),
                () -> {
                }, () -> {
        }, () -> {
        },
                robotCore.transformableArm::transformerInPosition,
                () -> new Rotation2D(Math.toRadians(-135)), () -> new Rotation2D(Math.toRadians(-180))
        ));
        /* stage zone (back) */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(AutonomousTemplateBlueDS2.position6, AutonomousTemplateBlueDS2.position5, AutonomousTemplateBlueDS2.position4, AutonomousTemplateBlueDS2.position3),
                aimBot.prepareToShoot(), () -> {
        }, () -> {
        },
                () -> true,
                () -> new Rotation2D(Math.toRadians(-180)), () -> new Rotation2D(Math.toRadians(-180))
        ));
        /* shoot the note */
        commandSegments.add(aimBot.shootWhileMoving(
                new BezierCurve(AutonomousTemplateBlueDS2.position3, AutonomousTemplateBlueDS2.position10, AutonomousTemplateBlueDS2.position11),
                assumedSpeakerPosition,
                4000
        ));

        /* grab the righter note of this alliance */
        commandSegments.add(aimBot.grabNote(() -> true, allianceRighterNotePosition, new Rotation2D(Math.toRadians(180)), intakeTimeOut, true));
        /* shoot still right here */
        commandSegments.add(aimBot.shootWhileMoving(
                new BezierCurve(AutonomousTemplateBlueDS2.position12, AutonomousTemplateBlueDS2.position12.addBy(new Vector2D(new double[]{0, 1}))),
                assumedSpeakerPosition,
                4000
        ));

        /* move back, crossing stage */
        commandSegments.add(waitForArmToBeInPosition);
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(AutonomousTemplateBlueDS2.position12.addBy(new Vector2D(new double[]{0, 1})), AutonomousTemplateBlueDS2.position4, AutonomousTemplateBlueDS2.position13, AutonomousTemplateBlueDS2.position14),
                () -> {
                }, () -> {
        }, () -> {
        },
                () -> true,
                () -> new Rotation2D(Math.toRadians(-180)), () -> new Rotation2D(Math.toRadians(-180))
        ));
        /* grab the center note on middle line */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(AutonomousTemplateBlueDS2.position14, AutonomousTemplateBlueDS2.position15),
                () -> {
                }, () -> {
        }, () -> {
        },
                () -> true,
                () -> new Rotation2D(Math.toRadians(-180)), () -> new Rotation2D(Math.toRadians(-180)),
                SpeedCurves.easeOut,
                timeScaleAutoIntake
        ));
        commandSegments.add(aimBot.grabNote(midLineCenterNotePosition, new Rotation2D(Math.toRadians(-180)), intakeTimeOut));

        /* drive back in the same way */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(AutonomousTemplateBlueDS2.position15, AutonomousTemplateBlueDS2.position14),
                () -> {
                }, () -> {
        }, () -> {
        },
                robotCore.transformableArm::transformerInPosition,
                () -> new Rotation2D(Math.toRadians(-135)), () -> new Rotation2D(Math.toRadians(-180))
        ));
        /* crossing stage */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(AutonomousTemplateBlueDS2.position14, AutonomousTemplateBlueDS2.position13, AutonomousTemplateBlueDS2.position4, AutonomousTemplateBlueDS2.position16),
                () -> {
                }, () -> {
        }, () -> {
        },
                () -> true,
                () -> new Rotation2D(Math.toRadians(-180)), () -> new Rotation2D(Math.toRadians(-180))
        ));
        /* shoot */
        commandSegments.add(aimBot.shootWhileMoving(
//                new BezierCurve(AutonomousTemplateBlueDS2.position16, AutonomousTemplateBlueDS2.position12, AutonomousTemplateBlueDS2.position17),
                new BezierCurve(AutonomousTemplateBlueDS2.position16, AutonomousTemplateBlueDS2.position12, AutonomousTemplateBlueDS2.position12),
                assumedSpeakerPosition,
                3000
        ));
        return commandSegments;
    }
}
