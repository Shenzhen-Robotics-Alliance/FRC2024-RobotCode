package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.CommandSequenceGenerator;
import frc.robot.Utils.ComputerVisionUtils.AutoStageVisionAimBot;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.List;

public class FourNoteLower implements CommandSequenceGenerator {
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, SequentialCommandFactory.getRobotStartingPosition("shoot preloaded lower"), new Rotation2D(Math.toRadians(180)));
        final AutoStageVisionAimBot aimBot = new AutoStageVisionAimBot(robotCore, 6000);
        final long shootingTimeOut = 5000, intakeTimeOut = 2000;

        commandSegments.add(commandFactory.calibratePositionEstimator());

        /* shoot the preloaded */
        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot preloaded lower").get(0),
                FieldPositions.speakerPosition,
                shootingTimeOut
        ));

        /* grab second */
        commandSegments.add(commandFactory.faceDirection(new Rotation2D(Math.toRadians(180))));
        commandSegments.add(aimBot.grabNote(
                FieldPositions.nearNote1,
                new Rotation2D(Math.toRadians(180)),
                intakeTimeOut
        ));
        /* shoot second */
        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot second grab third lower").get(0),
                FieldPositions.speakerPosition,
                shootingTimeOut
        ));

        /* move to third */
        commandSegments.add(commandFactory.followSingleCurve(
                "shoot second grab third lower",
                1,
                new Rotation2D(Math.toRadians(135))
        ));
        /* grab third */
        commandSegments.add(aimBot.grabNote(FieldPositions.farNoteLefter, new Rotation2D(Math.toRadians(135)), intakeTimeOut));
        /* move to shooting spot */
        commandSegments.add(commandFactory.followSingleCurve(
                "shoot third lower",
                0,
                new Rotation2D(Math.toRadians(-135))
        ));
        /* shoot third */
        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot third lower").get(1),
                FieldPositions.speakerPosition,
                shootingTimeOut
        ));

        /* move to fourth */
        commandSegments.add(commandFactory.followSingleCurve(
                "grab fourth lower",
                0,
                new Rotation2D(Math.toRadians(180))
        ));
        /* grab fourth */
        commandSegments.add(aimBot.grabNote(FieldPositions.farNoteLeftMost, new Rotation2D(Math.toRadians(135)), intakeTimeOut));
        /* move to shooting spot */
        commandSegments.add(commandFactory.followSingleCurve(
                "shoot fourth lower",
                0,
                new Rotation2D(Math.toRadians(-135))
        ));
        /* shoot fourth */
        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot fourth lower").get(1),
                FieldPositions.speakerPosition,
                shootingTimeOut
        ));

        return commandSegments;
    }
}