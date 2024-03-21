package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.CommandSequenceGenerator;
import frc.robot.Utils.ComputerVisionUtils.AutoStageVisionAimBot;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class FiveNotesUpper implements CommandSequenceGenerator {
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, SequentialCommandFactory.getRobotStartingPosition("shoot first move to second upper"), FieldPositions.toActualRotation(new Rotation2D(Math.toRadians(90))));
        final AutoStageVisionAimBot aimBot = new AutoStageVisionAimBot(robotCore, 6000);
        final long shootingTimeOut = 5000, intakeTimeOut = 2000;

        commandSegments.add(commandFactory.calibratePositionEstimator());

        /* shoot the preloaded */
        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot first move to second upper").get(0),
                FieldPositions.speakerPosition,
                shootingTimeOut
        ));
        /* move to second */
        commandSegments.add(commandFactory.followSingleCurve(
                "shoot first move to second upper", 1,
                new Rotation2D(Math.toRadians(180))
        ));
        commandSegments.add(commandFactory.followSingleCurve(
                "shoot first move to second upper", 2,
                new Rotation2D(Math.toRadians(180))
        ));

        /* grab second */
        commandSegments.add(aimBot.grabNote(
                FieldPositions.farNoteRightMost,
                new Rotation2D(Math.toRadians(180)),
                intakeTimeOut
        ));
        /* move to shooting position */
        commandSegments.add(commandFactory.followSingleCurve(
                "shoot second move to third upper", 0,
                new Rotation2D(Math.toRadians(160))
        ));
        /* shoot second */
        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot second move to third upper").get(1),
                FieldPositions.speakerPosition,
                shootingTimeOut
        ));
        /* move to third */
        commandSegments.add(commandFactory.followSingleCurve(
                "shoot second move to third upper", 2,
                new Rotation2D(Math.toRadians(-160))
        ));

        /* grab third */
        commandSegments.add(aimBot.grabNote(
                FieldPositions.farNoteRighter,
                new Rotation2D(Math.toRadians(-160)),
                intakeTimeOut
        ));
        /* move to shooting position */
        commandSegments.add(commandFactory.followSingleCurve(
                "shoot third move to fourth upper", 0,
                new Rotation2D(Math.toRadians(160))
        ));
        /* shoot third */
        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot third move to fourth upper").get(1),
                FieldPositions.speakerPosition,
                shootingTimeOut
        ));
        /* move to fourth */
        commandSegments.add(commandFactory.followSingleCurve(
                "shoot third move to fourth upper", 2,
                new Rotation2D(Math.toRadians(180))
        ));

        /* grab fourth */
        commandSegments.add(aimBot.grabNote(
                FieldPositions.farNoteCenter,
                new Rotation2D(Math.toRadians(180)),
                intakeTimeOut
        ));
        /* move to shooting position */
        commandSegments.add(commandFactory.followSingleCurve(
                "shoot fourth move to fifth upper", 0,
                new Rotation2D(Math.toRadians(-170))
        ));
        /* shoot fourth */
        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot fourth move to fifth upper").get(1),
                FieldPositions.speakerPosition,
                shootingTimeOut
        ));
        /* move to fifth */
        commandSegments.add(commandFactory.followSingleCurve(
                "shoot fourth move to fifth upper", 2,
                new Rotation2D(Math.toRadians(180))
        ));

        /* grab fifth */
        commandSegments.add(aimBot.grabNote(
                FieldPositions.nearNote1,
                new Rotation2D(Math.toRadians(180)),
                intakeTimeOut
        ));
        /* shoot fifth */
        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot fifth upper").get(0),
                FieldPositions.speakerPosition,
                shootingTimeOut
        ));
        return commandSegments;
    }
}
