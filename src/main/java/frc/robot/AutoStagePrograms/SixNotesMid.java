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

public class SixNotesMid implements CommandSequenceGenerator {
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, SequentialCommandFactory.getRobotStartingPosition("first note"), new Rotation2D(Math.toRadians(180)));
        final AutoStageVisionAimBot aimBot = new AutoStageVisionAimBot(robotCore, 6000);
        final long shootingTimeOut = 5000, intakeTimeOut = 2000;

        commandSegments.add(commandFactory.calibratePositionEstimator());

        /* shoot the preloaded */
        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("first note").get(0),
                FieldPositions.speakerPosition,
                shootingTimeOut
        ));

        /* move to second */
        commandSegments.addAll(Arrays.asList(commandFactory.followPathFacing(
                "move to second note",
                new Rotation2D(Math.toRadians(135))
        )));

        /* grab second */
        commandSegments.add(aimBot.grabNote(
                FieldPositions.nearNote3,
                new Rotation2D(Math.toRadians(135)),
                intakeTimeOut
        ));

        /* shoot second */
        commandSegments.add(aimBot.shootWhileMoving(
                () -> true,
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot second and grab third").get(0),
                FieldPositions.speakerPosition,
                new Rotation2D(Math.toRadians(-150)),
                shootingTimeOut
        ));

        /* move to third */
        commandSegments.add(commandFactory.followSingleCurve(
                "shoot second and grab third", 1,
                new Rotation2D(Math.toRadians(-150))
        ));

        /* grab third */
        commandSegments.add(aimBot.grabNote(
                        FieldPositions.nearNote1,
                        new Rotation2D(Math.toRadians(-150)),
                        intakeTimeOut
        ));

        /* shoot third */
        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot third grab fourth").get(0),
                FieldPositions.speakerPosition,
                shootingTimeOut
        ));
        /* move to fourth */
        commandSegments.add(commandFactory.followSingleCurve(
                "shoot third grab fourth", 1,
                new Rotation2D(Math.toRadians(-150))
        ));

        /* grab fourth */
        commandSegments.add(aimBot.grabNote(
                FieldPositions.farNoteRighter,
                new Rotation2D(Math.toRadians(-150)),
                intakeTimeOut
        ));

        /* move to shooting spot */
        commandSegments.add(commandFactory.followSingleCurve(
                "shoot fourth move to fifth", 0,
                new Rotation2D(Math.toRadians(150))
        ));
        /* shoot fourth */
        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot fourth move to fifth").get(1),
                FieldPositions.speakerPosition,
                shootingTimeOut
        ));
        /* move to fifth */
        commandSegments.add(commandFactory.followSingleCurve(
                "shoot fourth move to fifth", 2,
                new Rotation2D(Math.toRadians(180))
        ));
        /* grab fifth */
        commandSegments.add(aimBot.grabNote(
                FieldPositions.nearNote2,
                new Rotation2D(Math.toRadians(180)),
                intakeTimeOut
        ));

        /* shoot fifth */
        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot fifth grab sixth fast").get(0),
                FieldPositions.speakerPosition,
                shootingTimeOut
        ));
        /* move to sixth */
        commandSegments.add(commandFactory.followSingleCurve(
                "shoot fifth grab sixth fast", 1,
                new Rotation2D(Math.toRadians(-120))
        ));

        /* grab sixth */
        commandSegments.add(aimBot.grabNote(
                FieldPositions.farNoteCenter,
                new Rotation2D(Math.toRadians(-120)),
                intakeTimeOut
        ));
        /* move to shooting spot */
        commandSegments.add(commandFactory.followSingleCurve(
                "shoot sixth", 0,
                new Rotation2D(Math.toRadians(-170))
        ));
        /* shoot sixth */
        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot sixth").get(1),
                FieldPositions.speakerPosition,
                shootingTimeOut
        ));

        return commandSegments;
    }
}
