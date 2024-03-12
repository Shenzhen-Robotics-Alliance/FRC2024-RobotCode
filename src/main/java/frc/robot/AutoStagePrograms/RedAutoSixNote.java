package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.CommandSequenceGenerator;
import frc.robot.Utils.ComputerVisionUtils.AutoStageVisionAimBot;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class RedAutoSixNote implements CommandSequenceGenerator {
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, SequentialCommandFactory.getRobotStartingPosition("first note and grab"), new Rotation2D(Math.toRadians(180)));
        final AutoStageVisionAimBot aimBot = new AutoStageVisionAimBot(robotCore, 6000);
        final long shootingTimeOut = 5000, intakeTimeOut = 2000;

        commandSegments.add(commandFactory.calibratePositionEstimator());

        /* shoot the preloaded and move to the second */
        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("first note and grab").get(0),
                FieldPositions.toActualPosition(FieldPositions.speakerPosition),
                shootingTimeOut
        ));

        /* grab second */
        commandSegments.add(aimBot.grabNote(
                FieldPositions.toActualPosition(FieldPositions.nearNote2),
                FieldPositions.toActualRotation(new Rotation2D(Math.toRadians(180))),
                intakeTimeOut
        ));

        /* shoot second and move to third */
        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot second and grab third").get(0),
                FieldPositions.toActualPosition(FieldPositions.speakerPosition),
                shootingTimeOut
        ));

        /* grab third */
        commandSegments.add(aimBot.grabNote(
                        FieldPositions.toActualPosition(FieldPositions.nearNote1),
                        FieldPositions.toActualRotation(new Rotation2D(Math.toRadians(180))),
                        intakeTimeOut
        ));

        /* shoot third and move to fourth */
        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot third grab fourth").get(0),
                FieldPositions.toActualPosition(FieldPositions.speakerPosition),
                shootingTimeOut
        ));

        /* grab fourth */
        commandSegments.add(aimBot.grabNote(
                FieldPositions.toActualPosition(FieldPositions.nearNote3),
                FieldPositions.toActualRotation(new Rotation2D(Math.toRadians(135))),
                intakeTimeOut
        ));

        /* shoot fourth */
        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot fourth move to fifth").get(0),
                FieldPositions.toActualPosition(FieldPositions.speakerPosition),
                shootingTimeOut
        ));
        /* wait for arm to reset */
        commandSegments.add(aimBot.waitForArmToLower());

        /* move to fifth */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> SequentialCommandFactory.getBezierCurvesFromPathFile("shoot fourth move to fifth").get(1),
                ()->{}, ()->{}, ()->{},
                () -> true,
                () -> new Rotation2D(Math.toRadians(180)), () -> new Rotation2D(Math.toRadians(180))
        ));

        /* move back to shooting spot */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> SequentialCommandFactory.getBezierCurvesFromPathFile("shoot fifth grab sixth").get(0),
                ()->{}, ()->{}, ()->{},
                () -> true,
                () -> new Rotation2D(Math.toRadians(180)), () -> new Rotation2D(Math.toRadians(180))
        ));

        /* shoot fifth */
        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot fifth grab sixth").get(1),
                FieldPositions.toActualPosition(FieldPositions.speakerPosition),
                shootingTimeOut
        ));

        /* move to sixth */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> SequentialCommandFactory.getBezierCurvesFromPathFile("shoot fourth move to fifth").get(2),
                ()->{}, ()->{}, ()->{},
                () -> true,
                () -> new Rotation2D(Math.toRadians(180)), () -> new Rotation2D(Math.toRadians(135))
        ));
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> SequentialCommandFactory.getBezierCurvesFromPathFile("shoot fourth move to fifth").get(3),
                ()->{}, ()->{}, ()->{},
                () -> true,
                () -> new Rotation2D(Math.toRadians(135)), () -> new Rotation2D(Math.toRadians(135))
        ));

        /* grab sixth */
        commandSegments.add(aimBot.grabNote(
                FieldPositions.toActualPosition(FieldPositions.farNoteLefter),
                new Rotation2D(Math.toRadians(135)),
                intakeTimeOut
        ));

        /* move back to shooting spot */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> SequentialCommandFactory.getBezierCurvesFromPathFile("shoot sixth").get(0),
                ()->{}, ()->{}, ()->{},
                () -> true,
                () -> new Rotation2D(Math.toRadians(135)), () -> new Rotation2D(Math.toRadians(180))
        ));

        /* shoot sixth */
        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot sixth").get(1),
                FieldPositions.toActualPosition(FieldPositions.speakerPosition),
                shootingTimeOut
        ));

        return commandSegments;
    }
}
