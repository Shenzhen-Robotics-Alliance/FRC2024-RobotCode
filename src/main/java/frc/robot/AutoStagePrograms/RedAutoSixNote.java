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

        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("first note and grab").get(0),
                FieldPositions.toActualPosition(FieldPositions.speakerPosition),
                shootingTimeOut
        ));

        commandSegments.add(aimBot.grabNote(
                FieldPositions.toActualPosition(FieldPositions.nearNote2),
                FieldPositions.toActualRotation(new Rotation2D(Math.toRadians(180))),
                2000
        ));

        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot second and grab third").get(0),
                FieldPositions.toActualPosition(FieldPositions.speakerPosition),
                shootingTimeOut
        ));

        commandSegments.add(aimBot.grabNote(
                        FieldPositions.toActualPosition(FieldPositions.nearNote1),
                        FieldPositions.toActualRotation(new Rotation2D(Math.toRadians(-135))),
                        2000
        ));

        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot third grab fourth").get(0),
                FieldPositions.toActualPosition(FieldPositions.speakerPosition),
                shootingTimeOut
        ));

        commandSegments.add(aimBot.grabNote(
                FieldPositions.toActualPosition(FieldPositions.nearNote3),
                FieldPositions.toActualRotation(new Rotation2D(Math.toRadians(135))),
                2000
        ));

        commandSegments.add(aimBot.shootWhileMoving(
                SequentialCommandFactory.getBezierCurvesFromPathFile("shoot fourth move to fifth").get(0),
                FieldPositions.toActualPosition(FieldPositions.speakerPosition),
                shootingTimeOut
        ));

        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> SequentialCommandFactory.getBezierCurvesFromPathFile("shoot fourth move to fifth").get(1),
                ()->{}, ()->{}, ()->{},
                () -> true,
                () -> new Rotation2D(Math.toRadians(180)), () -> new Rotation2D(Math.toRadians(180))
        ));

        // TODO finish the rest

        commandSegments.addAll(Arrays.asList(commandFactory.followPath(
                "shoot fifth grab sixth",
                new Rotation2D[] {
                        FieldPositions.toActualRotation(new Rotation2D(Math.toRadians(180))),
                        FieldPositions.toActualRotation(new Rotation2D(Math.toRadians(180))),
                        FieldPositions.toActualRotation(new Rotation2D(Math.toRadians(135))),
                        FieldPositions.toActualRotation(new Rotation2D(Math.toRadians(135)))
                },
                () -> {}, () -> {}, () -> {}
        )));

        commandSegments.addAll(Arrays.asList(commandFactory.followPath(
                "shoot sixth",
                new Rotation2D[] {
                        FieldPositions.toActualRotation(new Rotation2D(Math.toRadians(135))),
                        FieldPositions.toActualRotation(new Rotation2D(Math.toRadians(180)))
                },
                () -> {}, () -> {}, () -> {}
        )));

        return commandSegments;
    }
}
