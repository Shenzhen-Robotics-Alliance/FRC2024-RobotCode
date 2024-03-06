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

public class BlueDS1PickNotes implements CommandSequenceGenerator {
    public static final Vector2D startingPosition = new Vector2D(new double[] {1.68, 0.3}),
            splitNotePosition = startingPosition.addBy(new Vector2D(new double[] {0, 3})),
            secondNoteIntakePosition = new Vector2D(new double[] {2.7, 7.8}),
            midLineRightMostNotePosition = new Vector2D(new double[] {1.68*2, 16.54/2}),
            moveBackFromMidLineRightMostEnd = new Vector2D(new double[] {1.4, 4.2}),
            moveBackFromCenterLineRighterMid = moveBackFromMidLineRightMostEnd.addBy(new Vector2D(new double[] {0.8, 0.8})),
            splitSecondNote2 = moveBackFromMidLineRightMostEnd.addBy(new Vector2D(new double[] {-0.5, -0.5})),
            splitSecondNote4 = new Vector2D(new double[] {0, 5.3}),
            splitSecondNote3 = splitSecondNote4.addBy(new Vector2D(new double[] {0, -0.8})),
            midLineCenterNotePosition = new Vector2D(new double[] {0, 16.54/2}),
            thirdNoteIntakePosition = midLineCenterNotePosition.addBy(new Vector2D(new double[] {0, -0.5})),
            midLineRighterNotePosition = new Vector2D(new double[] {1.68, 16.54/2}),
            fourthNoteIntakePosition = midLineRighterNotePosition.addBy(new Vector2D(new double[] {0, -0.5})),
            moveBackFromFourth2 = fourthNoteIntakePosition.addBy(new Vector2D(new double[] {1, -1})),
            endingPoint = new Vector2D(new double[] {1.5, 1.5}),
            moveBackFromFourth3 = endingPoint.addBy(new Vector2D(new double[] {0, 2}));

    private static final Rotation2D staringRotation = new Rotation2D(Math.toRadians(90)),
            rotation1 = new Rotation2D(Math.toRadians(135)), rotation2 = new Rotation2D(Math.toRadians(-135));

    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, startingPosition, staringRotation);
        final AutoStageVisionAimBot aimBot = new AutoStageVisionAimBot(robotCore, 6000);
        final SequentialCommandSegment waitForArmToBeInPosition = new SequentialCommandSegment(
                () -> true,
                () -> null,
                () -> {}, () -> {}, () -> {},
                robotCore.transformableArm::transformerInPosition,
                robotCore.positionReader::getRobotRotation2D, robotCore.positionReader::getRobotRotation2D
        );
        final long grabTimeOut = 2000, splitTimeOut = 7000;

        commandSegments.add(commandFactory.calibratePositionEstimator());

        /* split the preloaded note */
        commandSegments.add(aimBot.splitForwardWhileMoving(
                new BezierCurve(startingPosition, splitNotePosition),
                staringRotation, staringRotation,
                splitTimeOut
        ));

        /* drive to the right most note */
        commandSegments.add(commandFactory.moveFromPointToPoint(splitNotePosition, secondNoteIntakePosition, staringRotation, rotation1));
        /* take it */
        commandSegments.add(aimBot.grabNote(midLineRightMostNotePosition, rotation1, grabTimeOut));

        /* move back */
        commandSegments.add(commandFactory.moveFromPointToMidPointToPoint(secondNoteIntakePosition, moveBackFromCenterLineRighterMid, moveBackFromMidLineRightMostEnd));
        /* split, through the stage zone */
        commandSegments.add(waitForArmToBeInPosition);
        commandSegments.add(aimBot.splitForwardWhileMoving(
                new BezierCurve(moveBackFromMidLineRightMostEnd, splitSecondNote2, splitSecondNote3, splitSecondNote4),
                rotation1, rotation1,
                splitTimeOut
        ));

        /* move back to the third note */
        commandSegments.add(commandFactory.moveFromPointToPoint(splitSecondNote4, thirdNoteIntakePosition, rotation1, new Rotation2D(Math.toRadians(180))));
        /* grab */
        commandSegments.add(aimBot.grabNote(midLineCenterNotePosition, new Rotation2D(Math.toRadians(180)), grabTimeOut));

        /* back */
        commandSegments.add(commandFactory.moveFromPointToPoint(thirdNoteIntakePosition, splitSecondNote4, new Rotation2D(Math.toRadians(180)), rotation1));
        /* same reversely, split, through the stage */
        commandSegments.add(waitForArmToBeInPosition);
        commandSegments.add(aimBot.splitForwardWhileMoving(
                new BezierCurve(splitSecondNote4, splitSecondNote3, splitSecondNote2, moveBackFromMidLineRightMostEnd),
                rotation1, rotation1,
                splitTimeOut
        ));

        /* to the fourth note */
        commandSegments.add(commandFactory.moveFromPointToMidPointToPoint(moveBackFromMidLineRightMostEnd, moveBackFromCenterLineRighterMid, thirdNoteIntakePosition, rotation1, rotation2));
        /* grab the fourth note */
        commandSegments.add(aimBot.grabNote(midLineRighterNotePosition, rotation2, grabTimeOut));
        /* drive back to the end */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(fourthNoteIntakePosition, moveBackFromFourth2, moveBackFromFourth3, endingPoint),
                ()->{},()->{},()->{},
                () -> true,
                () -> rotation2,
                () -> new Rotation2D(0)
        ));
        return commandSegments;
    }
}
