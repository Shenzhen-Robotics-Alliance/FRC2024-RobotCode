package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.CommandSequenceGenerator;
import frc.robot.Utils.ComputerVisionUtils.AutoStageVisionAimBot;
import frc.robot.Utils.Flip;
import frc.robot.Utils.MathUtils.BezierCurve;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.List;

public class RedDS1PickNotes implements CommandSequenceGenerator {
    private static final Vector2D startingPosition = Flip.flipHorizontally(BlueDS1PickNotes.startingPosition),
            splitNotePosition = Flip.flipHorizontally(BlueDS1PickNotes.splitNotePosition),
            secondNoteIntakePosition = Flip.flipHorizontally(BlueDS1PickNotes.secondNoteIntakePosition),
            centerLineRightMostNotePosition = Flip.flipHorizontally(BlueDS1PickNotes.midLineRightMostNotePosition),
            moveBackFromCenterLineRighterEnd = Flip.flipHorizontally(BlueDS1PickNotes.moveBackFromMidLineRightMostEnd),
            moveBackFromCenterLineRighterMid = Flip.flipHorizontally(BlueDS1PickNotes.moveBackFromCenterLineRighterMid),
            splitSecondNote2 = Flip.flipHorizontally(BlueDS1PickNotes.splitSecondNote2),
            splitSecondNote4 = Flip.flipHorizontally(BlueDS1PickNotes.splitSecondNote4),
            splitSecondNote3 = Flip.flipHorizontally(BlueDS1PickNotes.splitSecondNote3),
            midLineCenterNotePosition = Flip.flipHorizontally(BlueDS1PickNotes.midLineCenterNotePosition),
            thirdNoteIntakePosition = Flip.flipHorizontally(BlueDS1PickNotes.thirdNoteIntakePosition),
            midLineRighterNotePosition = Flip.flipHorizontally(BlueDS1PickNotes.midLineRighterNotePosition),
            fourthNoteIntakePosition = Flip.flipHorizontally(BlueDS1PickNotes.fourthNoteIntakePosition),
            moveBackFromFourth2 = Flip.flipHorizontally(BlueDS1PickNotes.moveBackFromFourth2),
            endingPoint = Flip.flipHorizontally(BlueDS1PickNotes.endingPoint),
            moveBackFromFourth3 = Flip.flipHorizontally(BlueDS1PickNotes.moveBackFromFourth3);

    private static final Rotation2D staringRotation = new Rotation2D(Math.toRadians(-90)),
            rotation1 = new Rotation2D(Math.toRadians(-135)), rotation2 = new Rotation2D(Math.toRadians(135));

    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, AutonomousTemplateRedDS2.startingPosition, staringRotation);
        final AutoStageVisionAimBot aimBot = new AutoStageVisionAimBot(robotCore, 6000);
        final SequentialCommandSegment waitForArmToBeInPosition = new SequentialCommandSegment(
                () -> true,
                () -> null,
                () -> {}, () -> {}, () -> {},
                robotCore.transformableArm::transformerInPosition,
                robotCore.positionReader::getRobotRotation2D, robotCore.positionReader::getRobotRotation2D
        );
        final long grabTimeOut = 2000, splitTimeOut = 3000;

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
        commandSegments.add(aimBot.grabNote(centerLineRightMostNotePosition, rotation1, grabTimeOut));

        /* move back */
        commandSegments.add(commandFactory.moveFromPointToMidPointToPoint(secondNoteIntakePosition, moveBackFromCenterLineRighterMid, moveBackFromCenterLineRighterEnd));
        /* split, through the stage zone */
        commandSegments.add(waitForArmToBeInPosition);
        commandSegments.add(aimBot.splitForwardWhileMoving(
                new BezierCurve(moveBackFromCenterLineRighterEnd, splitSecondNote2, splitSecondNote3, splitSecondNote4),
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
                new BezierCurve(splitSecondNote4, splitSecondNote3, splitSecondNote2, moveBackFromCenterLineRighterEnd),
                rotation1, rotation1,
                splitTimeOut
        ));

        /* to the fourth note */
        commandSegments.add(commandFactory.moveFromPointToMidPointToPoint(moveBackFromCenterLineRighterEnd, moveBackFromCenterLineRighterMid, thirdNoteIntakePosition, rotation1, rotation2));
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

