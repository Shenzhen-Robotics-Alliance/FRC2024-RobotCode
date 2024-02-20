package frc.robot.Utils;

import frc.robot.Modules.PositionReader.PositionEstimator;
import frc.robot.Modules.Chassis.SwerveBasedChassis;
import frc.robot.RobotCore;
import frc.robot.Utils.MathUtils.BezierCurve;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;

public class SequentialCommandFactory {
    private final SwerveBasedChassis chassis;
    private final PositionEstimator positionEstimator;
    private final Vector2D robotStartingPosition;
    private final Rotation2D robotStartingRotation2D;

    public SequentialCommandFactory(RobotCore robotCore) {
        this(robotCore, new Vector2D(), new Rotation2D(0));
    }

    public SequentialCommandFactory(RobotCore robotCore, Vector2D robotStartingPosition, Rotation2D robotStartingRotation2D) {
        this.chassis = robotCore.chassisModule;
        this.positionEstimator = robotCore.positionReader;
        this.maintainCurrentRotation = () -> new Rotation2D(positionEstimator.getRobotRotation());
        this.robotStartingPosition = robotStartingPosition;
        this.robotStartingRotation2D = robotStartingRotation2D;
    }

    private static final SequentialCommandSegment.InitiateCondition justGo = () -> true;
    private static final Runnable doNothing = () -> {};
    private static final SequentialCommandSegment.IsCompleteChecker weDoNotCareAboutIsItComplete = () -> true;
    private final SequentialCommandSegment.RotationFeeder maintainCurrentRotation;
    private static final SequentialCommandSegment.RotationFeeder weDoNotCareAboutRotation = () -> null;

    public SequentialCommandSegment calibratePositionEstimator() {
        return justDoIt(() -> {
            positionEstimator.setRobotRotation(robotStartingRotation2D.getRadian());
            positionEstimator.setRobotPosition(robotStartingPosition);
        });
    }

    public SequentialCommandSegment moveToPointAndStop(Vector2D destination) {
        return moveToPointAndStopIf(justGo, destination);
    }
    public SequentialCommandSegment moveToPointAndStop(Vector2D destination, Runnable beginning, Runnable periodic, Runnable ending) {
        return moveToPointAndStopIf(justGo, destination, beginning, periodic, ending);
    }

    public SequentialCommandSegment moveToPointAndStopIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D destination) {
        return moveToPointAndStopIf(initiateCondition, destination, doNothing, doNothing, doNothing);
    }
    public SequentialCommandSegment moveToPointAndStopIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D destination, Runnable beginning, Runnable periodic, Runnable ending) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(positionEstimator.getRobotPosition2D(), destination),
                beginning, periodic, ending,
                chassis::isCurrentTranslationalTaskFinished,
                maintainCurrentRotation, maintainCurrentRotation
        );
    }

    public SequentialCommandSegment moveFromPointToPoint(Vector2D startingPoint, Vector2D endingPoint) {
        return moveFromPointToPointIf(justGo, startingPoint, endingPoint, doNothing, doNothing, doNothing, maintainCurrentRotation, maintainCurrentRotation);
    }

    public SequentialCommandSegment moveFromPointToPoint(Vector2D startingPoint, Vector2D endingPoint, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToPoint(startingPoint, endingPoint, doNothing, doNothing, doNothing, startingRotation, endingRotation);
    }

    public SequentialCommandSegment moveFromPointToPoint(Vector2D startingPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToPointIf(justGo, startingPoint, endingPoint, beginning, periodic, ending, () -> startingRotation, () -> endingRotation);
    }

    public SequentialCommandSegment moveFromPointToPointIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D startingPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, SequentialCommandSegment.RotationFeeder startingRotationFeeder, SequentialCommandSegment.RotationFeeder endingRotationFeeder) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(startingPoint, endingPoint),
                beginning, periodic, ending,
                weDoNotCareAboutIsItComplete,
                startingRotationFeeder, endingRotationFeeder
        );
    }

    public SequentialCommandSegment moveFromPointToMidPointToPoint(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint) {
        return moveFromPointToMidPointToPointIf(justGo, startingPoint, midPoint, endingPoint, doNothing, doNothing, doNothing, maintainCurrentRotation, maintainCurrentRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPoint(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToMidPointToPoint(startingPoint, midPoint, endingPoint, doNothing, doNothing, doNothing, startingRotation, endingRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPoint(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToMidPointToPointIf(justGo, startingPoint, midPoint, endingPoint, beginning, periodic, ending, () -> startingRotation, () -> endingRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPointIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, SequentialCommandSegment.RotationFeeder startingRotationFeeder, SequentialCommandSegment.RotationFeeder endingRotationFeeder) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(startingPoint, midPoint, endingPoint),
                beginning, periodic, ending,
                weDoNotCareAboutIsItComplete,
                startingRotationFeeder, endingRotationFeeder
        );
    }

    public SequentialCommandSegment moveFromPointToPointAndStop(Vector2D startingPoint, Vector2D endingPoint) {
        return moveFromPointToPointAndStopIf(justGo, startingPoint, endingPoint, doNothing, doNothing, doNothing, maintainCurrentRotation, maintainCurrentRotation);
    }

    public SequentialCommandSegment moveFromPointToPointAndStop(Vector2D startingPoint, Vector2D endingPoint, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToPointAndStop(startingPoint, endingPoint, doNothing, doNothing, doNothing, startingRotation, endingRotation);
    }

    public SequentialCommandSegment moveFromPointToPointAndStop(Vector2D startingPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToPointAndStopIf(justGo, startingPoint, endingPoint, beginning, periodic, ending, () -> startingRotation, () -> endingRotation);
    }

    public SequentialCommandSegment moveFromPointToPointAndStopIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D startingPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, SequentialCommandSegment.RotationFeeder startingRotationFeeder, SequentialCommandSegment.RotationFeeder endingRotationFeeder) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(startingPoint, endingPoint),
                beginning, periodic, ending,
                chassis::isCurrentTranslationalTaskFinished,
                startingRotationFeeder, endingRotationFeeder
        );
    }

    public SequentialCommandSegment moveFromPointToMidPointToPointAndStop(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint) {
        return moveFromPointToMidPointToPointAndStopIf(justGo, startingPoint, midPoint, endingPoint, doNothing, doNothing, doNothing, maintainCurrentRotation, maintainCurrentRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPointAndStop(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToMidPointToPointAndStop(startingPoint, midPoint, endingPoint, doNothing, doNothing, doNothing, startingRotation, endingRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPointAndStop(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToMidPointToPointAndStopIf(justGo, startingPoint, midPoint, endingPoint, beginning, periodic, ending, () -> startingRotation, () -> endingRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPointAndStopIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, SequentialCommandSegment.RotationFeeder startingRotationFeeder, SequentialCommandSegment.RotationFeeder endingRotationFeeder) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(startingPoint, midPoint, endingPoint),
                beginning, periodic, ending,
                chassis::isCurrentTranslationalTaskFinished,
                startingRotationFeeder, endingRotationFeeder
        );
    }


    public SequentialCommandSegment faceDirection(Rotation2D direction){
        return faceDirection(direction, doNothing, doNothing, doNothing);
    }

    public SequentialCommandSegment faceDirection(Rotation2D direction, Runnable beginning, Runnable periodic, Runnable ending) {

        return new SequentialCommandSegment(
                justGo,
                () -> null,
                beginning, periodic, ending,
                chassis::isCurrentRotationalTaskFinished,
                positionEstimator::getRobotRotation2D,
                () -> direction
        );
    }


    public SequentialCommandSegment justDoIt(Runnable job) {
        return new SequentialCommandSegment(
                justGo,
                () -> null,
                job,
                doNothing,
                doNothing,
                weDoNotCareAboutIsItComplete,
                weDoNotCareAboutRotation, weDoNotCareAboutRotation
        );
    }
}
