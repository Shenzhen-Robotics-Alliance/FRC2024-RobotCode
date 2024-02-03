package frc.robot.Utils;

import frc.robot.Modules.PositionReader.PositionEstimator;
import frc.robot.Modules.Chassis.SwerveBasedChassis;
import frc.robot.RobotCore;
import frc.robot.Utils.MathUtils.BezierCurve;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;

public class SequentialCommandFactory {// this class is for auto stage
    private final SwerveBasedChassis chassis;
    private final PositionEstimator positionEstimator;
    public SequentialCommandFactory(RobotCore robotCore) {
        this(robotCore.chassisModule, robotCore.positionReader);
    }
    public SequentialCommandFactory(SwerveBasedChassis chassis, PositionEstimator positionEstimator) {
        this.chassis = chassis;
        this.positionEstimator = positionEstimator;
        this.maintainCurrentRotation = () -> new Rotation2D(positionEstimator.getRobotRotation());
    }

    private static final SequentialCommandSegment.InitiateCondition justGo = () -> true;
    private static final Runnable doNothing = () -> {};
    private static final SequentialCommandSegment.IsCompleteChecker weDoNotCareAboutIsItComplete = () -> true;
    private final SequentialCommandSegment.RotationFeeder maintainCurrentRotation;
    private static final SequentialCommandSegment.RotationFeeder weDoNotCareAboutRotation = () -> null;

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

    public SequentialCommandSegment moveFromPointToPoint(Vector2D startingPoint, Vector2D middlePoint, Vector2D endingPoint) {
        return moveFromPointToPoint(startingPoint, middlePoint, endingPoint, doNothing, doNothing, doNothing);
    }

    public SequentialCommandSegment moveFromPointToPoint(Vector2D startingPoint, Vector2D middlePoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending) {
        return new SequentialCommandSegment(
                justGo,
                () -> new BezierCurve(startingPoint, middlePoint, endingPoint),
                beginning, periodic, ending,
                weDoNotCareAboutIsItComplete,
                maintainCurrentRotation, maintainCurrentRotation
        );
    }

    public SequentialCommandSegment moveFromPointToPointAndStop(Vector2D startingPoint, Vector2D middlePoint, Vector2D endingPoint) {
        return moveFromPointToPointAndStop(startingPoint, middlePoint, endingPoint, doNothing, doNothing, doNothing);
    }

    public SequentialCommandSegment moveFromPointToPointAndStop(Vector2D startingPoint, Vector2D endingPoint) {
        return moveFromPointToPointAndStop(startingPoint, startingPoint.addBy(Vector2D.displacementToTarget(startingPoint, endingPoint).multiplyBy(1.0/2)), endingPoint, doNothing, doNothing, doNothing);
    }
    public SequentialCommandSegment moveFromPointToPointAndStop(Vector2D startingPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending) {
        return moveFromPointToPointAndStop(startingPoint, startingPoint.addBy(Vector2D.displacementToTarget(startingPoint, endingPoint).multiplyBy(1.0/2)), endingPoint, beginning, periodic, ending);
    }

    public SequentialCommandSegment moveFromPointToPointAndStop(Vector2D startingPoint, Vector2D middlePoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending) {
        return new SequentialCommandSegment(
                justGo,
                () -> new BezierCurve(startingPoint, middlePoint, endingPoint),
                beginning, periodic, ending,
                chassis::isCurrentTranslationalTaskFinished,
                maintainCurrentRotation, maintainCurrentRotation
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
