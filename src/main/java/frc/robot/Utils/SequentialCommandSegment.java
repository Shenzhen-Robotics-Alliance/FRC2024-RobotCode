package frc.robot.Utils;
import frc.robot.Utils.MathUtils.AngleUtils;
import frc.robot.Utils.MathUtils.BezierCurve;
import frc.robot.Utils.MathUtils.Rotation2D;

/**
 *  Instead of using static paths and rotation goals, we make them dynamic.
 *  A path feeder and two rotation feeders are specified in the auto stage command segments.
 *  The robot decides, just before the command segment is executed, what path should be followed and what rotation should be faced
 * */
public class SequentialCommandSegment {// ^^ this class is for auto stage
    public final BezierCurveFeeder chassisMovementPathFeeder;
    public final Runnable beginning, periodic, ending;
    public final IsCompleteChecker isCompleteChecker;
    public final InitiateCondition initiateCondition;
    public final RotationFeeder startingRotationFeeder, endingRotationFeeder;
    public SequentialCommandSegment(InitiateCondition initiateCondition,  BezierCurveFeeder pathFeeder, Runnable beginning, Runnable periodic, Runnable ending, IsCompleteChecker isCompleteChecker, RotationFeeder startingRotation, RotationFeeder endingRotation) {
        this.chassisMovementPathFeeder = pathFeeder;

        this.beginning = beginning;
        this.periodic = periodic;
        this.ending = ending;

        this.isCompleteChecker = isCompleteChecker;
        this.initiateCondition = initiateCondition;

        this.startingRotationFeeder = startingRotation;
        this.endingRotationFeeder = endingRotation;
    }

    public double getStartingRotation() {
        return startingRotationFeeder.getRotation().getRadian();
    }

    public double getEndingRotation() {
        return endingRotationFeeder.getRotation().getRadian();
    }

    public double getMaxAngularVelocity() {
        return Math.abs(AngleUtils.getActualDifference(getStartingRotation(),  getEndingRotation()));
    }

    public BezierCurve getChassisMovementPath() {
        return chassisMovementPathFeeder.getBezierCurve();
    }

    public interface IsCompleteChecker {boolean isComplete();}
    public interface InitiateCondition {boolean initiateOrSkip();}
    public interface BezierCurveFeeder {BezierCurve getBezierCurve();}
    public interface RotationFeeder {Rotation2D getRotation();}

    /**
     * draw an instance of static command segment, using the feeders provided
     * this should be called right before the segment starts
     */
    public StaticSequentialCommandSegment embodyCurrentCommandSegment() {
        return new StaticSequentialCommandSegment(
                chassisMovementPathFeeder.getBezierCurve(),
                beginning,periodic,ending,isCompleteChecker,
                startingRotationFeeder.getRotation(),endingRotationFeeder.getRotation()
        );
    }
    /**
     * the static command segment generated from the feeders at the moment right before this segment start
     * */
    public static final class StaticSequentialCommandSegment {
        public final BezierCurve chassisMovementPath;
        public final Runnable beginning, periodic, ending;
        public final IsCompleteChecker isCompleteChecker;
        public final Rotation2D startingRotation, endingRotation;
        public StaticSequentialCommandSegment(BezierCurve chassisMovementPath, Runnable beginning, Runnable periodic, Runnable ending, IsCompleteChecker isCompleteChecker, Rotation2D startingRotation, Rotation2D endingRotation) {
            this.chassisMovementPath = chassisMovementPath;
            this.beginning = beginning;
            this.periodic = periodic;
            this.ending = ending;
            this.isCompleteChecker = isCompleteChecker;
            this.startingRotation = startingRotation;
            this.endingRotation = endingRotation;
        }

        public double getCurrentRotationWithLERP(double t) {
            if (startingRotation == null || endingRotation == null)
                throw new IllegalStateException("cannot obtain current rotation when the starting or ending rotation is not specified");
            if (t<0) t=0;
            else if (t>1) t=1;
            return AngleUtils.simplifyAngle(startingRotation.getRadian() + AngleUtils.getActualDifference(startingRotation.getRadian(), endingRotation.getRadian())*t);
        }
    }
}
