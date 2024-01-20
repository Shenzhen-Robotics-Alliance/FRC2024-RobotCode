package frc.robot.Utils;

import frc.robot.Utils.MathUtils.AngleUtils;
import frc.robot.Utils.MathUtils.BezierCurve;

public class SequentialCommandSegment {
    public final BezierCurve chassisMovementPath; // null for not required
    public final Runnable beginning, periodic, ending;
    public final IsCompleteChecker isCompleteChecker;
    public final double startingRotation, endingRotation, maxAngularVelocity;
    public SequentialCommandSegment(BezierCurve pathSchedule, Runnable beginning, Runnable periodic, Runnable ending, IsCompleteChecker isCompleteChecker, double startingRotation, double endingRotation) {
        this.chassisMovementPath = pathSchedule;
        this.beginning = beginning;
        this.periodic = periodic;
        this.ending = ending;
        this.isCompleteChecker = isCompleteChecker;

        this.startingRotation = startingRotation;
        this.endingRotation = endingRotation;
        this.maxAngularVelocity = Math.abs(AngleUtils.getActualDifference(startingRotation, endingRotation));
    }

    public double getCurrentRotationWithLERP(double t) {
        if (t<0) t=0;
        else if (t>1) t=1;
        return AngleUtils.simplifyAngle(startingRotation + AngleUtils.getActualDifference(startingRotation, endingRotation)*t);
    }

    public interface IsCompleteChecker {
        /** check whether this checkpoint */
        boolean isComplete();
    }
}