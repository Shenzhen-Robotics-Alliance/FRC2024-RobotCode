package frc.robot.Utils.MathUtils;

public class BezierCurveSchedule {
    private final double chassisAccelerationConstrain, chassisVelocityConstrain;
    private final BezierCurve path;
    private double t;
    public BezierCurveSchedule(double chassisAccelerationConstrain, double chassisVelocityConstrain, BezierCurve path) {
        this.chassisAccelerationConstrain = chassisAccelerationConstrain;
        this.chassisVelocityConstrain = chassisVelocityConstrain;
        this.path = path;
        t = 0;
    }

    /**
     * @param dt in seconds
     * @return the t value, scaled
     * */
    public double nextCheckPoint(double dt) {
        /* we need to scale the time by a factor such that the velocity and acceleration both don't exceed our constraint */

        /** the velocity and acceleration at the current point on the path, if we don't scale the time  */
        final double currentPointOriginalVelocity = path.getVelocityWithLERP(t).getMagnitude(),
                currentPointOriginalAcceleration = path.getAccelerationWithLERP(t).getMagnitude(),
                timeScaleFactor = Math.min(chassisVelocityConstrain / currentPointOriginalVelocity, chassisAccelerationConstrain / currentPointOriginalAcceleration);
        return t += dt * timeScaleFactor;
    }

    public Vector2D getPositionWithLERP() {
        return path.getPositionWithLERP(t);
    }

    public boolean isCurrentPathFinished() {
        return t >= 1;
    }
}
