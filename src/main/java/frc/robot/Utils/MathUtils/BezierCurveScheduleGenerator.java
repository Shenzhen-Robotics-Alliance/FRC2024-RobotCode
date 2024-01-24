package frc.robot.Utils.MathUtils;

import frc.robot.Utils.RobotConfigReader;

public class BezierCurveScheduleGenerator {
    private final double maximumVelocity, maximumAcceleration;
    public BezierCurveScheduleGenerator(RobotConfigReader robotConfig) {
        this.maximumVelocity = robotConfig.getConfig("auto/autoStageMaxVelocity");
        this.maximumAcceleration = robotConfig.getConfig("auto/autoStageMaxAcceleration");
    }

    public BezierCurveSchedule generateSchedule(Vector2D startingPoint, Vector2D endingPoint) {
        return generateSchedule(new BezierCurve(startingPoint, endingPoint));
    }

    public BezierCurveSchedule generateSchedule(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint) {
        return generateSchedule(new BezierCurve(startingPoint, midPoint, endingPoint));
    }

    public BezierCurveSchedule generateSchedule(BezierCurve path) {
        return new BezierCurveSchedule(maximumAcceleration, maximumVelocity, path);
    }
}
