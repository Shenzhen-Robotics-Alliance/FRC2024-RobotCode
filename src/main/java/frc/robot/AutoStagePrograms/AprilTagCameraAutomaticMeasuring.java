package frc.robot.AutoStagePrograms;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.Visions.RawObjectDetectionCamera;
import frc.robot.Modules.PositionReader.PositionEstimator;
import frc.robot.Robot;
import frc.robot.Utils.MathUtils.BezierCurve;
import frc.robot.Utils.MathUtils.StatisticsUtils;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.SequentialCommandSegment;
import frc.robot.Utils.Time;

import java.util.ArrayList;
import java.util.List;

public class AprilTagCameraAutomaticMeasuring implements AutoStageProgram{
    private final RawObjectDetectionCamera camera;
    private final Vector2D robotInitialPositionToAprilTag;
    private final int targetID;
    private final double minDistance, maxDistance, maxHorizontalDistance, targetHeightFromCamera;

    private static final int horizontalDistanceLevelsCount = 3;
    private static final int horizontalDistanceSamplesCount = horizontalDistanceLevelsCount * 2 - 1;
    private static final int verticalDistanceLevelsCount = 4;

    private final List<Double> pixelXSamples, angleXSamples, pixelYSamples, angleYSamples;
    public AprilTagCameraAutomaticMeasuring(RawObjectDetectionCamera camera, int targetID, double targetHeight, double minDistance, double maxDistance, double maxHorizontalDistance, Vector2D robotInitialPositionToTarget) {
        this.camera = camera;
        this.targetID = targetID;
        this.minDistance = minDistance;
        this.maxDistance = maxDistance;
        this.maxHorizontalDistance = maxHorizontalDistance;
        this.targetHeightFromCamera = targetHeight;

        this.robotInitialPositionToAprilTag = robotInitialPositionToTarget.multiplyBy(1/100.0);

        pixelXSamples = new ArrayList<>();
        angleXSamples = new ArrayList<>();
        pixelYSamples = new ArrayList<>();
        angleYSamples = new ArrayList<>();
    }
    @Override
    public List<SequentialCommandSegment> getCommandSegments(Robot robot) {
        List<SequentialCommandSegment> commandSegments = new ArrayList<>();

        commandSegments.add(new SequentialCommandSegment(
                null,
                camera::startRecognizing,
                () -> {},
                () -> {},
                () -> true,
                0, 0
        ));

        Vector2D robotPreviousPosition = robotInitialPositionToAprilTag;
        final double unitSpace = maxHorizontalDistance / (horizontalDistanceLevelsCount-1);
        for (int currentDistanceSample = 0; currentDistanceSample < verticalDistanceLevelsCount; currentDistanceSample++) {
            double horizontalDistance = -maxHorizontalDistance - unitSpace;
            double verticalDistance = minDistance + currentDistanceSample * (maxDistance - minDistance) / verticalDistanceLevelsCount;
            for (int currentHorizontalDistanceSample = 0; currentHorizontalDistanceSample < horizontalDistanceSamplesCount; currentHorizontalDistanceSample++) {
                final Vector2D targetedPosition = new Vector2D(new double[] {
                        -horizontalDistance / 100, -verticalDistance / 100 // notice here the robot moves in meters, so we need to divide it by 100
                });
                System.out.println("segment with ending position" + targetedPosition);
                commandSegments.add(new SequentialCommandSegment(
                        new BezierCurve(toFieldPosition(robotPreviousPosition), toFieldPosition(targetedPosition)),
                        () -> {
                            System.out.println("initiate of segment...");
                        },
                        this::putDataOnDashBoard,
                        () -> targetInPlace(robot.positionReader),
                        robot.chassisModule::isCurrentTranslationalTaskFinished,
                        0, 0
                ));
                robotPreviousPosition = targetedPosition;
                horizontalDistance += unitSpace;
            }
        }

        commandSegments.add(new SequentialCommandSegment(
                new BezierCurve(toFieldPosition(robotPreviousPosition), toFieldPosition(robotInitialPositionToAprilTag)),
                this::printResults,
                () -> {},
                () -> {},
                () -> true,
                0, 0
        ));

        return commandSegments;
    }

    private Vector2D toFieldPosition(Vector2D aprilTagNavigatedPosition) {
        return aprilTagNavigatedPosition.addBy(robotInitialPositionToAprilTag.multiplyBy(-1));
    }

    private Vector2D getAprilTagRelativePositionFromRobotView(Vector2D robotFieldPosition) {
        return robotFieldPosition.addBy(robotInitialPositionToAprilTag).multiplyBy(-1);
    }

    private void putDataOnDashBoard() {
        camera.update();
        RawObjectDetectionCamera.ObjectTargetRaw targetRaw;
        double x,y;
        if ((targetRaw = camera.getRawTargetsByID(targetID)) == null)
            x = y = -114514;
        else {
            x = targetRaw.x;
            y = targetRaw.y;
        }
        SmartDashboard.putNumber("fixed angle camera target X", x);
        SmartDashboard.putNumber("fixed angle camera target Y", y);
    }

    private void targetInPlace(PositionEstimator robotPositionEstimator) {
        long t0 = System.currentTimeMillis();
        while (camera.getRawTargetsByID(targetID) == null && System.currentTimeMillis() - t0 < 500) {
            camera.update();
            Time.sleep(100);
        }
        RawObjectDetectionCamera.ObjectTargetRaw targetRaw;
        if ((targetRaw = camera.getRawTargetsByID(targetID)) == null) {
            System.out.println("<-- Profile Measuring | target not seen in this position, skipping... -->");
            return;
        }

        final Vector2D aprilTagPositionToRobot = getAprilTagRelativePositionFromRobotView(robotPositionEstimator.getRobotPosition2D()).multiplyBy(100); // convert to cm
        pixelXSamples.add(targetRaw.x);
        angleXSamples.add(Math.toRadians(90) - aprilTagPositionToRobot.getHeading());

        pixelYSamples.add(targetRaw.y);
        angleYSamples.add(Math.atan(-targetHeightFromCamera / aprilTagPositionToRobot.getY()));
        System.out.println("<-- target successfully recorded: " + targetRaw + ", chassis position:" + robotPositionEstimator.getRobotPosition2D() + " -->");
        System.out.println("pixel X : " + targetRaw.x);
        System.out.println("angle X : " + Math.toDegrees(Math.toRadians(90) - aprilTagPositionToRobot.getHeading()));
        System.out.println("pixel Y : " + targetRaw.y);
        System.out.println("angle Y : " + Math.toDegrees(Math.atan(-targetHeightFromCamera / aprilTagPositionToRobot.getY())));
    }

    private void printResults() {
        final double[] pixelXSamples = StatisticsUtils.toArray(this.pixelXSamples),
                angleXSamples = StatisticsUtils.toArray(this.angleXSamples),
                pixelYSamples = StatisticsUtils.toArray(this.pixelYSamples),
                angleYSamples = StatisticsUtils.toArray(this.angleYSamples);
        final double cameraAngleRadianPerPixelX = StatisticsUtils.getBestFitLineSlope(pixelXSamples, angleXSamples),
                cameraFacingAngleBias = StatisticsUtils.getBestFitLineIntersect(pixelXSamples, angleXSamples),
                r_squared1 = Math.pow(StatisticsUtils.getCorrelationCoefficient(pixelXSamples, angleXSamples), 2),

                cameraAngleRadianPerPixelY = StatisticsUtils.getBestFitLineSlope(pixelYSamples, angleYSamples),
                cameraInstallationVerticalAngle = StatisticsUtils.getBestFitLineIntersect(pixelYSamples, angleYSamples),
                r_squared2 = Math.pow(StatisticsUtils.getCorrelationCoefficient(pixelYSamples, angleYSamples), 2);

        System.out.println("Camera Angle Radian per Pixel (x): " + cameraAngleRadianPerPixelX);
        System.out.println("Camera Facing Angle bias Radian: " + cameraFacingAngleBias);
        System.out.println("Data Coefficient of Determination: " + r_squared1);

        System.out.println("Camera Angle Radian per Pixel (Y): " + cameraAngleRadianPerPixelY);
        System.out.println("Camera Installation Vertical Angle: " + cameraInstallationVerticalAngle);
        System.out.println("Data Coefficient of Determination: " + r_squared2);
        /*
        * results:
        * camera angle radian per pixel (x) = 0.002167
        * camera facing angle bias = -0.0331
        * r^2 = 0.945
        *
        * camera angle radian per pixel (y) = 0.001342
        * camera installation vertical angle: -0.58288
        * r^2 = 0.988
        * */
    }
}
