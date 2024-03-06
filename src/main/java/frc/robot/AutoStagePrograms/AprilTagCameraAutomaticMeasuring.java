package frc.robot.AutoStagePrograms;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.Visions.RawObjectDetectionCamera;
import frc.robot.Modules.PositionReader.PositionEstimator;
import frc.robot.Modules.UpperStructure.TransformableArm;
import frc.robot.RobotCore;
import frc.robot.Utils.CommandSequenceGenerator;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.StatisticsUtils;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.List;

public class AprilTagCameraAutomaticMeasuring implements CommandSequenceGenerator {
    private final RawObjectDetectionCamera camera;
    private final Vector2D robotInitialPositionToAprilTag;
    private final int targetID;
    private final double minDistance, maxDistance, maxHorizontalAngleRadians, targetHeightFromCamera;

    private static final int horizontalAngleLevelsCount = 3;
    private static final int horizontalAngleSamplesCount = horizontalAngleLevelsCount * 2 - 1;
    private static final int verticalDistanceLevelsCount = 4;

    private final List<Double> pixelXSamples, angleXSamples, pixelYSamples, angleYSamples;
    private final Rotation2D cameraFacing;
    public AprilTagCameraAutomaticMeasuring(RawObjectDetectionCamera camera, int targetID, double targetHeight, double minDistance, double maxDistance, double maxHorizontalAngleDegrees, Vector2D robotInitialPositionToTarget) {
        this(camera, targetID, targetHeight, new Rotation2D(0), minDistance, maxDistance, maxHorizontalAngleDegrees, robotInitialPositionToTarget);
    }
    public AprilTagCameraAutomaticMeasuring(RawObjectDetectionCamera camera, int targetID, double targetHeight, Rotation2D cameraFacing, double minDistance, double maxDistance, double maxHorizontalAngleDegrees, Vector2D robotInitialPositionToTarget) {
        this.camera = camera;
        this.targetID = targetID;
        this.minDistance = minDistance;
        this.maxDistance = maxDistance;
        this.maxHorizontalAngleRadians = Math.toRadians(maxHorizontalAngleDegrees);
        this.targetHeightFromCamera = targetHeight;
        this.cameraFacing = cameraFacing;

        this.robotInitialPositionToAprilTag = robotInitialPositionToTarget.multiplyBy(1/100.0);

        pixelXSamples = new ArrayList<>();
        angleXSamples = new ArrayList<>();
        pixelYSamples = new ArrayList<>();
        angleYSamples = new ArrayList<>();
    }
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore);
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();

        commandSegments.add(commandFactory.justDoIt(camera::startRecognizing));
        commandSegments.add(commandFactory.justDoIt(() -> robotCore.transformableArm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.INTAKE, null)));

        Vector2D robotPreviousPosition = robotInitialPositionToAprilTag;
        Rotation2D robotPreviousFacing = new Rotation2D(0);
        final double unitAngle = maxHorizontalAngleRadians / (horizontalAngleLevelsCount -1);
        for (int currentDistanceSample = 0; currentDistanceSample < verticalDistanceLevelsCount; currentDistanceSample++) {
            double horizontalAngleRadian = -maxHorizontalAngleRadians - unitAngle;
            double verticalDistance = minDistance + currentDistanceSample * (maxDistance - minDistance) / verticalDistanceLevelsCount;

            final Vector2D targetedPosition = new Vector2D(new double[] {
                    0, -verticalDistance / 100 // notice here the robot moves in meters, so we need to divide it by 100
            });
            commandSegments.add(commandFactory.moveFromPointToPointAndStop(
                    toFieldPosition(robotPreviousPosition),
                    toFieldPosition(targetedPosition),
                    () -> System.out.println("moving to position"),
                    this::putDataOnDashBoard,
                    () -> {},
                    robotPreviousFacing, new Rotation2D(horizontalAngleRadian + unitAngle)
            ));

            System.out.println("<-- scheduling | going to position: " + targetedPosition + " -->");

            robotPreviousPosition = targetedPosition;
            for (int currentHorizontalDistanceSample = 0; currentHorizontalDistanceSample < horizontalAngleSamplesCount; currentHorizontalDistanceSample++) {
                System.out.println("<-- scheduling | face rotation: " + Math.toDegrees(horizontalAngleRadian) + " -->");
                commandSegments.add(commandFactory.faceDirection(
                        new Rotation2D(horizontalAngleRadian += unitAngle),
                        () -> System.out.println("turning to rotation: " + Math.toDegrees(robotCore.chassisModule.getCurrentRotationalTask().rotationalValue)),
                        this::putDataOnDashBoard,
                        () -> targetInPlace(robotCore, robotCore.positionReader)
                ));
            }
            robotPreviousFacing = new Rotation2D(maxHorizontalAngleRadians);
        }

        commandSegments.add(commandFactory.justDoIt(this::printResults));

        return commandSegments;
    }

    private Vector2D toFieldPosition(Vector2D aprilTagNavigatedPosition) {
        return aprilTagNavigatedPosition.multiplyBy(cameraFacing).addBy(robotInitialPositionToAprilTag.multiplyBy(-1));
    }

    private Vector2D getAprilTagRelativePositionFromRobotView(Vector2D robotFieldPosition, Rotation2D robotFacing) {
        final Vector2D robotRelativePositionToAprilTag = robotFieldPosition.addBy(robotInitialPositionToAprilTag),
                aprilTagRelativePositionToRobot = robotRelativePositionToAprilTag.multiplyBy(-1);
        final Rotation2D cameraRotation = cameraFacing.add(robotFacing);
        return aprilTagRelativePositionToRobot.multiplyBy(cameraRotation.getReversal());
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

    private void targetInPlace(RobotCore robotCore, PositionEstimator robotPositionEstimator) {
        long t0 = System.currentTimeMillis();
        while (camera.getRawTargetsByID(targetID) == null && System.currentTimeMillis() - t0 < 500) {
            robotCore.updateModules();
            camera.update();
        }
        RawObjectDetectionCamera.ObjectTargetRaw targetRaw;
        if ((targetRaw = camera.getRawTargetsByID(targetID)) == null) {
            System.out.println("<-- Profile Measuring | target not seen in this position, skipping... -->");
            return;
        }

        final Vector2D aprilTagPositionToRobot = getAprilTagRelativePositionFromRobotView(robotPositionEstimator.getRobotPosition2D(), robotPositionEstimator.getRobotRotation2D()).multiplyBy(100); // convert to cm
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

                cameraAngleRadianPerPixelY = -StatisticsUtils.getBestFitLineSlope(pixelYSamples, angleYSamples),
                cameraInstallationVerticalAngle = -StatisticsUtils.getBestFitLineIntersect(pixelYSamples, angleYSamples),
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
