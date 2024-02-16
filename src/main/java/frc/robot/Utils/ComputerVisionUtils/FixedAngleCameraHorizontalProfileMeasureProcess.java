package frc.robot.Utils.ComputerVisionUtils;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.Visions.RawObjectDetectionCamera;
import frc.robot.Utils.MathUtils.StatisticsUtils;
import frc.robot.Utils.TimeUtils;

public class FixedAngleCameraHorizontalProfileMeasureProcess {
    private static final int horizontalDistanceLevelsCount = 3;
    private static final int horizontalDistanceSamplesCount = horizontalDistanceLevelsCount * 2 - 1;
    private final RawObjectDetectionCamera camera;
    private final XboxController controller;
    private final int targetID;
    private final double[] angleSamples, pixelXSamples, horizontalDistanceFromCenterSamples, distanceSamples;

    private int currentHorizontalDistanceSample = horizontalDistanceSamplesCount;
    private int currentDistanceSample = -1;
    private boolean finished;
    public FixedAngleCameraHorizontalProfileMeasureProcess(RawObjectDetectionCamera camera, double[] distances, double maxHorizontalDistanceFromCenter, int targetID, XboxController controller) {
        this.camera = camera;
        this.controller = controller;
        this.targetID = targetID;
        this.distanceSamples = distances;

        this.angleSamples = new double[distances.length * horizontalDistanceSamplesCount];
        this.pixelXSamples = new double[distances.length * horizontalDistanceSamplesCount];
        this.horizontalDistanceFromCenterSamples = new double[horizontalDistanceSamplesCount];

        final double unitSpace = (maxHorizontalDistanceFromCenter) / (horizontalDistanceLevelsCount-1);
        double horizontalDistance = -maxHorizontalDistanceFromCenter - unitSpace;
        for (int i = 0; i < horizontalDistanceSamplesCount; i++) {
            horizontalDistanceFromCenterSamples[i] = horizontalDistance += unitSpace;
        }

        finished = false;

        camera.startRecognizing();
        nextSample();
    }


    int i =0;
    public void measuringPeriodic() {
        if (finished) {
            TimeUtils.sleep(50);
            return;
        }

        camera.update();
        if (controller.getAButton()) targetInPlace();
        putDataOnDashBoard();
    }

    private void putDataOnDashBoard() {
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

    private void targetInPlace() {
        RawObjectDetectionCamera.ObjectTargetRaw targetRaw;
        if ((targetRaw = camera.getRawTargetsByID(targetID)) == null) {
            System.out.println("<-- Profile Measuring | target not seen, please try again -->");
            TimeUtils.sleep(200);
            return;
        }

        final double horizontalDistanceToCenter = horizontalDistanceFromCenterSamples[currentHorizontalDistanceSample],
                distance = distanceSamples[currentDistanceSample];
        pixelXSamples[i] = targetRaw.x;
        angleSamples[i++] = Math.atan(horizontalDistanceToCenter / distance);
        System.out.println("<-- target successfully recorded: " + targetRaw + "-->");
        nextSample();
        TimeUtils.sleep(200);
    }

    private void nextSample() {
        if (++this.currentHorizontalDistanceSample >= horizontalDistanceSamplesCount) {
            currentHorizontalDistanceSample = 0;
            currentDistanceSample++;
            if (this.currentDistanceSample >= distanceSamples.length) {
                finished = true;
                printResults();
                return;
            }
            System.out.println("\n\n\n<-- Profile Measuring | measuring distance "+ (currentDistanceSample+1) + "/" + distanceSamples.length + ", please put the target " + (int)distanceSamples[currentDistanceSample] + " cm away from the camera -->");
        }

        System.out.println("\n<-- Profile Measuring | measuring sample " + (currentHorizontalDistanceSample +1) + "/" + horizontalDistanceSamplesCount + ", please put the target " + (int)horizontalDistanceFromCenterSamples[currentHorizontalDistanceSample] + " cm horizontally away from the camera center -->");
        System.out.println("<-- Profile Measuring | press A if the target is in position -->");
    }

    private void printResults() {
        final double cameraAngleRadianPerPixel = StatisticsUtils.getBestFitLineSlope(pixelXSamples, angleSamples),
                cameraFacingBiasRadian = StatisticsUtils.getBestFitLineIntersect(pixelXSamples, angleSamples);
        System.out.println("Camera Angle Radian per Pixel (x): " + cameraAngleRadianPerPixel);
        System.out.println("Camera Facing Angle bias Radian: " + cameraFacingBiasRadian);
        System.out.println("Data Coefficient of Determination: " + Math.pow(StatisticsUtils.getCorrelationCoefficient(pixelXSamples, angleSamples), 2));
    }
}
