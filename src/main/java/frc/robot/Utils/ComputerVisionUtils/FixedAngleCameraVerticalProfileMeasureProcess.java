package frc.robot.Utils.ComputerVisionUtils;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.Visions.RawObjectDetectionCamera;
import frc.robot.Utils.MathUtils.StatisticsUtils;
import frc.robot.Utils.Time;

public class FixedAngleCameraVerticalProfileMeasureProcess {
    private final RawObjectDetectionCamera camera;
    private final XboxController controller;
    private final int totalSampleSize, targetID;
    private final double[] distanceSamples, angleSamples, pixelYSamples;
    private final double cameraInstallationHeightCM, maxDistanceToTarget, minDistanceToTarget;

    private int currentSample = -1;
    private boolean finished;
    public FixedAngleCameraVerticalProfileMeasureProcess(RawObjectDetectionCamera camera, int totalSampleSize, double cameraInstallationHeightCM, double minDistanceToTarget, double maxDistanceToTarget, int targetID, XboxController controller) {
        this.camera = camera;
        this.controller = controller;
        this.totalSampleSize = totalSampleSize;
        this.cameraInstallationHeightCM = cameraInstallationHeightCM;
        this.maxDistanceToTarget = maxDistanceToTarget;
        this.minDistanceToTarget = minDistanceToTarget;
        this.targetID = targetID;

        this.distanceSamples = new double[totalSampleSize];
        this.angleSamples = new double[totalSampleSize];
        this.pixelYSamples = new double[totalSampleSize];
        for (int i = 0; i < totalSampleSize; i++) {
            distanceSamples[i] = minDistanceToTarget + i * (maxDistanceToTarget - minDistanceToTarget) / totalSampleSize;
            angleSamples[i] = Math.atan(cameraInstallationHeightCM / distanceSamples[i]);
        }
        finished = false;

        camera.startRecognizing();
        nextSample();
    }

    public void measuringPeriodic() {
        if (finished) {
            Time.sleep(50);
            return;
        }

        camera.update();
        if (controller.getAButton()) targetInPlace();
        putResultOnDashBoard();
    }

    private void putResultOnDashBoard() {
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
            Time.sleep(200);
            return;
        }

        pixelYSamples[currentSample] = targetRaw.y;
        System.out.println("<-- target successfully recorded: " + targetRaw + "-->");
        nextSample();
        Time.sleep(200);
    }

    private void nextSample() {
        if (++this.currentSample >= totalSampleSize) {
            finished = true;
            printResults();
            return;
        }

        System.out.println("\n<-- Profile Measuring | measuring sample " + (currentSample+1) + "/" + totalSampleSize + ", please put the target " + (int)distanceSamples[currentSample] + " cm away from the camera -->");
        System.out.println("<-- Profile Measuring | press A if the target is in position -->");
    }

    private void printResults() {
        final double cameraAngleRadianPerPixel = StatisticsUtils.getBestFitLineSlope(pixelYSamples, angleSamples),
                cameraInstallationAngleRadian = StatisticsUtils.getBestFitLineIntersect(pixelYSamples, angleSamples);
        System.out.println("Camera Angle (Radian) per Pixel: " + cameraAngleRadianPerPixel);
        System.out.println("CameraInstallationAngle (radian): " + cameraInstallationAngleRadian);
        System.out.println("data Coefficient of Determination: " + Math.pow(StatisticsUtils.getCorrelationCoefficient(pixelYSamples, angleSamples), 2));
    }
}
