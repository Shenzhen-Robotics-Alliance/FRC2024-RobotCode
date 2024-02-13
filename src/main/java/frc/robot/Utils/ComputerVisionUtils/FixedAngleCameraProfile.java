package frc.robot.Utils.ComputerVisionUtils;

/**
 * camera profile for april tag camera installed in a fix angle
 */
public final class FixedAngleCameraProfile {
    /**
     * the installation angle of the camera, 0 for VERTICALLY DOWNWARDS, positive is to the upper side, in radians
     */
    public final double cameraInstallationAngleRadian,
    /**
     * how much angle of a target, in radian, is altered per unit change in amount of pixels of the object's center in the camera view
     */
    angleRadianPerCameraPixelX, angleRadianPerCameraPixelY;

    /**
     * create a profile for n april tag camera with fixed angle
     *
     * @param cameraInstallationAngleRadian       the installation angle of the camera, 0 for VERTICALLY DOWNWARDS, positive is to the upper side, in radians
     * @param angleRadianPerCameraXPixel          how much angle of a target, in radian, is altered per unit change in amount of pixels of the object's center in the camera view (x-axis), positive is counter-clockwise
     * @param angleRadianPerCameraYPixel          how much angle of a target, in radian, is altered per unit change in amount of pixels of the object's center in the camera view (y-axis), positive is up
     */
    public FixedAngleCameraProfile(double cameraInstallationAngleRadian, double angleRadianPerCameraXPixel, double angleRadianPerCameraYPixel) {
        this.cameraInstallationAngleRadian = cameraInstallationAngleRadian;
        this.angleRadianPerCameraPixelX = angleRadianPerCameraXPixel;
        this.angleRadianPerCameraPixelY = angleRadianPerCameraYPixel;
    }

    /**
     * get the distance from the camera to the target
     * @param yPixel the y pixel of the target center, 0 is center
     * @param targetHeightCM the height of the target to the robot
     * @return the distance to target, in cm, calculated from the y pixel of the target and the camera profile
     * */
    public double getDistanceFromYPixel(double yPixel, double targetHeightCM) {
        final double targetVerticalAngleRadian = angleRadianPerCameraPixelY * yPixel;
//        System.out.println("tgt y pix: " + yPixel);
//        System.out.println("cam inst ang: " + Math.toDegrees(cameraInstallationAngleRadian));
//        System.out.println("tag vert ang: " + Math.toDegrees(targetVerticalAngleRadian));
//        System.out.println("target vertical angle: " + Math.toDegrees(cameraInstallationAngleRadian + targetVerticalAngleRadian));
//        System.out.println("distance: " + targetHeightCM / Math.tan(cameraInstallationAngleRadian + targetVerticalAngleRadian));
        return targetHeightCM / Math.tan(cameraInstallationAngleRadian + targetVerticalAngleRadian);
    }

    /**
     * calculate the angle of the target to the middle line of the camera's view
     * @param pixelX the x position of pixel
     * @return
     */
    public double getTargetAngleRadianFromXPixel(double pixelX) {
        return pixelX * angleRadianPerCameraPixelX + (Math.PI / 2);
    }
}