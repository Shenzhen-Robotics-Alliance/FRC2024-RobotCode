package frc.robot.Drivers.Visions;

import frc.robot.Utils.ComputerVisionUtils.FixedAngleCameraProfile;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;

import java.util.ArrayList;
import java.util.List;

public class FixedAnglePositionTrackingCamera implements TargetFieldPositionTracker {
    private final List<TargetOnField> targets;
    private List<TargetOnField> visibleTargets;
    private final RawObjectDetectionCamera camera;
    private final FixedAngleCameraProfile cameraProfile;
    private final double[] targetHeights;
    public FixedAnglePositionTrackingCamera(RawObjectDetectionCamera camera, FixedAngleCameraProfile cameraProfile, double[] targetHeights) {
        this.camera = camera;
        this.cameraProfile = cameraProfile;
        this.targetHeights = targetHeights;

        targets = new ArrayList<>();
        visibleTargets = new ArrayList<>();
    }

    @Override
    public void update(Vector2D robotPositionInField2D, Rotation2D robotRotation) {
        camera.update();
        visibleTargets = new ArrayList<>();
        for (RawObjectDetectionCamera.ObjectTargetRaw targetRaw: camera.getRawTargets()) {
            // System.out.println("<-- Fixed Angle Camera | working on target " + targetRaw.id +  " -->");
            double targetHeight = targetRaw.id < targetHeights.length ? targetHeights[targetRaw.id] : targetHeights[0];
            final double targetDistance = cameraProfile.getDistanceFromYPixel(targetRaw.y, targetHeight) / 100.0f,
                    targetDirection = cameraProfile.getTargetAngleRadianFromXPixel(targetRaw.x);
            // System.out.println("<-- Fixed Angle Camera | target distance: " + targetDistance + " -->");
            // System.out.println("<-- Fixed Angle Camera | target angle: " + (Math.toDegrees(targetDirection)-90) + " -->");
            final Vector2D relativePositionToCamera = new Vector2D(targetDirection ,targetDistance),
                    fieldPositionDifferenceFromCamera = relativePositionToCamera.multiplyBy(robotRotation),
                    targetFieldPosition2D =  robotPositionInField2D.addBy(fieldPositionDifferenceFromCamera);
            final TargetOnField target = new TargetOnField(targetRaw.id, targetFieldPosition2D);
            deleteTargetFromTargets(targetRaw.id);
            targets.add(target);
            visibleTargets.add(target);
        }
    }

    private void deleteTargetFromTargets(int targetID) {
        boolean flag = true;
        while (flag) {
            flag = false;
            int index = -1;
            for (int i = 0; i < targets.size(); i++)
                if (targets.get(i).id == targetID) {
                    flag = true;
                    index = i;
                }
            if (flag) targets.remove(index);
        }
    }


    @Override
    public List<TargetOnField> getAllTargets() {
        return targets;
    }

    @Override
    public List<TargetOnField> getVisibleTargets() {
        return visibleTargets;
    }
}
