package frc.robot.Utils.ComputerVisionUtils;

import frc.robot.Drivers.Visions.TargetFieldPositionTracker;
import frc.robot.Utils.MathUtils.Vector2D;

import java.util.Map;

public class AprilTagReferredTarget {
    private final Map<Integer, Vector2D> aprilTagReferences;
    private final TargetFieldPositionTracker targetTracker;


    /**
     * @param targetTracker
     * @param aprilTagReferences the id of the reference tags, and their positions to the target (oriented to the field and in meters)
     */
    public AprilTagReferredTarget(TargetFieldPositionTracker targetTracker, Map<Integer, Vector2D> aprilTagReferences) {
        this.aprilTagReferences = aprilTagReferences;
        this.targetTracker = targetTracker;
    }

    public Vector2D getTargetFieldPositionWithVisibleAprilTags() {
        int visibleTargetCount = 0;
        Vector2D targetFieldPosition = new Vector2D();
        for (int id:aprilTagReferences.keySet()) {
            if (!targetTracker.isTargetVisible(id))
                continue;
            TargetFieldPositionTracker.TargetOnField target = targetTracker.getTargetByID(id);

            /* the target's field position is equal the current reference's position minus the reference's relative position to target */
            final Vector2D targetFieldPositionEstimatedByCurrentReference = target.fieldPosition.addBy(aprilTagReferences.get(id).multiplyBy(-1));
            targetFieldPosition = targetFieldPosition.addBy(targetFieldPositionEstimatedByCurrentReference);
            visibleTargetCount++;
        }
        if (visibleTargetCount == 0) return null;

        return targetFieldPosition.multiplyBy(1.0/visibleTargetCount);
    }

    public Vector2D getTargetFieldPositionWithAprilTags(long timeUnseenTolerance) {
        int visibleTargetCount = 0;
        Vector2D targetFieldPosition = new Vector2D();
        for (int id:aprilTagReferences.keySet()) {
            TargetFieldPositionTracker.TargetOnField target = targetTracker.getTargetByID(id);
            if (target == null || target.timeMillisSinceLastContact() > timeUnseenTolerance)
                continue;

            /* the target's field position is equal the current reference's position minus the reference's relative position to target */
            final Vector2D targetFieldPositionEstimatedByCurrentReference = target.fieldPosition.addBy(aprilTagReferences.get(id).multiplyBy(-1));
            targetFieldPosition = targetFieldPosition.addBy(targetFieldPositionEstimatedByCurrentReference);
            visibleTargetCount++;
        }
        if (visibleTargetCount == 0) return null;

        return targetFieldPosition.multiplyBy(1.0/visibleTargetCount);
    }
}
