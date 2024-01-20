package frc.robot.Drivers.Visions;

import java.util.List;

public interface RawObjectDetectionCamera {
    void startRecognizing();
    void stopRecognizing();
    void update();
    List<ObjectTargetRaw> getRawTargets();

    /**
     * gets an april tag target by its id
     * @param id the id of the desired april tag
     * @return the april tag raw target if seen, or null if unseen
     * */
    default ObjectTargetRaw getRawTargetsByID(int id) {
        try {
            for (ObjectTargetRaw targetRaw : getRawTargets())
                if (targetRaw.id == id) return targetRaw;
        } catch (NullPointerException ignored) {}
        return null;
    }

    final class ObjectTargetRaw {
        public final int id;
        /** in reference to the center of the camera view */
        public final double x, y, areaReflected;
        public ObjectTargetRaw(int id, double x, double y, double areaReflected) {
            this.id = id;
            this.x = x;
            this.y = y;
            this.areaReflected = areaReflected;
        }

        @Override
        public String toString() {
            return "Raw Target (id:" + id + ") at (" + (int)x + "," + (int)y + ") with area: " + areaReflected;
        }
    }
}
