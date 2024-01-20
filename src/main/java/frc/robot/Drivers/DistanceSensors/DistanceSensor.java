package frc.robot.Drivers.DistanceSensors;

public interface DistanceSensor {
    /**
     * get the newest distance sensor update
     * @return the distance of the obstacle ahead, in CM, -1 for invalid
     * */
    double getDistanceCM();
    default void setEnabled(boolean enabled) {}
}
