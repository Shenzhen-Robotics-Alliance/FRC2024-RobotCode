package frc.robot.Utils.ComputerVisionUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PhantomClient {
    private final NetworkTableInstance networkTableInstance;
    private final NetworkTableEntry robot_odometer_x_entry, robot_odometer_y_entry, robot_rotation_entry, robot_position_x, robot_position_y;

    private Pose2d previousPosition;

    public PhantomClient(String server) {
        networkTableInstance = NetworkTableInstance.create();
        networkTableInstance.setServer(server);
        networkTableInstance.startClient3("robot");
        this.robot_odometer_x_entry = networkTableInstance.getTable("Vision").getEntry("robot_odometer_x");
        this.robot_odometer_y_entry = networkTableInstance.getTable("Vision").getEntry("robot_odometer_y");
        this.robot_rotation_entry = networkTableInstance.getTable("Vision").getEntry("robot_rotation");
        this.robot_position_x = networkTableInstance.getTable("Vision").getEntry("robot_position_x");
        this.robot_position_y = networkTableInstance.getTable("Vision").getEntry("robot_position_y");
    }

    public void update(Pose2d robot_odometer_position) {
        previousPosition = robot_odometer_position;
        this.robot_odometer_x_entry.setDouble(robot_odometer_position.getX());
        this.robot_odometer_y_entry.setDouble(robot_odometer_position.getY());

        this.robot_rotation_entry.setDouble(robot_odometer_position.getRotation().getRadians());
        networkTableInstance.flush();
    }

    public Pose2d getRobotPose() {
        networkTableInstance.flush();
//        if (!networkTableInstance.isConnected())
//            System.out.println("<-- waiting for vision to start, using odometry for now ... -->");
//        else
//            System.out.println("<-- pulling results from server -->");
        return new Pose2d(
                robot_position_x.getDouble(previousPosition.getX()),
                robot_position_y.getDouble(previousPosition.getY()),
                previousPosition.getRotation()
        );
    }
}
