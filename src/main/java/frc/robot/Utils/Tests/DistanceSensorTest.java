package frc.robot.Utils.Tests;

import frc.robot.Drivers.DistanceSensors.Rev2mDistanceSensorEncapsulation;

public class DistanceSensorTest implements SimpleRobotTest {
    private final Rev2mDistanceSensorEncapsulation testDistanceSensor = new Rev2mDistanceSensorEncapsulation();

    @Override
    public void testStart() {
        testDistanceSensor.setEnabled(true);
    }

    @Override
    public void testPeriodic() {
        System.out.println("distance sensor reads (cm): " + testDistanceSensor.getDistanceCM());
    }
}
