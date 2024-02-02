package frc.robot.Utils.Tests;

import frc.robot.Drivers.DistanceSensors.Rev2mDistanceSensorEncapsulation;

public class IntakeNoteSensorTest implements SimpleRobotTest {
    private final Rev2mDistanceSensorEncapsulation noteSensor = new Rev2mDistanceSensorEncapsulation();
    @Override
    public void testStart() {

    }

    @Override
    public void testPeriodic() {
        System.out.println("distance sensor reading: (cm)" + noteSensor.getDistanceCM());
    }
}
