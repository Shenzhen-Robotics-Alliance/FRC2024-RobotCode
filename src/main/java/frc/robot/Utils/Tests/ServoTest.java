package frc.robot.Utils.Tests;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;

public class ServoTest implements SimpleRobotTest {
    private static final Servo servo = new Servo(2);
    private static final XboxController controller = new XboxController(1);
    @Override
    public void testStart() {

    }

    @Override
    public void testPeriodic() {
        if (controller.getLeftStickButton())
            servo.set(0.5-0.5*controller.getLeftY());
        else
            servo.setDisabled();
    }
}
