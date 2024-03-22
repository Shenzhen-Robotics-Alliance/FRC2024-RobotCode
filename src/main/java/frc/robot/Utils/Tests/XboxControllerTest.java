package frc.robot.Utils.Tests;

import edu.wpi.first.wpilibj.XboxController;

public class XboxControllerTest implements SimpleRobotTest {
    private final XboxController xboxController = new XboxController(1);
    @Override
    public void testStart() {

    }

    @Override
    public void testPeriodic() {
        System.out.println(xboxController.getPOV());
    }
}
