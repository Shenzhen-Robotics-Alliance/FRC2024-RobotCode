package frc.robot.Utils.Tests;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

public class RumbleTest implements SimpleRobotTest {
    private final XboxController xboxController = new XboxController(1);
    @Override
    public void testStart() {
        xboxController.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
    }

    @Override
    public void testPeriodic() {

    }
}
