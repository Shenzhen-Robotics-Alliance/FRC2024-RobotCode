package frc.robot.Utils.Tests;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class LEDTest implements SimpleRobotTest {// a function that can make the light change 3 colors, the light also need to be able to blink
    private final Solenoid R = new Solenoid(PneumaticsModuleType.CTREPCM, 0), G = new Solenoid(PneumaticsModuleType.CTREPCM, 1), B = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
    private final XboxController xboxController = new XboxController(1);
    @Override
    public void testStart() {
        System.out.println("<-- LED Test | Testing... -->");
    }

    @Override
    public void testPeriodic() {
        R.set(xboxController.getAButton());
        G.set(xboxController.getXButton());
        B.set(xboxController.getBButton());
    }
}
