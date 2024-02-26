package frc.robot.Utils.Tests;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Modules.UpperStructure.LEDStatusLights;

public class LEDStatusLightsTest implements SimpleRobotTest {
    private final SendableChooser<LEDStatusLights.LEDStatus> ledStatusSendableChooser = new SendableChooser<>();
    private final LEDStatusLights ledStatusLights = new LEDStatusLights();
    @Override
    public void testStart() {
        for (LEDStatusLights.LEDStatus ledStatus: LEDStatusLights.LEDStatus.values())
            ledStatusSendableChooser.addOption(ledStatus.name(), ledStatus);
        ledStatusSendableChooser.setDefaultOption(LEDStatusLights.LEDStatus.OFF.name(), LEDStatusLights.LEDStatus.OFF);

        ledStatusLights.init();
    }

    @Override
    public void testPeriodic() {
        ledStatusLights.setCurrentStatus(ledStatusSendableChooser.getSelected(), null);
        ledStatusLights.periodic();
    }
}
