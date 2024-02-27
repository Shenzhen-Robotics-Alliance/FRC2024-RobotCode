package frc.robot.Utils.Tests;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Modules.UpperStructure.LEDStatusLights;

public class LEDStatusLightsTest implements SimpleRobotTest {
    private final SendableChooser<LEDStatusLights.LEDStatus> ledStatusSendableChooser = new SendableChooser<>();
    private final LEDStatusLights ledStatusLights = new LEDStatusLights(0);
    @Override
    public void testStart() {
        for (LEDStatusLights.LEDStatus ledStatus: LEDStatusLights.LEDStatus.values())
            ledStatusSendableChooser.addOption(ledStatus.name(), ledStatus);
        ledStatusSendableChooser.setDefaultOption(LEDStatusLights.LEDStatus.OFF.name(), LEDStatusLights.LEDStatus.OFF);

        SmartDashboard.putData("Select LED Status", ledStatusSendableChooser);
        ledStatusLights.init();
    }

    @Override
    public void testPeriodic() {
        ledStatusLights.setCurrentStatus(ledStatusSendableChooser.getSelected(), null);
        ledStatusLights.periodic();
    }
}
