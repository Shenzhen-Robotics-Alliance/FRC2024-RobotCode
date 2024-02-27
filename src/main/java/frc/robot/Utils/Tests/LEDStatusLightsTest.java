package frc.robot.Utils.Tests;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Modules.UpperStructure.LEDStatusLights;

public class LEDStatusLightsTest implements SimpleRobotTest {
    private final SendableChooser<LEDStatusLights.LEDStatus> greenStatusSendableChooser = new SendableChooser<>(), blueStatusSendableChooser = new SendableChooser<>(), redStatusSendableChooser = new SendableChooser<>();
    private final LEDStatusLights red = new LEDStatusLights(0), green = new LEDStatusLights(1), blue = new LEDStatusLights(2);
    @Override
    public void testStart() {
        for (LEDStatusLights.LEDStatus ledStatus: LEDStatusLights.LEDStatus.values()) {
            redStatusSendableChooser.addOption(ledStatus.name(), ledStatus);
            greenStatusSendableChooser.addOption(ledStatus.name(), ledStatus);
            blueStatusSendableChooser.addOption(ledStatus.name(), ledStatus);
        }
        redStatusSendableChooser.setDefaultOption(LEDStatusLights.LEDStatus.OFF.name(), LEDStatusLights.LEDStatus.OFF);
        greenStatusSendableChooser.setDefaultOption(LEDStatusLights.LEDStatus.OFF.name(), LEDStatusLights.LEDStatus.OFF);
        blueStatusSendableChooser.setDefaultOption(LEDStatusLights.LEDStatus.OFF.name(), LEDStatusLights.LEDStatus.OFF);

        SmartDashboard.putData("Select RED LED Status", redStatusSendableChooser);
        SmartDashboard.putData("Select GREEN LED Status", greenStatusSendableChooser);
        SmartDashboard.putData("Select BLUE LED Status", blueStatusSendableChooser);
        red.init();
        green.init();
        blue.init();
    }

    @Override
    public void testPeriodic() {
        red.setCurrentStatus(redStatusSendableChooser.getSelected(), null);
        red.periodic();

        green.setCurrentStatus(greenStatusSendableChooser.getSelected(), null);
        green.periodic();

        blue.setCurrentStatus(blueStatusSendableChooser.getSelected(), null);
        blue.periodic();
    }
}
