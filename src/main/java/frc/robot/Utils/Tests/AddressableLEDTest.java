package frc.robot.Utils.Tests;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class AddressableLEDTest implements SimpleRobotTest {
    private final AddressableLED led = new AddressableLED(0);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(155);
    @Override
    public void testStart() {
        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
    }

    @Override
    public void testPeriodic() {
        for (int i = 0; i < buffer.getLength(); i++)
            buffer.setRGB(i, 0, 0, 255);

        led.setData(buffer);
    }
}
