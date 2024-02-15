package frc.robot.Utils.Tests;

import edu.wpi.first.wpilibj.I2C;

import java.io.IOException;

public class TOF050CI2CSensorTest implements SimpleRobotTest {
    private static final int DEVICE_ADDRESS = 0x29;
    private final I2C distanceSensor = new I2C(I2C.Port.kOnboard, DEVICE_ADDRESS);
    @Override
    public void testStart() {

    }

    @Override
    public void testPeriodic() {
        byte[] buffer = new byte[4]; // Adjust buffer size as needed
        boolean success;
        try {
            // Read data from the I2C device
            success = distanceSensor.read(0x00, buffer.length, buffer);
            if (success) {
                // Process the data (e.g., convert bytes to a distance value)
                // ...
                System.out.println("Read data: " + buffer);
            } else {
                System.err.println("Error reading data from I2C device.");
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
