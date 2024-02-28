package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Drivers.Motors.TalonFXMotor;

public class ShooterSpeedTest implements SimpleRobotTest {
    private final TalonFXMotor testShooterMotor = new TalonFXMotor(new TalonFX(16), true);
    private final XboxController xboxController = new XboxController(1);
    @Override
    public void testStart() {
        testShooterMotor.gainOwnerShip(null);
    }

    @Override
    public void testPeriodic() {
        final double power = xboxController.getRightTriggerAxis();
        if (power > 0.05)
            testShooterMotor.setPower(xboxController.getRightTriggerAxis(), null);
        else
            testShooterMotor.disableMotor(null);
        System.out.println("motor power: " + power + ", shooter speed (rpm) : " + testShooterMotor.getEncoderVelocity() / 2048.0 * 60.0);
    }
}
