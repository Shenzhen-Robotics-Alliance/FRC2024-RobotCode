package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Drivers.Motors.TalonFXMotor;

public class ShooterSpeedTest implements SimpleRobotTest {
    private final TalonFXMotor testShooterMotor = new TalonFXMotor(new TalonFX(14), true);
    private final XboxController xboxController = new XboxController(1);
    @Override
    public void testStart() {
        testShooterMotor.gainOwnerShip(null);
    }

    @Override
    public void testPeriodic() {
        testShooterMotor.setPower(xboxController.getRightTriggerAxis(), null);
        System.out.println("shooter speed (rpm) : " + testShooterMotor.getEncoderVelocity() / 2048.0 * 60.0);
    }
}
