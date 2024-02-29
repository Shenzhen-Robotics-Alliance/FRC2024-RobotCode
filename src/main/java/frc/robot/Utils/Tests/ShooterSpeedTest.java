package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Drivers.Motors.TalonFXMotor;

public class ShooterSpeedTest implements SimpleRobotTest {
    private final TalonFXMotor testShooterMotor = new TalonFXMotor(new TalonFX(16), true);
    private final GenericHID genericHID = new GenericHID(0);
    @Override
    public void testStart() {
        testShooterMotor.gainOwnerShip(null);
    }

    @Override
    public void testPeriodic() {
        final double power = genericHID.getRawAxis(2);
        if (power > 0)
            testShooterMotor.setPower(power, null);
        else
            testShooterMotor.disableMotor(null);
        System.out.println("motor power: " + power + ", shooter speed (rpm) : " + testShooterMotor.getEncoderVelocity() / 2048.0 * 60.0);
    }
}
