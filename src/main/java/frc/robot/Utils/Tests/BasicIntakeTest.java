package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.Motors.TalonFXMotor;

public class BasicIntakeTest implements SimpleRobotTest{
    private final TalonFXMotor intakeMotor1 = new TalonFXMotor(new TalonFX(14), true);
    private final TalonFXMotor intakeMotor2 = new TalonFXMotor(new TalonFX(15), true);
    private final XboxController xboxController = new XboxController(1);
    private double motorPower = 0;
    private final Timer dt = new Timer();
    @Override
    public void testStart() {
        dt.start();
        intakeMotor1.gainOwnerShip(null);
        intakeMotor2.gainOwnerShip(null);
    }

    @Override
    public void testPeriodic() {
        motorPower += dt.get() * -1.2 * xboxController.getRightY();
        if (Math.abs(motorPower) >= 1)
            motorPower = Math.copySign(1, motorPower);
        System.out.println("intake motor power:" + motorPower);
        dt.reset();
        final double power = xboxController.getRightBumper() ? motorPower : 0;
        intakeMotor1.setPower(power, null);
        intakeMotor2.setPower(power, null);
    }
}
