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
    @Override
    public void testStart() {
        intakeMotor1.gainOwnerShip(null);
        intakeMotor2.gainOwnerShip(null);
    }

    @Override
    public void testPeriodic() {
        double power;
        if (xboxController.getRightBumper())
            power = -0.45;
        else
            power = xboxController.getRightTriggerAxis();
        intakeMotor1.setPower(power, null);
        intakeMotor2.setPower(power, null);
    }
}
