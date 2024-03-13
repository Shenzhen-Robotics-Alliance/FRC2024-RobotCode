package frc.robot.Utils.Tests;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Drivers.Motors.MotorsSet;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.Drivers.Motors.VictorSPXMotor;

public class ClimbTest implements SimpleRobotTest {
    private final TalonFXMotor left = new TalonFXMotor(new TalonFX(-1), false), right = new TalonFXMotor(new TalonFX(-1), false);
    private final XboxController xboxController = new XboxController(1);

    @Override
    public void testStart() {

    }

    @Override
    public void testPeriodic() {
        left.setPower(xboxController.getLeftY() * -1, null);
        right.setPower(xboxController.getRightY() * -1, null);
    }
}
