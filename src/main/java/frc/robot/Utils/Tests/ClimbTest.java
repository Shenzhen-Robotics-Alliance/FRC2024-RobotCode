package frc.robot.Utils.Tests;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Drivers.Motors.MotorsSet;
import frc.robot.Drivers.Motors.VictorSPXMotor;

public class ClimbTest implements SimpleRobotTest {
    private final MotorsSet climbMotors = new MotorsSet(new Motor[] {
            new VictorSPXMotor(new VictorSPX(20), false),
            new VictorSPXMotor(new VictorSPX(21), true)
    });
    private final XboxController xboxController = new XboxController(1);

    @Override
    public void testStart() {

    }

    @Override
    public void testPeriodic() {
        climbMotors.setPower(xboxController.getRightTriggerAxis() - xboxController.getLeftTriggerAxis(), null);
    }
}
