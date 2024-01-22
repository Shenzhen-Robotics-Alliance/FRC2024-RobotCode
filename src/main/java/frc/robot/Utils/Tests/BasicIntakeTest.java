package frc.robot.Utils.Tests;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BasicIntakeTest implements SimpleRobotTest{
    private final VictorSP motor = new VictorSP(0);
    private final XboxController xboxController = new XboxController(1);
    private double motorPower = 0;
    private final Timer dt = new Timer();
    @Override
    public void testStart() {
        dt.start();
    }

    @Override
    public void testPeriodic() {
        motorPower += dt.get() * -2 * xboxController.getLeftY();
        if (Math.abs(motorPower) >= 1)
            motorPower = Math.copySign(1, motorPower);
        SmartDashboard.putNumber("intake motor power", motorPower);
        dt.reset();
        if (xboxController.getRightBumper())
            motor.set(motorPower);
        else
            motor.set(0);
    }
}
