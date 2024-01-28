package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Drivers.Motors.TalonFXMotor;

import java.util.ArrayList;
import java.util.List;

public class IntakeEncoderTest implements SimpleRobotTest {
    private final TalonFXMotor testMotorAndEncoder = new TalonFXMotor(new TalonFX(14), true);
    private final Motor followerMotor = new TalonFXMotor(new TalonFX(15), true);
    private final XboxController testController = new XboxController(1);
    private List<Double> rollingIntegral;
    private static final double rollingIntegralTime = 0.5;
    private static final double period = 0.05;
    @Override
    public void testStart() {
        testMotorAndEncoder.gainOwnerShip(null);
        followerMotor.gainOwnerShip(null);
        dt.start();
        rollingIntegral = new ArrayList<>();
        for (int i = 0; i < rollingIntegralTime / period; i++)
            rollingIntegral.add(0.0);
    }

    private double encoderVelocity = 0;
    private final Timer dt = new Timer();
    @Override
    public void testPeriodic() {
        double power = 0;
        if (testController.getAButton())
            power = 0.65;
        else if (testController.getBButton())
            power = -0.65;
        testMotorAndEncoder.setPower(power, null);
        followerMotor.setPower(power, null);

        SmartDashboard.putNumber("test encoder velocity", testMotorAndEncoder.getEncoderVelocity());

        if (dt.get() > period) {
            SmartDashboard.putNumber("test encoder acceleration", getRollingIntegralValue());
            rollingIntegral.add((testMotorAndEncoder.getEncoderVelocity() - encoderVelocity) / dt.get());
            encoderVelocity = testMotorAndEncoder.getEncoderVelocity();
            rollingIntegral.remove(0);
            dt.reset();
        }
    }

    private double getRollingIntegralValue() {
        double result = 0;
        for (Double x : rollingIntegral)
            result += x;
        return result / rollingIntegral.size();
    }
}
