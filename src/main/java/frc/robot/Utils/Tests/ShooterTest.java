package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.Encoders.Encoder;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Drivers.Motors.MotorsSet;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.Modules.ShooterModule;
import frc.robot.Utils.FlyWheelSpeedController;

import java.util.List;

public class ShooterTest implements SimpleRobotTest {
    private ShooterModule shooterModule;
    private final TalonFXMotor shooterMotor1 = new TalonFXMotor(new TalonFX(14), true),
            shooterMotor2 = new TalonFXMotor(new TalonFX(15), true);
    private final VictorSP intake = new VictorSP(0);
    private final XboxController xboxController = new XboxController(1);
    private final Timer dt = new Timer();
    public ShooterTest() {
        Motor[] shooterMotors = new Motor[] {
                new TalonFXMotor(new TalonFX(0), false),
                new TalonFXMotor(new TalonFX(0), false)
        };
        Motor motorSet = new MotorsSet(shooterMotors);
        Encoder shooterEncoder = new TalonFXMotor(new TalonFX(0));
        FlyWheelSpeedController speedController = new FlyWheelSpeedController(new FlyWheelSpeedController.FlyWheelSpeedControllerProfile(
                0.8,
                0.4,
                0.1,
                0,
                160000,
                2
                ));
         shooterModule = new ShooterModule(motorSet, shooterEncoder,speedController);
    }
    @Override
    public void testStart() {
        power = 0;
        dt.start();
    }

    double power;
    @Override
    public void testPeriodic() {
        power += dt.get() * -2 * xboxController.getLeftY();
        power = Math.max(0, Math.min(0.75, power));
        SmartDashboard.putNumber("fly wheel power", power);
        SmartDashboard.putNumber("fly wheel 1 speed rpm", shooterMotor1.getEncoderVelocity() / 2048 * 60);
        SmartDashboard.putNumber("fly wheel 2 speed rpm", shooterMotor2.getEncoderVelocity() / 2048 * 60);
        dt.reset();
        shooterMotor1.gainOwnerShip(null);
        shooterMotor2.gainOwnerShip(null);
        if (xboxController.getAButton()) {
            shooterMotor1.setPower(power, null);
            shooterMotor2.setPower(power, null);
        }
        else {
            shooterMotor1.setPower(0, null);
            shooterMotor2.setPower(0, null);
        }

        if (xboxController.getLeftBumper())
            intake.set(-0.5);
        else
            intake.set(xboxController.getLeftTriggerAxis());

    }
}
