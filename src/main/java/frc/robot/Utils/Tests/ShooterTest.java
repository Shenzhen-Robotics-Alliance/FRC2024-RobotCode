package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.Modules.ShooterModule;
import frc.robot.Utils.MechanismControllers.EncoderMotorMechanism;
import frc.robot.Utils.MechanismControllers.FlyWheelSpeedController;

public class ShooterTest implements SimpleRobotTest {
    private ShooterModule shooterModule;
    private final VictorSP intake = new VictorSP(0);
    private final XboxController xboxController = new XboxController(1);
    private final Timer dt = new Timer();
    public ShooterTest() {
        EncoderMotorMechanism[] shooters = new EncoderMotorMechanism[] {
                new TalonFXMotor(new TalonFX(14), true).toMechanism(),
                new TalonFXMotor(new TalonFX(15), true).toMechanism()
        };
        FlyWheelSpeedController.FlyWheelSpeedControllerProfile speedControllerProfile = new FlyWheelSpeedController.FlyWheelSpeedControllerProfile(
                0.8,
                0.4,
                0.1,
                0,
                160000,
                2
        );
        shooterModule = new ShooterModule(shooters, speedControllerProfile);
    }
    @Override
    public void testStart() {
        desiredShooterRPM = 0;
        dt.start();
        shooterModule.init();
        shooterModule.gainOwnerShip(null);
    }

    double desiredShooterRPM;
    @Override
    public void testPeriodic() {
        desiredShooterRPM += dt.get() * (-160000.0 / 2048.0 * 60.0) * xboxController.getLeftY();
        desiredShooterRPM = Math.max(0, Math.min(160000.0 / 2048.0 * 60.0, desiredShooterRPM));
        dt.reset();
        SmartDashboard.putNumber("Set Shooter RPM (Press A to confirm)", (int)desiredShooterRPM);
        if (xboxController.getAButton())
            shooterModule.setDesiredSpeed(desiredShooterRPM);

        if (!xboxController.getLeftBumper()) shooterModule.disable();
        shooterModule.periodic();

        if (xboxController.getRightBumper())
            intake.set(-0.5);
        else
            intake.set(xboxController.getRightTriggerAxis());
    }
}
