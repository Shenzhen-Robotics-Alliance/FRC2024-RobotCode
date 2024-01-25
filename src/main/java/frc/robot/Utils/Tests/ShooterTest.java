package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.Modules.ShooterModule;
import frc.robot.Utils.MechanismControllers.EncoderMotorMechanism;
import frc.robot.Utils.RobotConfigReader;

public class ShooterTest implements SimpleRobotTest {
    private final ShooterModule shooterModule;
    private final VictorSP intake = new VictorSP(0);
    private final XboxController xboxController = new XboxController(1);
    private final Timer dt = new Timer();
    public ShooterTest() {
        RobotConfigReader robotConfig;
        try {
            robotConfig = new RobotConfigReader("slowerChassis");
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("error while reading robot config");
        }

        EncoderMotorMechanism[] shooters = new EncoderMotorMechanism[] {
                new TalonFXMotor(
                        new TalonFX((int)robotConfig.getConfig("shooter/shooter1Port")),
                        robotConfig.getConfig("shooter/shooter1Reversed") != 0
                ).toEncoderAndMotorMechanism(),
                new TalonFXMotor(
                        new TalonFX((int)robotConfig.getConfig("shooter/shooter2Port")),
                        robotConfig.getConfig("shooter/shooter2Reversed") != 0
                ).toEncoderAndMotorMechanism()
        };

        shooterModule = new ShooterModule(shooters, robotConfig);
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
        if (xboxController.getAButton())
            shooterModule.setDesiredSpeed(desiredShooterRPM);

        if (xboxController.getLeftBumper()) {
            shooterModule.enable();
            shooterModule.periodic();
        }
        else shooterModule.disable();

        if (xboxController.getRightBumper())
            intake.set(-0.5);
        else
            intake.set(xboxController.getRightTriggerAxis());

        SmartDashboard.putNumber("Set Shooter RPM (Press A to confirm)", desiredShooterRPM);
    }
}
