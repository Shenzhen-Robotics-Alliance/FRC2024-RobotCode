package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.Modules.UpperStructure.Shooter;
import frc.robot.Utils.MechanismControllers.EncoderMotorMechanism;
import frc.robot.Utils.RobotConfigReader;

public class IntakeAndShooterTest implements SimpleRobotTest {
    private final Shooter shooter;
    private final XboxController xboxController = new XboxController(1);
    private final Timer dt = new Timer();
    private final SimpleRobotTest intakeTest = new IntakeTest();

    public IntakeAndShooterTest() {
        RobotConfigReader robotConfig;
        try {
            robotConfig = new RobotConfigReader();
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

        shooter = new Shooter(shooters, robotConfig);
    }
    @Override
    public void testStart() {
        desiredShooterRPM = 0;
        dt.start();
        shooter.init();
        shooter.gainOwnerShip(null);

        intakeTest.testStart();
    }

    double desiredShooterRPM;
    @Override
    public void testPeriodic() {
        desiredShooterRPM += dt.get() * (-180000.0 / 2048.0 * 60.0) * xboxController.getLeftY();
        desiredShooterRPM = Math.max(0, Math.min(180000.0 / 2048.0 * 60.0, desiredShooterRPM));
        dt.reset();
        if (xboxController.getAButton())
            shooter.setDesiredSpeed(desiredShooterRPM);

        shooter.updateConfigs();
        if (xboxController.getLeftBumper()) {
            shooter.enable();
            shooter.periodic();
        }
        else shooter.disable();

        SmartDashboard.putNumber("Set Shooter RPM (Press A to confirm)", desiredShooterRPM);

        intakeTest.testPeriodic();
    }
}
