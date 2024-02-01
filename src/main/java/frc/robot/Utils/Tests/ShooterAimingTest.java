package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Drivers.DistanceSensors.Rev2mDistanceSensorEncapsulation;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Drivers.Motors.MotorsSet;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.Modules.UpperStructure.Intake;
import frc.robot.Modules.UpperStructure.IntakeWithDistanceSensor;
import frc.robot.Modules.UpperStructure.Shooter;
import frc.robot.Modules.UpperStructure.TransformableArm;
import frc.robot.Utils.EasyShuffleBoard;
import frc.robot.Utils.MechanismControllers.EncoderMotorMechanism;
import frc.robot.Utils.RobotConfigReader;

public class ShooterAimingTest implements SimpleRobotTest {
    // TODO vision have to be here
    final TalonFXMotor armMotor = new TalonFXMotor(new TalonFX(25));
    final RobotConfigReader robotConfig = new RobotConfigReader();
    final TransformableArm transformableArm = new TransformableArm(armMotor, armMotor, robotConfig);
    final MotorsSet intakeMotors = new MotorsSet(
            new Motor[] {
                    new TalonFXMotor(new TalonFX(14), true),
                    new TalonFXMotor(new TalonFX(15), true)
            });
    final Intake intake = new IntakeWithDistanceSensor(intakeMotors, new Rev2mDistanceSensorEncapsulation(), robotConfig);
    final EncoderMotorMechanism[] shooterMechanisms = new EncoderMotorMechanism[] {
            new TalonFXMotor(
                    new TalonFX((int)robotConfig.getConfig("shooter/shooter1Port")),
                    robotConfig.getConfig("shooter/shooter1Reversed") != 0
            ).toEncoderAndMotorMechanism(),
            new TalonFXMotor(
                    new TalonFX((int)robotConfig.getConfig("shooter/shooter2Port")),
                    robotConfig.getConfig("shooter/shooter2Reversed") != 0
            ).toEncoderAndMotorMechanism()
    };
    final Shooter shooter = new Shooter(shooterMechanisms, robotConfig);

    final XboxController xboxController = new XboxController(1);
    @Override
    public void testStart() {
        shooter.init();
        intake.init();
        transformableArm.init();
        shootingPosition = 0;
        shooterRPM = 1200;
        dt.start();
    }


    double shootingPosition, shooterRPM;
    private final Timer dt = new Timer();
    @Override
    public void testPeriodic() {
        shooter.periodic();
        intake.periodic();
        transformableArm.periodic();

        if (xboxController.getAButton())
            transformableArm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SHOOT_NOTE, null);
        else
            transformableArm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.INTAKE, null);

        if (xboxController.getLeftBumper())
            shooter.setShooterMode(Shooter.ShooterMode.SPECIFIED_RPM, null);
        else
            shooter.setShooterMode(Shooter.ShooterMode.DISABLED, null);

        if (xboxController.getStartButton()) {
            shooter.setDesiredSpeed(shooterRPM, null);
            transformableArm.updateShootingPosition(Math.toRadians(shootingPosition), null);
        }

        shootingPosition += -15 * xboxController.getRightY() * dt.get();
        shooterRPM += -3000 * xboxController.getLeftY() * dt.get();
        if (Math.abs(shootingPosition) > 15)
            shootingPosition = Math.copySign(15, shootingPosition);
        if (shooterRPM > 6000)
            shooterRPM = 6000;
        else if (shooterRPM < 0)
            shooterRPM = 0;

        EasyShuffleBoard.putNumber("aiming", "desiredShooterRPM (Press Start to confirm)", shooterRPM);
        EasyShuffleBoard.putNumber("aiming", "desiredArmPosition (Press Start to confirm)", shootingPosition);
        dt.reset();
    }
}
