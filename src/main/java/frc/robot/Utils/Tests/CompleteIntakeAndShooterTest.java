package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Drivers.DistanceSensors.Rev2mDistanceSensorEncapsulation;
import frc.robot.Drivers.Encoders.DCAbsolutePositionEncoder;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Drivers.Motors.MotorsSet;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.Modules.UpperStructure.Intake;
import frc.robot.Modules.UpperStructure.IntakeWithDistanceSensor;
import frc.robot.Modules.UpperStructure.Shooter;
import frc.robot.Modules.UpperStructure.TransformableArm;
import frc.robot.Services.TransformableIntakeAndShooterService;
import frc.robot.Utils.MechanismControllers.EncoderMotorMechanism;
import frc.robot.Utils.RobotConfigReader;

public class CompleteIntakeAndShooterTest implements SimpleRobotTest {
    /* TODO following constants in robotConfigs */
    final TalonFXMotor armMotor = new TalonFXMotor(new TalonFX(25) ,false);
    final DCAbsolutePositionEncoder armEncoder = new DCAbsolutePositionEncoder(1, false);
    final RobotConfigReader robotConfig = new RobotConfigReader();
    final TransformableArm transformableArm = new TransformableArm(armMotor, armEncoder, robotConfig);
    final MotorsSet intakeMotors = new MotorsSet(
            new Motor[] {
                    new TalonFXMotor(new TalonFX(13), true),
                    new TalonFXMotor(new TalonFX(14), true)
            });
    final Intake intake = new IntakeWithDistanceSensor(intakeMotors, new TalonFXMotor(new TalonFX(14), true), new Rev2mDistanceSensorEncapsulation(), robotConfig);
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
    final TransformableIntakeAndShooterService intakeAndShooterService = new TransformableIntakeAndShooterService(intake, shooter, transformableArm, robotConfig, new XboxController(1));

    @Override
    public void testStart() {
        shooter.init();
        shooter.reset();
        intake.init();
        intake.reset();
        transformableArm.init();
        transformableArm.reset();
        intakeAndShooterService.init();
        intakeAndShooterService.reset();
    }

    @Override
    public void testPeriodic() {
        intakeAndShooterService.periodic();

        shooter.periodic();
        intake.periodic();
        transformableArm.updateConfigs();
        transformableArm.periodic();
    }

    @Override
    public void testEnd() {
        shooter.disable();
        intake.disable();
        transformableArm.disable();
    }
}
