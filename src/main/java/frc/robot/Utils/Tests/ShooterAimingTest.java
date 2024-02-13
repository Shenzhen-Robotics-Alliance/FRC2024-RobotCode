package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Drivers.DistanceSensors.Rev2mDistanceSensorEncapsulation;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Drivers.Motors.MotorsSet;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.Drivers.Visions.FixedAnglePositionTrackingCamera;
import frc.robot.Drivers.Visions.JetsonDetectionAppClient;
import frc.robot.Drivers.Visions.TargetFieldPositionTracker;
import frc.robot.Modules.UpperStructure.Intake;
import frc.robot.Modules.UpperStructure.IntakeWithDistanceSensor;
import frc.robot.Modules.UpperStructure.Shooter;
import frc.robot.Modules.UpperStructure.TransformableArm;
import frc.robot.Utils.ComputerVisionUtils.FixedAngleCameraProfile;
import frc.robot.Utils.EasyShuffleBoard;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.MechanismControllers.EncoderMotorMechanism;
import frc.robot.Utils.RobotConfigReader;

public class ShooterAimingTest implements SimpleRobotTest {
    private final Shooter shooter;
    private final TransformableArm transformableArm;
    private final Intake intake;
    public ShooterAimingTest(Shooter shooter, Intake intake, TransformableArm transformableArm) {
        this.shooter = shooter;
        this.transformableArm = transformableArm;
        this.intake = intake;
    }

    final XboxController xboxController = new XboxController(1);
    @Override
    public void testStart() {
        shooter.init();
        intake.init();
        transformableArm.init();
        shooter.enable();
        intake.enable();
        transformableArm.enable();
        shootingPosition = 0;
        shooterRPM = 1200;
        dt.start();
    }


    double shootingPosition, shooterRPM;
    private final Timer dt = new Timer();
    @Override
    public void testPeriodic() {
        if (xboxController.getLeftBumper()) {
            transformableArm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.INTAKE, null);
            shooter.setShooterMode(Shooter.ShooterMode.DISABLED, null);
            intake.startIntake(null);
        } else if (xboxController.getRightBumper()) {
            transformableArm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SHOOT_NOTE, null);
            shooter.setShooterMode(Shooter.ShooterMode.SPECIFIED_RPM, null);
            shooter.setDesiredSpeed(shooterRPM, null);
            transformableArm.updateShootingPosition(Math.toRadians(shootingPosition), null);
            if (xboxController.getAButton())
                intake.startLaunch(null);
        }
        else {
            transformableArm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.DEFAULT, null);
            shooter.setShooterMode(Shooter.ShooterMode.DISABLED, null);
            intake.turnOffIntake(null);
        }

        final double maxShootingAngle = 20;
        shootingPosition += -5 * xboxController.getRightY() * dt.get();
        shooterRPM += -1500 * xboxController.getLeftY() * dt.get();
        if (Math.abs(shootingPosition) > maxShootingAngle)
            shootingPosition = Math.copySign(maxShootingAngle, shootingPosition);
        if (shooterRPM > 6700)
            shooterRPM = 6700;
        else if (shooterRPM < 0)
            shooterRPM = 0;

        EasyShuffleBoard.putNumber("aiming", "desiredShooterRPM", shooterRPM);
        EasyShuffleBoard.putNumber("aiming", "desiredArmPosition", shootingPosition);
        dt.reset();
        System.out.println("test periodic update dt: " + dt.get());
    }
}
