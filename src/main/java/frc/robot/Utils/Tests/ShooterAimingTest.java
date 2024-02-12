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
    final TalonFXMotor armMotor = new TalonFXMotor(new TalonFX(25));
    final RobotConfigReader robotConfig = new RobotConfigReader();
    final TransformableArm transformableArm = new TransformableArm(armMotor, armMotor, robotConfig);
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

    final XboxController xboxController = new XboxController(1);
    final JetsonDetectionAppClient aprilTagDetectionAppClient = new JetsonDetectionAppClient("AprilTagDetector", "10.55.16.109", 8888);
    final double[] targetHeights = new double[] {130, 130, 130, 130, 130, 130};
    final FixedAnglePositionTrackingCamera aprilTagPositionTrackingCamera = new FixedAnglePositionTrackingCamera(
            aprilTagDetectionAppClient,
            new FixedAngleCameraProfile(
                    0.37725,
                    -0.00284,
                    -0.00203)
            , targetHeights
    );
    @Override
    public void testStart() {
        shooter.init();
        intake.init();
        transformableArm.init();
        shootingPosition = 0;
        shooterRPM = 1200;
        dt.start();
        aprilTagDetectionAppClient.startRecognizing();
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

        printAprilTagCameraResultsToDashboard();
    }

    private void printAprilTagCameraResultsToDashboard() {
        aprilTagPositionTrackingCamera.update(new Vector2D(), new Rotation2D(0));
        final int targetID = 4;
        final TargetFieldPositionTracker.TargetOnField target;
        final double x,y,dis,hdg;
        if ((target = aprilTagPositionTrackingCamera.getVisibleTargetByID(targetID)) == null)
            x = y = dis = hdg = -1;
        else {
            x = target.fieldPosition.getX() * 100;
            y = target.fieldPosition.getY() * 100;
            dis = target.fieldPosition.getMagnitude() * 100;
            hdg = Math.toDegrees(target.fieldPosition.getHeading()) - 90;
        }
        EasyShuffleBoard.putNumber("apriltag", "target relative position to camera X (CM)", x);
        EasyShuffleBoard.putNumber("apriltag", "target relative position to camera Y (CM)", y);
        EasyShuffleBoard.putNumber("apriltag", "target distance from camera (CM)", dis);
        EasyShuffleBoard.putNumber("apriltag", "target Ang From Center Line (DEG)", hdg);
    }
}
