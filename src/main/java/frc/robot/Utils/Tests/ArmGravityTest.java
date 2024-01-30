package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.Utils.MathUtils.LookUpTable;
import frc.robot.Utils.MechanismControllers.ArmGravityController;
import frc.robot.Utils.MechanismControllers.EncoderMotorMechanism;

public class ArmGravityTest implements SimpleRobotTest {
    private final XboxController xboxController = new XboxController(1);
    private final EncoderMotorMechanism testArmMotor = new TalonFXMotor(new TalonFX(25), true).toEncoderAndMotorMechanism();
    private static final double overallGearRatio = 133.33,
            radianPerEncoderTick = Math.PI * 2 / overallGearRatio / 2048;
    private final ArmGravityController testArmGravityController = new ArmGravityController(new ArmGravityController.ArmProfile(
            0.55,
            Math.toRadians(40) / radianPerEncoderTick,
            Math.toRadians(2) / radianPerEncoderTick,
            0.06,
            0,
            Math.toRadians(18) / radianPerEncoderTick,
            Math.toRadians(36) / radianPerEncoderTick,
            new LookUpTable(new double[] {Math.toRadians(0) / radianPerEncoderTick, Math.toRadians(45) / radianPerEncoderTick, Math.toRadians(90) / radianPerEncoderTick}, new double[] {0.17, 0, -0.17})
            ));

    @Override
    public void testStart() {
        testArmGravityController.reset(testArmMotor.getEncoderPosition());
        testArmMotor.gainOwnerShip(null);
        testArmMotor.setController(testArmGravityController);
    }

    @Override
    public void testPeriodic() {
        if (xboxController.getAButton())
            testArmGravityController.goToDesiredPosition(testArmMotor.getEncoderPosition(), testArmMotor.getEncoderVelocity(), Math.toRadians(90) / radianPerEncoderTick);
        else if (xboxController.getBButton())
            testArmGravityController.goToDesiredPosition(testArmMotor.getEncoderPosition(), testArmMotor.getEncoderVelocity(), Math.toRadians(45) / radianPerEncoderTick);
        else if (xboxController.getYButton())
            testArmGravityController.goToDesiredPosition(testArmMotor.getEncoderPosition(), testArmMotor.getEncoderVelocity(), Math.toRadians(0) / radianPerEncoderTick);

        if (xboxController.getLeftBumper())
            testArmMotor.updateWithController(null);
        else testArmMotor.setPower((xboxController.getRightTriggerAxis() - xboxController.getLeftTriggerAxis()) * 0.5, null);
        testArmGravityController.getMotorPower(testArmMotor.getEncoderVelocity(), testArmMotor.getEncoderPosition());

        if (xboxController.getBackButton())
            testArmMotor.setMotorZeroPowerBehavior(Motor.ZeroPowerBehavior.RELAX, null);
        else
            testArmMotor.setMotorZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE, null);
        if (xboxController.getStartButton())
            testArmMotor.setCurrentPositionAsZeroPosition();
    }
}
