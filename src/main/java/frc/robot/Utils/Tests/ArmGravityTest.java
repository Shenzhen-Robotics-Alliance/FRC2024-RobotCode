package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.Utils.MathUtils.LookUpTable;
import frc.robot.Utils.MechanismControllers.ArmGravityController;
import frc.robot.Utils.MechanismControllers.EncoderMotorMechanism;

public class ArmGravityTest implements SimpleRobotTest {
    private final XboxController xboxController = new XboxController(1);
    private final EncoderMotorMechanism testArmMotor = new TalonFXMotor(new TalonFX(25), true).toEncoderAndMotorMechanism();
    private static final double overallGearRatio = 80.0 * 5.0 / 3.0,
            radianPerEncoderTick = Math.PI * 2 / overallGearRatio / 2048;
    private final ArmGravityController testArmGravityController = new ArmGravityController(new ArmGravityController.ArmProfile(
            0.75,
            Math.toRadians(5) / radianPerEncoderTick,
            0,
            Math.toRadians(180) / radianPerEncoderTick,
            Math.toRadians(90) / radianPerEncoderTick,
            new LookUpTable(new double[] {Math.toRadians(0) / radianPerEncoderTick, Math.toRadians(45) / radianPerEncoderTick, Math.toRadians(90) / radianPerEncoderTick}, new double[] {0.2, 0, -0.2})
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
        else if (xboxController.getAButton())
            testArmGravityController.goToDesiredPosition(testArmMotor.getEncoderPosition(), testArmMotor.getEncoderVelocity(), Math.toRadians(45) / radianPerEncoderTick);
        else if (xboxController.getYButton())
            testArmGravityController.goToDesiredPosition(testArmMotor.getEncoderPosition(), testArmMotor.getEncoderVelocity(), Math.toRadians(0) / radianPerEncoderTick);

        SmartDashboard.putNumber("arm gravity controller correction power", testArmGravityController.getMotorPower(testArmMotor.getEncoderVelocity(), testArmMotor.getEncoderPosition()));
        if (xboxController.getLeftBumper())
            testArmMotor.updateWithController(null);
        else testArmMotor.setPower((xboxController.getRightTriggerAxis() - xboxController.getLeftTriggerAxis()) * 0.5, null);
    }
}
