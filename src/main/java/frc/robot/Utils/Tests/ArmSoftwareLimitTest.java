package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Drivers.Encoders.DCAbsolutePositionEncoder;
import frc.robot.Drivers.Encoders.Encoder;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.RobotCore;
import frc.robot.RobotShell;
import frc.robot.Utils.EasyShuffleBoard;
import frc.robot.Utils.MechanismControllers.EncoderMotorMechanism;
import frc.robot.Utils.RobotConfigReader;

public class ArmSoftwareLimitTest implements SimpleRobotTest {
    private final TalonFXMotor armMotor = new TalonFXMotor(new TalonFX(25), false);
    private final Encoder armEncoder = new DCAbsolutePositionEncoder(1);
    private final EncoderMotorMechanism armMechanism = new EncoderMotorMechanism(armEncoder, armMotor);
    private final XboxController xboxController = new XboxController(1);
    @Override
    public void testStart() {
        final RobotConfigReader robotConfig = new RobotConfigReader();
        this.armEncoder.setZeroPosition(robotConfig.getConfig("arm", "encoderZeroPositionRadians"));
        armMechanism.setSoftEncoderLimit(Math.toRadians(robotConfig.getConfig("arm", "lowerPositionLimit")), Math.toRadians(robotConfig.getConfig("arm", "upperPositionLimit")));
    }

    @Override
    public void testPeriodic() {
        armMechanism.setPower((xboxController.getLeftTriggerAxis()-xboxController.getRightTriggerAxis()) * 0.3, null);
        System.out.println("arm encoder reading(deg): "+ Math.toDegrees(armEncoder.getEncoderPosition()));
    }
}
