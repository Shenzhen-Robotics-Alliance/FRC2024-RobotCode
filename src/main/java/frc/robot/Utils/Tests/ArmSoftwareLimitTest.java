package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Drivers.Encoders.DCAbsolutePositionEncoder;
import frc.robot.Drivers.Encoders.Encoder;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Drivers.Motors.MotorsSet;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.RobotCore;
import frc.robot.RobotShell;
import frc.robot.Utils.EasyShuffleBoard;
import frc.robot.Utils.MechanismControllers.EncoderMotorMechanism;
import frc.robot.Utils.RobotConfigReader;

public class ArmSoftwareLimitTest implements SimpleRobotTest {
    private final Encoder armEncoder;
    private final EncoderMotorMechanism armMechanism;
    private final XboxController xboxController = new XboxController(1);
    private final RobotConfigReader robotConfig;
    public ArmSoftwareLimitTest(RobotConfigReader robotConfig) {
        this.robotConfig = robotConfig;
        // TalonFXMotor armMotor = new TalonFXMotor(new TalonFX((int) robotConfig.getConfig("arm", "armMotorPort")), robotConfig.getConfig("arm", "armMotorReversed") != 0);
        MotorsSet armMotors = new MotorsSet(new Motor[] {
            new TalonFXMotor(new TalonFX((int) robotConfig.getConfig("arm", "armMotorPort")), robotConfig.getConfig("arm", "armMotorReversed") != 0),
            new TalonFXMotor(new TalonFX((int) robotConfig.getConfig("arm", "armMotor2Port")), robotConfig.getConfig("arm", "armMotor2Reversed") != 0)
        });
        armEncoder = new DCAbsolutePositionEncoder(1,robotConfig.getConfig("arm", "armEncoderReversed")!=0);
        armMechanism = new EncoderMotorMechanism(armEncoder, armMotors);
    }
    @Override
    public void testStart() {
        this.armEncoder.setZeroPosition(robotConfig.getConfig("arm", "encoderZeroPositionRadians"));
        armMechanism.setSoftEncoderLimit(Math.toRadians(robotConfig.getConfig("arm", "lowerPositionLimit")), Math.toRadians(robotConfig.getConfig("arm", "upperPositionLimit")));
    }

    @Override
    public void testPeriodic() {
        if (xboxController.getBackButton())
            armMechanism.disableMotor(null);
        else armMechanism.setPower((xboxController.getLeftTriggerAxis()-xboxController.getRightTriggerAxis()) * 0.3, null);
        EasyShuffleBoard.putNumber("arm",  "encoder reading(deg)", Math.toDegrees(armEncoder.getEncoderPosition()));
        EasyShuffleBoard.putNumber("arm", "power", (xboxController.getLeftTriggerAxis()-xboxController.getRightTriggerAxis()) * 0.3);
    }
}
