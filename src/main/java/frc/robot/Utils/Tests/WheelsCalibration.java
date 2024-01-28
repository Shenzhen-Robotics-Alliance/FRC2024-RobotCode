package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.SimEnum;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.Encoders.CanCoder;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.Utils.RobotConfigReader;

public class WheelsCalibration implements SimpleRobotTest { // calibrate wheel
    private CanCoder testCanCoder;
    private TalonFXMotor testSteerMotor;
    private TalonFXMotor testDriveMotor;
    private XboxController testController;
    private RobotConfigReader config;

    private enum Wheel {
        frontLeft, frontRight, backLeft, backRight
    }

    private final SendableChooser<WheelsCalibration.Wheel> wheelsSendableChooser = new SendableChooser<>();

    @Override
    public void testStart() {
        try {
            config = new RobotConfigReader("fasterChassis");
        } catch (Exception e) {
            throw new RuntimeException();
        }

        for (WheelsCalibration.Wheel wheel : WheelsCalibration.Wheel.values())
            wheelsSendableChooser.addOption(wheel.name(), wheel);
        wheelsSendableChooser.setDefaultOption(WheelsCalibration.Wheel.frontLeft.name(), WheelsCalibration.Wheel.frontLeft);
        SmartDashboard.putData("select wheel to calibrate", wheelsSendableChooser);

        testController = new XboxController(1);
    }

    @Override
    public void testPeriodic() {
        testDriveMotor = new TalonFXMotor(
                // new TalonFX((int) config.getConfig("hardware", wheelsSendableChooser.getSelected().name() + "WheelDriveMotor"))
                new TalonFX((int) config.getConfig("hardware", wheelsSendableChooser.getSelected().name() + "WheelDriveMotor"), "ChassisCanivore")
        );
        testDriveMotor.gainOwnerShip(null);

        testSteerMotor = new TalonFXMotor(
                // new TalonFX((int) config.getConfig("hardware", wheelsSendableChooser.getSelected().name() + "WheelSteerMotor"))
                new TalonFX((int) config.getConfig("hardware", wheelsSendableChooser.getSelected().name() + "WheelSteerMotor"), "ChassisCanivore")
        );
        testSteerMotor.gainOwnerShip(null);

        testCanCoder = new CanCoder(
                // new CANCoder((int)config.getConfig("hardware", wheelsSendableChooser.getSelected().name() + "WheelEncoder"))
                new CANcoder((int) config.getConfig("hardware", wheelsSendableChooser.getSelected().name() + "WheelEncoder"), "ChassisCanivore")
        );

        SmartDashboard.putNumber("raw steer encoder reading", testCanCoder.getRawSensorReading());
        SmartDashboard.putNumber("raw steer encoder velocity", testCanCoder.getEncoderVelocity());

        if (testController.getAButton())
            testDriveMotor.setPower(0.3, null);
        else
            testDriveMotor.disableMotor(null);

        if (testController.getBButton())
            testSteerMotor.setPower(0.3, null);
        else
            testSteerMotor.disableMotor(null);
    }
}
