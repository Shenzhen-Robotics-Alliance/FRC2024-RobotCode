package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Drivers.DistanceSensors.Rev2mDistanceSensorEncapsulation;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Drivers.Motors.MotorsSet;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.Modules.UpperStructure.Intake;
import frc.robot.Utils.RobotConfigReader;

public class IntakeTest implements SimpleRobotTest {
    private final MotorsSet intakeMotors = new MotorsSet(
            new Motor[] {
                    new TalonFXMotor(new TalonFX(14), true),
                    new TalonFXMotor(new TalonFX(15), true)
            });
    private final Intake intake;
    private final XboxController testController = new XboxController(1);

    public IntakeTest() {
        RobotConfigReader robotConfigReader;
        try {
            robotConfigReader = new RobotConfigReader();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        this.intake = new Intake(intakeMotors, new Rev2mDistanceSensorEncapsulation(), robotConfigReader);
    }

    @Override
    public void testStart() {
        intake.init();
        intake.enable();
        intake.gainOwnerShip(null);
    }

    @Override
    public void testPeriodic() {
        if (testController.getBButton())
            intake.startIntake(null);
        else if (testController.getYButton())
            intake.startLaunch(null);
        else if (testController.getXButton())
            intake.startSplit(null);

        if (!testController.getLeftBumper()) {
            intake.turnOffIntake(null);
        }

        intake.periodic();
    }
}
