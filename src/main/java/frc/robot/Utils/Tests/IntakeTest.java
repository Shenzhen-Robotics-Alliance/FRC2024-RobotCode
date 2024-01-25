package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Drivers.Motors.MotorsSet;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.Modules.Intake;
import frc.robot.Utils.RobotConfigReader;

public class IntakeTest implements SimpleRobotTest {
    private final MotorsSet intakeMotors = new MotorsSet(
            new Motor[] {
                    new TalonFXMotor(new TalonFX(0), false),
                    new TalonFXMotor(new TalonFX(1), false)
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

        this.intake = new Intake(intakeMotors, robotConfigReader);
    }

    @Override
    public void testStart() {
        intake.init();
        intake.enable();
        intake.gainOwnerShip(null);
    }

    @Override
    public void testPeriodic() {
        intake.periodic();

        if (testController.getRightBumper()) {

        }
    }
}
