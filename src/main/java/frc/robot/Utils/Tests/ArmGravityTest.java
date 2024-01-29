package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.Utils.MechanismControllers.ArmGravityController;

public class ArmGravityTest implements SimpleRobotTest {
    private final TalonFXMotor testArmMotor = new TalonFXMotor(new TalonFX(25), true);
    /*
    * overall gear ratio: 80 * 5 / 3
    *  */
//    private final ArmGravityController testArmGravityController = new ArmGravityController(new ArmGravityController.ArmProfile(
//            0.75,
//    ))

    @Override
    public void testStart() {

    }

    @Override
    public void testPeriodic() {

    }
}
