package frc.robot.Utils.Tests;

public class IntakeAndShooterTest implements SimpleRobotTest {
    private final SimpleRobotTest intake, shooter;
    public IntakeAndShooterTest() {
        this.intake = new BasicIntakeTest();
        this.shooter = new ShooterTest();
    }
    @Override
    public void testStart() {
        intake.testStart();
        shooter.testStart();
    }

    @Override
    public void testPeriodic() {
        intake.testPeriodic();
        shooter.testPeriodic();
    }
}
