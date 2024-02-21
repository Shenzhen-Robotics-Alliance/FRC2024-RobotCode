package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.AutoStagePrograms.*;
import frc.robot.Drivers.Visions.JetsonDetectionAppClient;
import frc.robot.Services.*;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.SequentialCommandSegment;
import frc.robot.Utils.Tests.*;

import java.util.ArrayList;
import java.util.List;

public class RobotShell extends TimedRobot {
    private static final int updateFreq = 100;
    public static final boolean isFormalCompetition = false;
    public RobotShell() {
        super(1.0/updateFreq);
    }

    private RobotCore robotCore;
    /** called once when the robot powers on */
    @Override
    public void robotInit() {
        // System.out.println("<-- Robot Shell | robot init -->");
        robotCore = new RobotCore("5516");
    }

    /** called once when the driver station first connects to the robot */
    @Override
    public void driverStationConnected() {
        // System.out.println("<-- Robot Shell | driver station connected -->");
        robotCore.initializeRobot();
    }

    /** called repeatedly after the robot powers on, no matter enabled or not */
    @Override
    public void robotPeriodic() {
        // System.out.println("<-- Robot Shell | robot periodic -->");
    }

    /** called once when auto is selected and enable button is hit */
    @Override
    public void autonomousInit() {
        // System.out.println("<-- Robot Shell | autonomous init -->");
        startAutoStage(
//                new AprilTagCameraAutomaticMeasuring(
//                    robotCore.noteDetectionAppClient,
//                    1,
//                    -30,
//                    new Rotation2D(Math.PI),
//                    25,
//                    100,
//                    45,
//                    new Vector2D(new double[] {0, 25})
//                )
                new AprilTagCameraAutomaticMeasuring(
                robotCore.aprilTagDetectionAppClient,
                4,
                100,
                140,
                300,
                45,
                new Vector2D(new double[] {0, -110})
                )
//                new AutonomousTemplateRedDS3()
        ); // TODO use sendable chooser
    }

    @Override
    public void autonomousPeriodic() {
        // System.out.println("<-- Robot Shell | auto periodic -->");
        robotCore.updateRobot();
    }

    @Override
    public void teleopInit() {
        // System.out.println("<-- Robot Shell | teleop init -->");
        startManualStage();
    }

    @Override
    public void teleopPeriodic() {
        // System.out.println("<-- Robot Shell | teleop periodic -->");
        robotCore.updateRobot();
    }

    @Override
    public void disabledInit() {
        // System.out.println("<-- Robot Shell | disable init -->");
        stopStage();
        if (robotTest != null)
            robotTest.testEnd();
    }

    @Override
    public void disabledPeriodic() {
        // System.out.println("<-- Robot Shell | disabled periodic -->");
    }

    private SimpleRobotTest robotTest = null;
    @Override
    public void testInit() {
        // System.out.println("<-- Robot Shell | test init -->");
        if (robotTest == null)
            this.robotTest = new ArmSoftwareLimitTest(robotCore.robotConfig);
        robotTest.testStart();
    }

    @Override
    public void testPeriodic() {
        // System.out.println("<-- Robot Shell | robot init -->");
        robotTest.testPeriodic();
    }

    private void startAutoStage(AutoStageProgram autoStageProgram) {
        final List<RobotServiceBase> services = new ArrayList<>();
        final List<SequentialCommandSegment> commandSegments = autoStageProgram.getCommandSegments(robotCore);
        final AutoProgramRunner autoProgramRunner = new AutoProgramRunner(commandSegments, robotCore.chassisModule, robotCore.robotConfig);
        services.add(autoProgramRunner);
        robotCore.startStage(services);
    }

    private void startManualStage() {
        final List<RobotServiceBase> services = new ArrayList<>();

        final PilotChassis pilotChassis = new PilotChassis(robotCore.chassisModule, robotCore.robotConfig);
        final TransformableIntakeAndShooterService intakeAndShooterService = new TransformableIntakeAndShooterService(robotCore.intake, robotCore.shooter, robotCore.transformableArm, robotCore.robotConfig, new XboxController(1));
        final VisionAidedPilotChassis visionAidedPilotChassis = new VisionAidedPilotChassis(robotCore.chassisModule, robotCore.shooter, robotCore.intake, robotCore.transformableArm, robotCore.speakerTarget, robotCore.amplifierTarget, robotCore.noteTarget, new XboxController(1), robotCore.robotConfig);

        services.add(intakeAndShooterService);
        services.add(pilotChassis);
//        services.add(visionAidedPilotChassis);

        robotCore.startStage(services);
    }

    private void stopStage() {
        robotCore.stopStage();
    }
}