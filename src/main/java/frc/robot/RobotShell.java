package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutoStagePrograms.*;
import frc.robot.Services.*;
import frc.robot.Utils.CommandSequenceGenerator;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.SequentialCommandSegment;
import frc.robot.Utils.Tests.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class RobotShell extends TimedRobot {
    private static final int updateFreq = 100;
    public static final boolean isFormalCompetition = false;

    private RobotCore robotCore;
    private AutoProgramRunner autoProgramRunner;
    private TransformableIntakeAndShooterService intakeAndShooterService;
    private VisionAidedPilotChassis visionAidedPilotChassis;
    private final Map<String, CommandSequenceGenerator> autoStagePrograms = new HashMap<>();
    private final SendableChooser<CommandSequenceGenerator> autoStageChooser = new SendableChooser<>();

    public RobotShell() {
        super(1.0/updateFreq);
    }


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

        autoProgramRunner = new AutoProgramRunner(robotCore.chassisModule, robotCore.robotConfig);
        intakeAndShooterService = new TransformableIntakeAndShooterService(robotCore.intake, robotCore.shooter, robotCore.transformableArm, robotCore.robotConfig, new XboxController(1));
        visionAidedPilotChassis = new VisionAidedPilotChassis(robotCore.chassisModule, robotCore.shooter, robotCore.intake, robotCore.transformableArm, robotCore.speakerTarget, robotCore.amplifierTarget, robotCore.noteTarget, new XboxController(1), robotCore.robotConfig);
        addAutoStagePrograms();
        SmartDashboard.putData("Select Auto", autoStageChooser);
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
        startAutoStage(autoStageChooser.getSelected());
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
        robotCore.updateModules();
    }

    private SimpleRobotTest robotTest = null;
    @Override
    public void testInit() {
        // System.out.println("<-- Robot Shell | test init -->");
        if (robotTest == null)
            this.robotTest = new LEDStatusLightsTest();
        robotTest.testStart();
    }

    @Override
    public void testPeriodic() {
        // System.out.println("<-- Robot Shell | robot init -->");
        robotTest.testPeriodic();
    }

    @Override
    public void simulationInit() {
        System.out.println("<-- simulate hello world! -->");
    }

    private void startAutoStage(CommandSequenceGenerator autoStageProgram) {
        System.out.println("<-- Robot Shell | starting auto" + autoStageProgram + " -->");

        final List<RobotServiceBase> services = new ArrayList<>();
        services.add(autoProgramRunner);
        robotCore.startStage(services);
        autoProgramRunner.scheduleCommandSegments(autoStageProgram.getCommandSegments(robotCore));
    }

    private void startManualStage() {
        final List<RobotServiceBase> services = new ArrayList<>();

        services.add(intakeAndShooterService);
        services.add(visionAidedPilotChassis);

        robotCore.startStage(services);
    }

    private void stopStage() {
        robotCore.stopStage();
    }

    public void addAutoStagePrograms() {
        autoStageChooser.addOption("April Tag Camera Calibration",
                new AprilTagCameraAutomaticMeasuring(
                        robotCore.aprilTagDetectionAppClient,
                        4,
                        100,
                        130,
                        240,
                        30,
                        new Vector2D(new double[] {0, -120})
                ));
        autoStageChooser.addOption("Note Camera Calibration",
                new AprilTagCameraAutomaticMeasuring(
                        robotCore.noteDetectionAppClient,
                        1,
                        -60,
                        new Rotation2D(Math.PI),
                        50,
                        100,
                        40,
                        new Vector2D(new double[] {0, 43})
                ));
        autoStageChooser.setDefaultOption("Blue DS2", new BlueDS2());
    }
}