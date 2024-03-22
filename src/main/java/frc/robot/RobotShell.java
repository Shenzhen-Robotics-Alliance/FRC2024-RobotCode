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
import java.util.List;

public class RobotShell extends TimedRobot {
    private static final int updateFreq = 100;
    public static final boolean isFormalCompetition = false;

    private final XboxController copilotGamePad = new XboxController(1);

    private RobotCore robotCore;
    private AutoProgramRunner autoProgramRunner;
    private TransformableIntakeAndShooterService intakeAndShooterService;
    private ArmIntakeAndShootManuallyOperatedBackUpService intakeAndShooterBackUpService;
    private VisionAidedPilotChassis visionAidedPilotChassis;
    private ClimbService climbService;
    private List<SequentialCommandSegment> commandSegments;
    private final SendableChooser<CommandSequenceGenerator> autoStageChooser = new SendableChooser<>();

    public RobotShell() {
        super(1.0/updateFreq);
    }


    /** called once when the robot powers on */
    @Override
    public void robotInit() {
        // System.out.println("<-- Robot Shell | robot init -->");
        robotCore = new RobotCore("6706-2024");
    }

    /** called once when the driver station first connects to the robot */
    @Override
    public void driverStationConnected() {
        // System.out.println("<-- Robot Shell | driver station connected -->");
        robotCore.initializeRobot();

        autoProgramRunner = new AutoProgramRunner(robotCore.chassisModule, robotCore.robotConfig);
        intakeAndShooterService = new TransformableIntakeAndShooterService(robotCore.intake, robotCore.shooter, robotCore.transformableArm, robotCore.robotConfig, copilotGamePad);
        intakeAndShooterBackUpService = new ArmIntakeAndShootManuallyOperatedBackUpService(robotCore.intake, robotCore.shooter, robotCore.transformableArm, robotCore.robotConfig, copilotGamePad);
        visionAidedPilotChassis = new VisionAidedPilotChassis(robotCore.chassisModule, robotCore.shooter, robotCore.intake, robotCore.transformableArm, robotCore.speakerTarget, robotCore.amplifierTarget, robotCore.noteTarget, copilotGamePad, robotCore.robotConfig, robotCore.red, robotCore.green, robotCore.blue);
        climbService = new ClimbService(copilotGamePad, robotCore.climb, robotCore.robotConfig);

        addAutoStagePrograms();
        scheduleAutoCommands(autoStageChooser.getSelected());
        autoStageChooser.onChange(this::scheduleAutoCommands);
        SmartDashboard.putData("Select Auto", autoStageChooser);
    }

    private void scheduleAutoCommands(CommandSequenceGenerator commandSequenceGenerator) {
        System.out.println("<-- RobotShell || scheduling commands with auto program -->");
        if (commandSequenceGenerator!=null)
            this.commandSegments = commandSequenceGenerator.getCommandSegments(robotCore);
        System.out.println("<-- complete -->");
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

        if (autoProgramRunner.isAutoStageComplete())
            robotCore.stopStage();
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
            this.robotTest = new DistanceSensorTest();
        robotTest.testStart();
    }

    @Override
    public void testPeriodic() {
        // System.out.println("<-- Robot Shell | robot init -->");
        robotTest.testPeriodic();
    }

    private void startAutoStage(CommandSequenceGenerator autoStageProgram) {
        System.out.println("<-- Robot Shell | starting auto" + autoStageProgram + " -->");

        final List<RobotServiceBase> services = new ArrayList<>();
        services.add(autoProgramRunner);
        robotCore.startStage(services);
        autoProgramRunner.scheduleCommandSegments(commandSegments);
    }

    private void startManualStage() {
        final List<RobotServiceBase> services = new ArrayList<>();

        // services.add(intakeAndShooterService);
        services.add(intakeAndShooterBackUpService);
        services.add(visionAidedPilotChassis);
        services.add(climbService);

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
                        145.5-32,
                        150,
                        240,
                        30,
                        new Vector2D(new double[] {0, -110})
                ));
        autoStageChooser.addOption("Note Camera Calibration",
                new AprilTagCameraAutomaticMeasuring(
                        robotCore.noteDetectionAppClient,
                        1,
                        -60,
                        new Rotation2D(Math.PI),
                        60,
                        200,
                        40,
                        new Vector2D(new double[] {0, 60})
                ));

        autoStageChooser.setDefaultOption("Leave Community", new LeaveCommunity());

        // autoStageChooser.addOption("Middle Six Notes (Test Route)", new SixNotesMidTestRoute());
        autoStageChooser.addOption("Middle Five Notes (Test Route)", new FiveNotesMidTestRoute());
        autoStageChooser.addOption("Upper Five Notes (Test Route)", new FiveNotesUpperTestRoute());

        // autoStageChooser.addOption("Lower Four Notes", new FourNoteLower());
        // autoStageChooser.addOption("Middle Six Notes", new SixNotesMid());
        autoStageChooser.addOption("Middle Five Notes", new FiveNotesMid());
        autoStageChooser.addOption("Upper Five Notes", new FiveNotesUpper());

        autoStageChooser.addOption("Test Auto Intake", new TestAutoIntake());
    }
}