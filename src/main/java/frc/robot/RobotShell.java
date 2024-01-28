package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutoStagePrograms.AutoStageProgram;
import frc.robot.AutoStagePrograms.TestAutoStageProgram;
import frc.robot.Drivers.Visions.FixedAnglePositionTrackingCamera;
import frc.robot.Drivers.Visions.JetsonDetectionAppClient;
import frc.robot.Drivers.Visions.TargetFieldPositionTracker;
import frc.robot.Services.AutoProgramRunner;
import frc.robot.Services.PilotChassis;
import frc.robot.Services.RobotServiceBase;
import frc.robot.Utils.FixedAngleCameraUtils.FixedAngleCameraProfile;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.SequentialCommandSegment;
import frc.robot.Utils.Tests.*;

import java.util.ArrayList;
import java.util.List;

public class RobotShell extends TimedRobot {
    private static final int updateFreq = 64;
    public RobotShell() {
        super(1.0/updateFreq);
    }
    /** TODO: put the vision client inside robot core */
    private JetsonDetectionAppClient aprilTagDetectionAppClient = null;
    private TargetFieldPositionTracker aprilTagPositionTrackingCamera;

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
        startAutoStage(new TestAutoStageProgram()); // TODO use sendable chooser
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

        aprilTagDetectionAppClient = new JetsonDetectionAppClient("AprilTagDetector", "10.99.99.109", 8888);
        final double[] targetHeights = new double[] {100, 100, 100, 100, 100, 100};
        aprilTagPositionTrackingCamera = new FixedAnglePositionTrackingCamera(
                aprilTagDetectionAppClient,
                new FixedAngleCameraProfile(
                        -0.58288,
                        -0.001342,
                        -0.002167
                ),
                targetHeights
        );
        aprilTagDetectionAppClient.startRecognizing();
    }

    @Override
    public void teleopPeriodic() {
        // System.out.println("<-- Robot Shell | teleop periodic -->");
        robotCore.updateRobot();

        // TODO below are here temporarily, move them to robot core
        if (aprilTagPositionTrackingCamera == null) return;
        aprilTagPositionTrackingCamera.update(new Vector2D(), new Rotation2D(0));
        final int targetID = 4;
        final TargetFieldPositionTracker.TargetOnField target;
        final double x,y,dis,hdg;
        if ((target = aprilTagPositionTrackingCamera.getTargetByID(targetID)) == null)
            x = y = dis = hdg = -114514;
        else {
            x = target.fieldPosition.getX() * 100;
            y = target.fieldPosition.getY() * 100;
            dis = target.fieldPosition.getMagnitude() * 100;
            hdg = Math.toDegrees(target.fieldPosition.getHeading()) - 90;
        }
        SmartDashboard.putNumber("target relative position to camera X (CM)", x);
        SmartDashboard.putNumber("target relative position to camera Y (CM)", y);
        SmartDashboard.putNumber("target distance from camera (CM)", dis);
        SmartDashboard.putNumber("target Ang From Center Line (DEG)", hdg);
    }

    @Override
    public void disabledInit() {
        // System.out.println("<-- Robot Shell | disable init -->");
        stopStage();
    }

    @Override
    public void disabledPeriodic() {
        // System.out.println("<-- Robot Shell | disabled periodic -->");

        if (aprilTagDetectionAppClient != null)
            aprilTagDetectionAppClient.stopRecognizing();

        // TODO if the robot is left unused for a long time, let the motors sing
    }

    private SimpleRobotTest robotTest = null;
    @Override
    public void testInit() {
        // System.out.println("<-- Robot Shell | test init -->");
        if (robotTest == null)
            this.robotTest = new IntakeAndShooterTest();
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
        PilotChassis pilotChassis = new PilotChassis(robotCore.chassisModule, robotCore.robotConfig);
        services.add(pilotChassis);
        robotCore.startStage(services);
    }

    private void stopStage() {
        robotCore.stopStage();
    }
}