package frc.robot.RobotSystem;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.AutoStagePrograms.AutoStageProgram;
import frc.robot.AutoStagePrograms.TestAutoStageProgram;
import frc.robot.Services.AutoProgramRunner;
import frc.robot.Services.PilotChassis;
import frc.robot.Services.RobotServiceBase;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.List;

public class RobotShell extends TimedRobot {
    private RobotCore robotCore;
    /** called once when the robot powers on */
    @Override
    public void robotInit() {
        System.out.println("<-- Robot Shell | robot init -->");
        robotCore = new RobotCore();
    }

    /** called once when the driver station first connects to the robot */
    @Override
    public void driverStationConnected() {
        System.out.println("<-- Robot Shell | driver station connected -->");
        robotCore.initializeRobot();
    }

    /** called repeatedly after the robot powers on, no matter enabled or not */
    @Override
    public void robotPeriodic() {
        System.out.println("<-- Robot Shell | robot periodic -->");
    }

    /** called once when auto is selected and enable button is hit */
    @Override
    public void autonomousInit() {
        System.out.println("<-- Robot Shell | autonomous init -->");
        startAutoStage(new TestAutoStageProgram()); // TODO use sendable chooser
    }

    @Override
    public void autonomousPeriodic() {
        System.out.println("<-- Robot Shell | auto periodic -->");
        robotCore.updateRobot();
    }

    @Override
    public void teleopInit() {
        System.out.println("<-- Robot Shell | teleop init -->");
        startManualStage();
    }

    @Override
    public void teleopPeriodic() {
        System.out.println("<-- Robot Shell | teleop periodic -->");
        robotCore.updateRobot();
    }

    @Override
    public void disabledInit() {
        System.out.println("<-- Robot Shell | disable init -->");
        stopStage();
    }

    @Override
    public void disabledPeriodic() {
        System.out.println("<-- Robot Shell | disabled periodic -->");
        // TODO if the robot is left unused for a long time, let the motors sing
    }

    @Override
    public void testInit() {
        System.out.println("<-- Robot Shell | test init -->");
    }

    @Override
    public void testPeriodic() {
        System.out.println("<-- Robot Shell | robot init -->");
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