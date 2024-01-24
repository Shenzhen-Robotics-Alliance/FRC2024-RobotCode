package frc.robot.Services;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Modules.SwerveBasedChassis;
import frc.robot.Utils.MathUtils.BezierCurveSchedule;
import frc.robot.Utils.MathUtils.BezierCurveScheduleGenerator;
import frc.robot.Utils.RobotConfigReader;
import frc.robot.Utils.SequentialCommandSegment;
import frc.robot.Utils.MathUtils.Vector2D;

import java.util.List;

/**
 * runs a sequence of command segment
 * the auto stage of our robot is basically running the modules in this service, simulating a pilot's commands
 */
public class AutoProgramRunner extends RobotServiceBase {
    private final List<SequentialCommandSegment> commandSegments;

    private final SwerveBasedChassis robotChassis;
    private final BezierCurveScheduleGenerator scheduleGenerator;
    private final RobotConfigReader robotConfig;
    private int currentSegment;
    private SequentialCommandSegment.StaticSequentialCommandSegment currentCommandSegment;
    private BezierCurveSchedule currentPathSchedule;
    private boolean errorOccurred = false;
    private final Timer dt = new Timer();

    public AutoProgramRunner(List<SequentialCommandSegment> commandSegments, SwerveBasedChassis chassis, RobotConfigReader robotConfig) {
        super("Auto-Program-Runner");
        this.commandSegments = commandSegments;
        this.robotChassis = chassis;
        this.robotConfig = robotConfig;
        this.scheduleGenerator = new BezierCurveScheduleGenerator(robotConfig);

        dt.start();
    }

    @Override
    public void init() {
        this.reset();
    }

    @Override
    public void periodic() {
        updateConfigs();

        final double t = currentPathSchedule.nextCheckPoint(dt.get());
        if (currentPathSchedule != null)
            robotChassis.setTranslationalTask(new SwerveBasedChassis.ChassisTaskTranslation(
                            SwerveBasedChassis.ChassisTaskTranslation.TaskType.GO_TO_POSITION,
                            currentPathSchedule.getPositionWithLERP()),
                    this);

//        System.out.println("current segment:" + currentSegment);
//        if (this.commandSegments.get(currentSegment).chassisMovementPath != null)
//            System.out.println("time scale calculation:" + getTimeScaleWithMaximumVelocityAndAcceleration());
        SmartDashboard.putNumber("dt(s)", dt.get());
        SmartDashboard.putNumber("scaled t", t);
        if (currentPathSchedule != null) {
            SmartDashboard.putNumber("auto command position (x)", currentPathSchedule.getPositionWithLERP().getX());
            SmartDashboard.putNumber("auto command position (y)", currentPathSchedule.getPositionWithLERP().getY());
        }
        if (currentCommandSegment.startingRotation != null && currentCommandSegment.endingRotation != null)
            robotChassis.setRotationalTask(new SwerveBasedChassis.ChassisTaskRotation(
                        SwerveBasedChassis.ChassisTaskRotation.TaskType.FACE_DIRECTION,
                        currentCommandSegment.getCurrentRotationWithLERP(t)),
                this);
        currentCommandSegment.periodic.run();

        if (isCurrentSegmentComplete())
            nextSegment();

        dt.reset();
    }

    @Override
    public void onDestroy() {

    }

    @Override
    public void reset() {
        this.currentSegment = 0;
        robotChassis.gainOwnerShip(this);
        initiateSegment(0);
        updateConfigs();
    }

    private void nextSegment() {
        this.commandSegments.get(currentSegment).ending.run();

        if (currentSegment+1 >= commandSegments.size())
            return;
        initiateSegment(++currentSegment);
    }

    private void initiateSegment(int segmentID) {
        currentCommandSegment = this.commandSegments.get(segmentID).embodyCurrentCommandSegment();

        currentCommandSegment.beginning.run();

        if (currentPathSchedule == null) return;
        this.currentPathSchedule = scheduleGenerator.generateSchedule(commandSegments.get(segmentID).getChassisMovementPath());
        robotChassis.gainOwnerShip(this);
    }

    public boolean isAutoStageComplete() {
        if (errorOccurred) return true;
        return this.commandSegments.size() - this.currentSegment == 1
                && this.isCurrentSegmentComplete();
    }

    public boolean isCurrentSegmentComplete() {
        SequentialCommandSegment currentSegment = this.commandSegments.get(this.currentSegment);
        final boolean chassisMovementFinished = currentPathSchedule == null || currentPathSchedule.isCurrentPathFinished();
        return chassisMovementFinished
                && currentSegment.isCompleteChecker.isComplete();
//                && robotChassis.isCurrentTranslationalTaskRoughlyComplete()
//                && robotChassis.isCurrentRotationalTaskComplete();
    }
}
