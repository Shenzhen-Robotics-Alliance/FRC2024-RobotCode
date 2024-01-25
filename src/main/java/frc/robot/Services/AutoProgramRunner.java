package frc.robot.Services;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Modules.SwerveBasedChassis;
import frc.robot.Utils.MathUtils.BezierCurveSchedule;
import frc.robot.Utils.MathUtils.BezierCurveScheduleGenerator;
import frc.robot.Utils.RobotConfigReader;
import frc.robot.Utils.SequentialCommandSegment;

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
    private int currentSegmentID;
    private SequentialCommandSegment.StaticSequentialCommandSegment currentCommandSegment;
    private BezierCurveSchedule currentPathSchedule;
    private final Timer dt = new Timer();
    private double currentSegmentRotationScheduleETA, rotationT;

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


        if (currentPathSchedule != null) {
            final double translationalT = currentPathSchedule.nextCheckPoint(dt.get());
            robotChassis.setTranslationalTask(new SwerveBasedChassis.ChassisTaskTranslation(
                            SwerveBasedChassis.ChassisTaskTranslation.TaskType.GO_TO_POSITION,
                            currentPathSchedule.getPositionWithLERP()),
                    this);
            SmartDashboard.putNumber("translational scaled T", translationalT);
            SmartDashboard.putNumber("auto command position (x)", currentPathSchedule.getPositionWithLERP().getX());
            SmartDashboard.putNumber("auto command position (y)", currentPathSchedule.getPositionWithLERP().getY());
        }

//        System.out.println("current segment:" + currentSegment);
//        if (this.commandSegments.get(currentSegment).chassisMovementPath != null)
//            System.out.println("time scale calculation:" + getTimeScaleWithMaximumVelocityAndAcceleration());
        SmartDashboard.putNumber("dt(s)", dt.get());
        if (currentSegmentRotationScheduleETA != -1)
            robotChassis.setRotationalTask(new SwerveBasedChassis.ChassisTaskRotation(
                            SwerveBasedChassis.ChassisTaskRotation.TaskType.FACE_DIRECTION,
                            currentCommandSegment.getCurrentRotationWithLERP(rotationT+=dt.get() / currentSegmentRotationScheduleETA)),
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
        this.currentSegmentID = 0;
        robotChassis.gainOwnerShip(this);
        initiateSegment(0);
        updateConfigs();
    }

    private void nextSegment() {
        this.commandSegments.get(currentSegmentID).ending.run();

        if (currentSegmentID +1 >= commandSegments.size())
            return;
        initiateSegment(++currentSegmentID);
    }

    private void initiateSegment(int segmentID) {
        currentCommandSegment = this.commandSegments.get(segmentID).embodyCurrentCommandSegment();

        currentCommandSegment.beginning.run();

        final boolean rotationSpecified = currentCommandSegment.startingRotation != null && currentCommandSegment.endingRotation != null;
        this.currentSegmentRotationScheduleETA = rotationSpecified ?
                scheduleGenerator.getTimeNeededToFinishRotationalSchedule(currentCommandSegment.startingRotation.getRadian(), currentCommandSegment.endingRotation.getRadian())
                : -1;
        rotationT = 0;

        if (currentCommandSegment.chassisMovementPath == null) return;
        this.currentPathSchedule = scheduleGenerator.generateTranslationalSchedule(currentCommandSegment.chassisMovementPath);
        robotChassis.gainOwnerShip(this);
    }

    public boolean isAutoStageComplete() {
        return this.commandSegments.size() - this.currentSegmentID == 1
                && this.isCurrentSegmentComplete();
    }

    public boolean isCurrentSegmentComplete() {
        SequentialCommandSegment currentSegment = this.commandSegments.get(this.currentSegmentID);
        final boolean chassisMovementFinished = currentPathSchedule == null || currentPathSchedule.isCurrentPathFinished();
        return chassisMovementFinished
                && currentSegment.isCompleteChecker.isComplete();
//                && robotChassis.isCurrentTranslationalTaskRoughlyComplete()
//                && robotChassis.isCurrentRotationalTaskComplete();
    }
}
