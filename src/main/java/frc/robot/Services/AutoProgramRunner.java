package frc.robot.Services;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Modules.SwerveBasedChassis;
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
    private int currentSegment;
    private final SwerveBasedChassis robotChassis;
    private final RobotConfigReader robotConfig;
    private double currentSegmentTime;
    private double currentSegmentChassisPathTimeScale; // slow the time down when smaller than 1 (=1/ETA)
    private boolean errorOccurred = false;
    private final Timer dt = new Timer();

    public AutoProgramRunner(List<SequentialCommandSegment> commandSegments, SwerveBasedChassis chassis, RobotConfigReader robotConfig) {
        super("Auto-Program-Runner");
        this.commandSegments = commandSegments;
        this.robotChassis = chassis;
        this.robotConfig = robotConfig;

        dt.start();
    }

    @Override
    public void init() {
        /* check if there is any jump in the starting and ending point */
        for (int currentSegment = 0; currentSegment < commandSegments.size()-1; currentSegment++) {
            if (commandSegments.get(currentSegment).chassisMovementPath == null || commandSegments.get(currentSegment+1).chassisMovementPath == null)
                continue;
            final double distanceBetweenCurrentEndToNextStart = Vector2D.displacementToTarget(
                            commandSegments.get(currentSegment).chassisMovementPath.getPositionWithLERP(1),
                            commandSegments.get(currentSegment+1).chassisMovementPath.getPositionWithLERP(0))
                    .getMagnitude();
            if (distanceBetweenCurrentEndToNextStart > 10)
                throw new IllegalArgumentException("current segment (id:" + currentSegment + ")'s starting point does match the ending point of the last segment with deviation " + distanceBetweenCurrentEndToNextStart);
        }
        this.reset();
    }

    @Override
    public void periodic() {
        updateConfigs();
        currentSegmentTime += dt.get();

        final SequentialCommandSegment currentCommandSegment = commandSegments.get(currentSegment);
        final double t = currentSegmentTime * currentSegmentChassisPathTimeScale;
        // TODO also include acceleration limiting in chassis module
        if (commandSegments.get(currentSegment).chassisMovementPath != null)
            robotChassis.setTranslationalTask(new SwerveBasedChassis.ChassisTaskTranslation(
                            SwerveBasedChassis.ChassisTaskTranslation.TaskType.GO_TO_POSITION,
                            currentCommandSegment.chassisMovementPath.getPositionWithLERP(t)),
                    this);

//        System.out.println("current segment:" + currentSegment);
//        if (this.commandSegments.get(currentSegment).chassisMovementPath != null)
//            System.out.println("time scale calculation:" + getTimeScaleWithMaximumVelocityAndAcceleration());
        SmartDashboard.putNumber("time(scaled)", t);
        SmartDashboard.putNumber("time(raw)", currentSegmentTime);
        SmartDashboard.putNumber("dt(s)", dt.get());
        SmartDashboard.putNumber("ETA",1.0f/currentSegmentChassisPathTimeScale);
        if (currentCommandSegment.chassisMovementPath!=null) {
            SmartDashboard.putNumber("auto command position (x)", currentCommandSegment.chassisMovementPath.getPositionWithLERP(t).getX());
            SmartDashboard.putNumber("auto command position (y)", currentCommandSegment.chassisMovementPath.getPositionWithLERP(t).getY());
        }
        robotChassis.setRotationalTask(new SwerveBasedChassis.ChassisTaskRotation(
                        SwerveBasedChassis.ChassisTaskRotation.TaskType.FACE_DIRECTION,
                        currentCommandSegment.getCurrentRotationWithLERP(t)),
                this);
        currentCommandSegment.periodic.run();

        if (isCurrentSegmentComplete())
            nextSegment();

        dt.reset();
    }

    private double autoStageMaxAcceleration, autoStageMaxVelocity, autoStageMaxAngularVelocity;
    @Override
    public void updateConfigs() {
        autoStageMaxAcceleration = robotConfig.getConfig("auto", "autoStageMaxAcceleration");
        autoStageMaxVelocity = robotConfig.getConfig("auto", "autoStageMaxVelocity");
        autoStageMaxAngularVelocity = Math.toRadians(robotConfig.getConfig("auto", "autoStageMaxAngularVelocity"));
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
        this.currentSegmentTime = 0;
        this.commandSegments.get(segmentID).beginning.run();

        if (commandSegments.get(segmentID).chassisMovementPath == null) return;
        robotChassis.gainOwnerShip(this);
        this.currentSegmentChassisPathTimeScale = getTimeScaleWithMaximumVelocityAndAcceleration();
        System.out.println("segment " +segmentID + " time scale: "+ this.currentSegmentChassisPathTimeScale);
    }
    private double getTimeScaleWithMaximumVelocityAndAcceleration() {
        final double maxVel = this.commandSegments.get(currentSegment).chassisMovementPath.maximumSpeed;
        final double maxAcc = this.commandSegments.get(currentSegment).chassisMovementPath.maximumAcceleration;
        final double maxAngularVel = this.commandSegments.get(currentSegment).maxAngularVelocity;

        System.out.println("current segment max acc: " + maxAcc + ", max vel: " + maxVel + ", max ang vel: " + maxAngularVel);
        System.out.println("allowed segment max acc: " + autoStageMaxAcceleration + ", max vel: " + autoStageMaxVelocity + ", max ang vel: " + autoStageMaxAngularVelocity);
        return Math.min(Math.min(autoStageMaxAcceleration / maxAcc, autoStageMaxVelocity/ maxVel), autoStageMaxAngularVelocity / maxAngularVel);
    }

    public boolean isAutoStageComplete() {
        if (errorOccurred) return true;
        return this.commandSegments.size() - this.currentSegment == 1
                && this.isCurrentSegmentComplete();
    }

    public boolean isCurrentSegmentComplete() {
        SequentialCommandSegment currentSegment = this.commandSegments.get(this.currentSegment);
        final double ETA = currentSegment.chassisMovementPath == null ?
                0:  (1.0f/currentSegmentChassisPathTimeScale);
        return currentSegmentTime >= ETA
                && currentSegment.isCompleteChecker.isComplete();
//                && robotChassis.isCurrentTranslationalTaskRoughlyComplete()
//                && robotChassis.isCurrentRotationalTaskComplete();
    }
}
