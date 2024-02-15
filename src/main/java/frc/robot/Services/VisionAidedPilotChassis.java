package frc.robot.Services;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.Visions.TargetFieldPositionTracker;
import frc.robot.Modules.Chassis.SwerveBasedChassis;
import frc.robot.Modules.UpperStructure.Intake;
import frc.robot.Modules.UpperStructure.Shooter;
import frc.robot.Modules.UpperStructure.TransformableArm;
import frc.robot.Utils.ComputerVisionUtils.AprilTagReferredTarget;
import frc.robot.Utils.MathUtils.AngleUtils;
import frc.robot.Utils.MathUtils.BezierCurve;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.RobotConfigReader;

import java.util.HashMap;
import java.util.Map;
import java.util.zip.CheckedOutputStream;

/**
 * based on the pilot chassis, we add auto-aiming, shoot and intake functions in this service
 * a sendable chooser on smartdashboard will choose whether this service, or the more basic TransformableIntakeAndShooterService controls the three modules: shooter, transformer and intake
 * */
public class VisionAidedPilotChassis extends PilotChassis {
    public enum VisionTargetClass {
        SPEAKER,
        AMPLIFIER
    }
    private SendableChooser<VisionTargetClass> targetChooser;
    private enum Status {
        /**
         * the movement of the chassis is controlled by the pilot
         * the vision system will take over the rotation control if the pilot pressed "auto-rotation-control" button
         * */
        MANUALLY_DRIVING,
        /**
         * searching for the april tag on the selected shooting target
         * the chassis will drive slower and once it sees
         * */
        SEARCHING_FOR_SHOOT_TARGET,
        /**
         * the april tag target is already seen and the robot is reaching for it
         * */
        REACHING_TO_SHOOT_TARGET,
        /**
         * searching for Notes in the back of the robot
         * */
        SEARCHING_FOR_NOTE,
        /**
         * the Note is found, and we are moving to approach it
         * */
        GRABBING_NOTE
    }
    private Status currentStatus;

    private final Shooter shooter;
    private final Intake intake;
    private final TransformableArm arm;
    private final AprilTagReferredTarget speakerTarget, amplifierTarget, noteTarget;
    private final XboxController copilotGamePad;
    private final DriverStation.Alliance alliance;



    /**
     * @param chassis
     * @param shooter
     * @param intake
     * @param arm
     * @param copilotGamePad
     * @param robotConfig
     */
    public VisionAidedPilotChassis(SwerveBasedChassis chassis, Shooter shooter, Intake intake, TransformableArm arm, AprilTagReferredTarget speakerTarget, AprilTagReferredTarget amplifierTarget, AprilTagReferredTarget noteTarget, XboxController copilotGamePad, RobotConfigReader robotConfig) {
        super(chassis, robotConfig);
        this.shooter = shooter;
        this.intake = intake;
        this.arm = arm;

        this.noteTarget = noteTarget;
        this.speakerTarget = speakerTarget;
        this.amplifierTarget = amplifierTarget;
        this.copilotGamePad = copilotGamePad;
        this.alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
    }


    @Override
    public void init() {
        super.init();
        targetChooser = new SendableChooser<>();
        targetChooser.setDefaultOption(VisionTargetClass.SPEAKER.name(), VisionTargetClass.SPEAKER);
        targetChooser.addOption(VisionTargetClass.AMPLIFIER.name(), VisionTargetClass.AMPLIFIER);
        SmartDashboard.putData("current aiming target", targetChooser);
    }


    @Override
    public void reset() {
        super.reset();
        this.currentStatus = Status.MANUALLY_DRIVING;
        updateConfigs();
        activateControlUpperStructure(); // TODO use sendable chooser
    }


    /** the robot's position when the current task started, 0.4 seconds in advance of the current chassis position (predicted by velocity) */
    private Vector2D chassisPositionWhenCurrentVisionTaskStarted;
    /** the position of the current target, when it is last seen, this is always updated when a visual task started and is updated during the task process whenever target is seen */
    private Vector2D currentVisualTargetLastSeenPosition;
    /** only calculated once when the task is initiated */
    private double currentVisionTaskETA;
    private long timeTaskStartedMillis;
    @Override
    public void periodic() {
        super.periodic();
        final int translationAutoPilotButton = (int)robotConfig.getConfig(super.controllerName, "translationAutoPilotButton"),
                rotationAutoPilotButton = (int)robotConfig.getConfig(super.controllerName, "rotationAutoPilotButton");
        final VisionTargetClass currentAimingTargetClass = targetChooser.getSelected();
        final AprilTagReferredTarget currentAimingTarget = switch (currentAimingTargetClass) {
            case SPEAKER -> speakerTarget;
            case AMPLIFIER -> amplifierTarget;
        };

        if (pilotController.keyOnHold(rotationAutoPilotButton))
            chassis.setRotationalTask(new SwerveBasedChassis.ChassisTaskRotation(SwerveBasedChassis.ChassisTaskRotation.TaskType.FACE_DIRECTION,
                            intake.isNoteInsideIntake() ? getAprilTagTargetRotation(currentAimingTargetClass, currentAimingTarget) : getNoteRotation()),
                    this);
        if (copilotGamePad.getXButton())
            this.currentStatus = Status.MANUALLY_DRIVING;

        switch (currentStatus) {
            case MANUALLY_DRIVING -> {
                shooter.setShooterMode(Shooter.ShooterMode.DISABLED, this);
                arm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.DEFAULT, this);
                if (copilotGamePad.getBButton())
                    intake.startSplit(this); // in case if the Note is stuck
                else
                    intake.turnOffIntake(this);
                if (pilotController.keyOnPress(translationAutoPilotButton))
                    currentStatus = intake.isNoteInsideIntake() ?  Status.SEARCHING_FOR_SHOOT_TARGET : Status.SEARCHING_FOR_NOTE;
            }
            case SEARCHING_FOR_SHOOT_TARGET -> {
                updateChassisPositionWhenTaskStarted();

                shooter.setShooterMode(
                        switch (currentAimingTargetClass) {
                            case SPEAKER -> Shooter.ShooterMode.SHOOT;
                            case AMPLIFIER -> Shooter.ShooterMode.AMPLIFY;
                        }, this);
                arm.setTransformerDesiredPosition(switch (currentAimingTargetClass) {
                    case SPEAKER -> TransformableArm.TransformerPosition.SHOOT_NOTE;
                    case AMPLIFIER -> TransformableArm.TransformerPosition.SCORE_AMPLIFIER;
                }, this);
                intake.turnOffIntake(this);


                System.out.println("<-- AP | searching for shoot target... --> ");
                if (currentAimingTarget.isVisible())
                    switch (currentAimingTargetClass) {
                        case SPEAKER -> initiateGoToSpeakerTargetProcess();
                        case AMPLIFIER -> initiateGoToAmplifierProcess();
                    }
                else if (!pilotController.keyOnHold(translationAutoPilotButton) && arm.transformerInPosition() && shooter.shooterReady()) {
                    System.out.println("<-- AP | translational AP button released, launch begins -->"); // TODO why does the launch process just start
                    intake.startLaunch(this);
                }
                if (!intake.isNoteInsideIntake()) {
                    currentStatus = Status.MANUALLY_DRIVING;
                    System.out.println("<-- AP | note gone when searching for shoot target, exiting... --->");
                }
            }
            case SEARCHING_FOR_NOTE -> {
                updateChassisPositionWhenTaskStarted();

                shooter.setShooterMode(Shooter.ShooterMode.DISABLED, this);
                arm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.INTAKE, this);
                intake.startIntake(this);

                if (!pilotController.keyOnHold(translationAutoPilotButton) || intake.isNoteInsideIntake())
                    currentStatus = Status.MANUALLY_DRIVING;
                else if (noteTarget.isVisible())
                    initiateGrabNoteProcess();
            }
            case REACHING_TO_SHOOT_TARGET -> {
                intake.turnOffIntake(this);
                switch (currentAimingTargetClass) {
                    case SPEAKER -> proceedGoToSpeakerTarget(translationAutoPilotButton);
                    case AMPLIFIER -> proceedGoToAmplifierTarget(translationAutoPilotButton);
                }
            }
            case GRABBING_NOTE -> proceedGrabNoteProcess(translationAutoPilotButton);
        }
    }

    private void updateChassisPositionWhenTaskStarted() {
        chassisPositionWhenCurrentVisionTaskStarted = chassis.positionEstimator.getRobotPosition2D().addBy(chassis.positionEstimator.getRobotVelocity2D().multiplyBy(chassisReactionDelay));
    }

    /**
     * @return the facing of the chassis such that it faces the target, or the default value if unseen. In radian, zero is to front.
     * */
    private double getAprilTagTargetRotation(VisionTargetClass currentAimingTargetClass, AprilTagReferredTarget currentAimingTarget) {
        final Vector2D targetFieldPosition = currentAimingTarget.getTargetFieldPositionWithAprilTags(objectUnseenTimeOut);
        if (targetFieldPosition == null)
            return switch (currentAimingTargetClass) {
                case SPEAKER -> speakerDefaultRotation;
                case AMPLIFIER -> amplifyingDefaultFacing;
            };
        return Vector2D.displacementToTarget(chassis.positionEstimator.getRobotPosition2D(), targetFieldPosition).getHeading() - Math.toRadians(90);
    }

    /**
     * @return the facing of the chassis such that it faces the target, or the default value if unseen. In radian, zero is to front.
     * */
    private double getNoteRotation() {
        final int pov = copilotGamePad.getPOV();
        if (pov == -1)
            return grabbingNoteDefaultFacing;
        return AngleUtils.simplifyAngle(Math.toRadians(180 - pov));
    }

    private void initiateGoToSpeakerTargetProcess() {
        if (updateTargetPositionIfSeen(speakerTarget)) return; // failed when unseen

        currentStatus = Status.REACHING_TO_SHOOT_TARGET;
        final double length = getPathToSpeakerTarget().getLength(10);
        currentVisionTaskETA = length / chassisSpeedLimitWhenAutoAim;
        timeTaskStartedMillis = System.currentTimeMillis();
    }

    private void proceedGoToSpeakerTarget(int translationAutoPilotButton) {
        shooter.setShooterMode(Shooter.ShooterMode.SHOOT, this);
        arm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SHOOT_NOTE, this);

        /* if seen, update the position */
        updateTargetPositionIfSeen(speakerTarget);

        final double timeSinceTaskStarted = (System.currentTimeMillis() - timeTaskStartedMillis) / 1000.0;
        final BezierCurve currentPath = getPathToSpeakerTarget();
        chassis.setTranslationalTask(new SwerveBasedChassis.ChassisTaskTranslation(SwerveBasedChassis.ChassisTaskTranslation.TaskType.GO_TO_POSITION,
                currentPath.getPositionWithLERP(timeSinceTaskStarted / currentVisionTaskETA)), this);
        chassis.setRotationalTask(new SwerveBasedChassis.ChassisTaskRotation(SwerveBasedChassis.ChassisTaskRotation.TaskType.FACE_DIRECTION,
                getAprilTagTargetRotation(VisionTargetClass.SPEAKER, speakerTarget)), this);

        if (intake.getCurrentStatus() != Intake.IntakeModuleStatus.LAUNCHING && shooter.shooterReady() && shooter.targetInRange() && arm.transformerInPosition() && chassis.isCurrentRotationalTaskFinished()) {
            // start shooting
            intake.startLaunch(this);
        }
        if (timeSinceTaskStarted > currentVisionTaskETA + objectUnseenTimeOut/1000.0 || !intake.isNoteInsideIntake() || !pilotController.keyOnHold(translationAutoPilotButton))
            currentStatus = Status.MANUALLY_DRIVING; // finished or cancelled
    }

    private BezierCurve getPathToSpeakerTarget() {
        final boolean pilotSpecifyingShootingProcessEndPoint = pilotController.getTranslationalStickValue().getMagnitude() > 0.4; // only when significant movement
        final Vector2D
                /* the position of the ending point of the path of the shooting task, relative to the shooting sweet-spot  */
                shootingProcessEndPointFromSweetSpotDeviation = pilotSpecifyingShootingProcessEndPoint ?
                pilotController.getTranslationalStickValue().multiplyBy(shootingProcessEndingPointUpdatableRange) :
                defaultShootProcessEndingPoint,
                /* the position of the ending point of the path of the shooting task, relative to the speaker  */
                shootingProcessEndPoint = shootingSweetSpot.addBy(shootingProcessEndPointFromSweetSpotDeviation),
                /* the middle point of the path, relative to the shooting sweet-spot  */
                shootingProcessAnotherPoint = shootingSweetSpot.addBy(shootingProcessEndPointFromSweetSpotDeviation.multiplyBy(-1)),
                shootingProcessEndPointFieldPosition = shootingProcessEndPoint.addBy(currentVisualTargetLastSeenPosition),
                shootingProcessAnotherPointFieldPosition = shootingProcessAnotherPoint.addBy(currentVisualTargetLastSeenPosition);

        return new BezierCurve(
                chassisPositionWhenCurrentVisionTaskStarted, // starting from the chassis initial position during task
                shootingProcessAnotherPointFieldPosition, // middle point is the another point
                shootingProcessEndPointFieldPosition
        );
    }

    private void initiateGoToAmplifierProcess() {
        if (!updateTargetPositionIfSeen(amplifierTarget)) return; // failed if unseen
        currentStatus = Status.REACHING_TO_SHOOT_TARGET;
    }

    private void proceedGoToAmplifierTarget(int translationAutoPilotButton) {
        shooter.setShooterMode(Shooter.ShooterMode.AMPLIFY, this);
        arm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SCORE_AMPLIFIER, this);

        /* if seen, update the position */
        updateTargetPositionIfSeen(amplifierTarget);

        /* pass the target position to chassis */
        chassis.setTranslationalTask(new SwerveBasedChassis.ChassisTaskTranslation(SwerveBasedChassis.ChassisTaskTranslation.TaskType.GO_TO_POSITION,
                currentVisualTargetLastSeenPosition.addBy(amplifyingPositionToAmplifier)), this);
        chassis.setRotationalTask(new SwerveBasedChassis.ChassisTaskRotation(SwerveBasedChassis.ChassisTaskRotation.TaskType.FACE_DIRECTION,
                getAprilTagTargetRotation(VisionTargetClass.AMPLIFIER, amplifierTarget)), this);

        if (intake.getCurrentStatus() != Intake.IntakeModuleStatus.LAUNCHING && chassis.isCurrentRotationalTaskFinished() && chassis.isCurrentTranslationalTaskFinished()) {
            // start shooting
            intake.startLaunch(this);
        }
        if (!intake.isNoteInsideIntake() || !pilotController.keyOnHold(translationAutoPilotButton))
            currentStatus = Status.MANUALLY_DRIVING; // finished or cancelled
    }

    private void initiateGrabNoteProcess() {
        if (!updateTargetPositionIfSeen(noteTarget)) return; // fails if note not seen

        chassisPositionWhenCurrentVisionTaskStarted = chassis.positionEstimator.getRobotPosition2D();
        currentStatus = Status.GRABBING_NOTE;
        final double length = getPathToNoteTarget().getLength(10);
        currentVisionTaskETA = length / chassisSpeedLimitWhenAutoAim;
        intake.startIntake(this);
        timeTaskStartedMillis = System.currentTimeMillis();
    }

    private void proceedGrabNoteProcess(int translationAutoPilotButton) {
        shooter.setShooterMode(Shooter.ShooterMode.DISABLED, this);
        arm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.INTAKE, this);

        /* if seen, update the position */
        updateTargetPositionIfSeen(noteTarget);

        final double timeSinceTaskStarted = (System.currentTimeMillis() - timeTaskStartedMillis) / 1000.0;
        final BezierCurve currentPath = getPathToNoteTarget();
        chassis.setTranslationalTask(new SwerveBasedChassis.ChassisTaskTranslation(SwerveBasedChassis.ChassisTaskTranslation.TaskType.GO_TO_POSITION,
                currentPath.getPositionWithLERP(timeSinceTaskStarted / currentVisionTaskETA)), this);
        chassis.setRotationalTask(new SwerveBasedChassis.ChassisTaskRotation(SwerveBasedChassis.ChassisTaskRotation.TaskType.FACE_DIRECTION,
                getNoteRotation()), this);

        if (!pilotController.keyOnHold(translationAutoPilotButton) || intake.isNoteInsideIntake())
            currentStatus = Status.MANUALLY_DRIVING;
    }

    private BezierCurve getPathToNoteTarget() {
        // only specify when significant command is sent by the pilot
        final boolean pilotSpecifyingGrabbingProcessEndPoint = pilotController.getTranslationalStickValue().getMagnitude() > 0.4;
        final Vector2D
                /* the position of the ending point of the path of the grabbing task, relative to the note target  */
                grabbingProcessEndPointFromNoteDeviation =
                pilotSpecifyingGrabbingProcessEndPoint ?
                        new Vector2D(pilotController.getTranslationalStickValue().getHeading(), grabbingNoteDistance) :
                        new Vector2D(new double[] {0, -grabbingNoteDistance}).multiplyBy(new Rotation2D(grabbingNoteDefaultFacing)), // by default, we move backwards in relative to the robot, but we need to convert this to in relative to field by rotating it.
                /* the position of the ending point of the path of the grabbing task, relative to the speaker */
                grabbingProcessEndingPoint = currentVisualTargetLastSeenPosition.addBy(grabbingProcessEndPointFromNoteDeviation),
                grabbingProcessEndingAnotherPoint = currentVisualTargetLastSeenPosition.addBy(grabbingProcessEndPointFromNoteDeviation.multiplyBy(-1));

        return new BezierCurve(
                chassisPositionWhenCurrentVisionTaskStarted, // starting from the chassis initial position during task
                grabbingProcessEndingAnotherPoint, // middle point is the another point
                grabbingProcessEndingPoint
        );
    }

    /**
     * @return whether the position have been updated
     * */
    private boolean updateTargetPositionIfSeen(AprilTagReferredTarget target) {
        if (!target.isVisible())
            return false;
        currentVisualTargetLastSeenPosition = target.getTargetFieldPositionWithVisibleAprilTags();
        return true;
    }

    private long objectUnseenTimeOut;
    private double speakerDefaultRotation;
    private Vector2D amplifyingPositionToAmplifier;
    /** in radian, zero is front */
    private double amplifyingDefaultFacing;
    /** in radian, zero is front */
    private double grabbingNoteDefaultFacing;
    /** the amount of distance to travel when intake is sucking the note */
    private double grabbingNoteDistance;
    private double chassisSpeedLimitWhenAutoAim; // m/s
    private Vector2D shootingSweetSpot;
    /** the pilot can specify the spot of shooting, by this amount of distance away from the sweet spot */
    private double shootingProcessEndingPointUpdatableRange;
    /** the default shoot process ending point, in reference to the shooting sweet spot */
    private Vector2D defaultShootProcessEndingPoint;
    /** how many seconds does the chassis need to react */
    private double chassisReactionDelay;
    @Override
    public void updateConfigs() {
        super.updateConfigs();

        /* TODO read from robotConfig */
        objectUnseenTimeOut = 1000;
        grabbingNoteDistance = 0.25;
        chassisSpeedLimitWhenAutoAim = 5;
        shootingSweetSpot = new Vector2D(new double[] {0, -2.5});
        shootingProcessEndingPointUpdatableRange = 0.5;
        chassisReactionDelay = 0.4;
        switch (alliance) {
            case Red -> {
                amplifyingPositionToAmplifier = new Vector2D(new double[] {0, -0.4});
                speakerDefaultRotation = Math.toRadians(180);
                amplifyingDefaultFacing = Math.toRadians(-90);
                grabbingNoteDefaultFacing = Math.toRadians(-120);
                defaultShootProcessEndingPoint = new Vector2D(new double[] {0, -shootingProcessEndingPointUpdatableRange});
            }
            case Blue -> {
                amplifyingPositionToAmplifier = new Vector2D(new double[] {0, 0.4});
                speakerDefaultRotation = Math.toRadians(180);
                amplifyingDefaultFacing = Math.toRadians(90);
                grabbingNoteDefaultFacing = Math.toRadians(120);
                defaultShootProcessEndingPoint = new Vector2D(new double[] {0, shootingProcessEndingPointUpdatableRange});
            }
        }
    }

    /**
     * activate this service to control the upper structure
     * */
    public void activateControlUpperStructure() {
        shooter.gainOwnerShip(this);
        intake.gainOwnerShip(this);
        arm.gainOwnerShip(this);
    }
}
