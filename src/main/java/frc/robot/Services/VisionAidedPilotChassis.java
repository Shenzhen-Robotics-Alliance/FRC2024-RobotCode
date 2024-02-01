package frc.robot.Services;

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
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.RobotConfigReader;

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
    private final TargetFieldPositionTracker noteTracker;
    private final AprilTagReferredTarget speakerTarget, amplifierTarget;
    private final XboxController copilotGamePad;

    /**
     * @param chassis
     * @param shooter
     * @param intake
     * @param arm
     * @param speakerTarget
     * @param amplifierTarget
     * @param noteTracker
     * @param robotConfig
     */
    public VisionAidedPilotChassis(SwerveBasedChassis chassis, Shooter shooter, Intake intake, TransformableArm arm, AprilTagReferredTarget speakerTarget, AprilTagReferredTarget amplifierTarget, TargetFieldPositionTracker noteTracker, XboxController copilotGamePad, RobotConfigReader robotConfig) {
        super(chassis, robotConfig);
        this.shooter = shooter;
        this.intake = intake;
        this.arm = arm;
        this.speakerTarget = speakerTarget;
        this.amplifierTarget = amplifierTarget;
        this.noteTracker = noteTracker;
        this.copilotGamePad = copilotGamePad;
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
    }


    private Vector2D chassisPositionWhenCurrentVisionTaskStarted;
    /** only calculated once when the task is initiated */
    private double currentVisionTaskETA;
    private long timeTaskStartedMillis;
    private boolean launchStartedInCurrentTask;
    @Override
    public void periodic() {
        super.periodic();
        final int translationAutoPilotButton = (int)robotConfig.getConfig("control-" + super.controllerName, "translationAutoPilotButton"),
                rotationAutoPilotButton = (int)robotConfig.getConfig("control-" + super.controllerName, "rotationAutoPilotButton");
        final VisionTargetClass currentAimingTargetClass = targetChooser.getSelected();
        final AprilTagReferredTarget currentAimingTarget = switch (currentAimingTargetClass) {
            case SPEAKER -> speakerTarget;
            case AMPLIFIER -> amplifierTarget;
        };

        switch (currentStatus) {
            case MANUALLY_DRIVING -> {
                shooter.setShooterMode(Shooter.ShooterMode.DISABLED, this);
                arm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.DEFAULT, this);
                if (copilotGamePad.getXButton()) intake.startSplit(this); // in case if the Note is stuck
                else intake.turnOffIntake(this);

                if (pilotController.keyOnPress(translationAutoPilotButton))
                    currentStatus = intake.isNoteInsideIntake() ?  Status.SEARCHING_FOR_SHOOT_TARGET : Status.SEARCHING_FOR_NOTE;
                else if (pilotController.keyOnHold(rotationAutoPilotButton))
                    chassis.setRotationalTask(new SwerveBasedChassis.ChassisTaskRotation(SwerveBasedChassis.ChassisTaskRotation.TaskType.FACE_DIRECTION,
                            intake.isNoteInsideIntake() ? getNoteRotation() : getAprilTagTargetRotation(currentAimingTargetClass, currentAimingTarget)),
                            this);
            }
            case SEARCHING_FOR_SHOOT_TARGET -> {
                shooter.setShooterMode(Shooter.ShooterMode.SHOOT, this);
                arm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SHOOT_NOTE, this);
                intake.turnOffIntake(this);

                if (!pilotController.keyOnHold(translationAutoPilotButton))
                    currentStatus = Status.MANUALLY_DRIVING;
                else if (currentAimingTarget.getTargetFieldPositionWithVisibleAprilTags() != null)
                    switch (currentAimingTargetClass) {
                        case SPEAKER -> initiateGoToSpeakerTargetProcess();
                        case AMPLIFIER -> initiateGoToAmplifierProcess();
                    }
            }
            case SEARCHING_FOR_NOTE -> {
                shooter.setShooterMode(Shooter.ShooterMode.DISABLED, this);
                arm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.INTAKE_STANDBY, this);
                intake.turnOffIntake(this);

                if (!pilotController.keyOnHold(translationAutoPilotButton))
                    currentStatus = Status.MANUALLY_DRIVING;
                else if (noteTracker.isTargetVisible(0))
                    initiateGrabNoteProcess();
            }
            case REACHING_TO_SHOOT_TARGET -> {
                intake.turnOffIntake(this);
                switch (currentAimingTargetClass) {
                    case SPEAKER -> proceedGoToSpeakerTarget(translationAutoPilotButton);
                    case AMPLIFIER -> proceedGoToAmplifierTarget(translationAutoPilotButton);
                }
            }
            case GRABBING_NOTE -> {
                shooter.setShooterMode(Shooter.ShooterMode.DISABLED, this);
                arm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.INTAKE, this);

                final double timeSinceTaskStarted = (System.currentTimeMillis() - timeTaskStartedMillis) / 1000.0;
                final BezierCurve currentPath = getPathToNoteTarget();
                chassis.setTranslationalTask(new SwerveBasedChassis.ChassisTaskTranslation(SwerveBasedChassis.ChassisTaskTranslation.TaskType.GO_TO_POSITION,
                        currentPath.getPositionWithLERP(timeSinceTaskStarted / currentVisionTaskETA)), this);
                chassis.setRotationalTask(new SwerveBasedChassis.ChassisTaskRotation(SwerveBasedChassis.ChassisTaskRotation.TaskType.FACE_DIRECTION,
                        getNoteRotation()), this);

                if (!pilotController.keyOnHold(translationAutoPilotButton) || intake.isNoteInsideIntake())
                    currentStatus = Status.MANUALLY_DRIVING;
            }
        }
    }

    /**
     * @return the facing of the chassis such that it faces the target, or the default value if unseen. In radian, zero is to front.
     * */
    private double getAprilTagTargetRotation(VisionTargetClass currentAimingTargetClass, AprilTagReferredTarget currentAimingTarget) {
        final Vector2D targetPosition2D = currentAimingTarget.getTargetFieldPositionWithAprilTags(objectUnseenTimeOut);
        if (targetPosition2D == null)
            return switch (currentAimingTargetClass) {
                case SPEAKER -> speakerDefaultRotation;
                case AMPLIFIER -> amplifierDefaultRotation;
            };
        return Vector2D.displacementToTarget(chassis.positionEstimator.getRobotPosition2D(), targetPosition2D).getHeading() - Math.toRadians(90);
    }

    /**
     * @return the facing of the chassis such that it faces the target, or the default value if unseen. In radian, zero is to front.
     * */
    private double getNoteRotation() {
        final int pov = copilotGamePad.getPOV();
        if (pov == -1)
            return noteDefaultRotation;
        return AngleUtils.simplifyAngle(Math.toRadians(pov + 180));
    }

    private void initiateGoToSpeakerTargetProcess() {
        chassisPositionWhenCurrentVisionTaskStarted = chassis.positionEstimator.getRobotPosition2D();
        currentStatus = Status.REACHING_TO_SHOOT_TARGET;
        final double length = getPathToSpeakerTarget().getLength(10);
        currentVisionTaskETA = length / chassisSpeedLimitWhenAutoAim;
        launchStartedInCurrentTask = false;
        timeTaskStartedMillis = System.currentTimeMillis();
    }

    private void proceedGoToSpeakerTarget(int translationAutoPilotButton) {
        shooter.setShooterMode(Shooter.ShooterMode.SHOOT, this);
        arm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SHOOT_NOTE, this);

        final double timeSinceTaskStarted = (System.currentTimeMillis() - timeTaskStartedMillis) / 1000.0;
        final BezierCurve currentPath = getPathToSpeakerTarget();
        chassis.setTranslationalTask(new SwerveBasedChassis.ChassisTaskTranslation(SwerveBasedChassis.ChassisTaskTranslation.TaskType.GO_TO_POSITION,
                currentPath.getPositionWithLERP(timeSinceTaskStarted / currentVisionTaskETA)), this);
        chassis.setRotationalTask(new SwerveBasedChassis.ChassisTaskRotation(SwerveBasedChassis.ChassisTaskRotation.TaskType.FACE_DIRECTION,
                getAprilTagTargetRotation(VisionTargetClass.SPEAKER, speakerTarget)), this);

        if (!launchStartedInCurrentTask && shooter.shooterReady() && shooter.targetInRange() && arm.transformerInPosition() && chassis.isCurrentRotationalTaskFinished()) {
            // start shooting
            intake.startLaunch(this);
            launchStartedInCurrentTask = true;
        }
        if (timeSinceTaskStarted > currentVisionTaskETA + objectUnseenTimeOut/1000.0 || !intake.isNoteInsideIntake() || !pilotController.keyOnHold(translationAutoPilotButton))
            currentStatus = Status.MANUALLY_DRIVING; // finished or cancelled
    }

    private BezierCurve getPathToSpeakerTarget() {
        return new BezierCurve(new Vector2D(), new Vector2D()); // TODO finish this logic
    }

    private void initiateGoToAmplifierProcess() {
        launchStartedInCurrentTask = false;
        currentStatus = Status.REACHING_TO_SHOOT_TARGET;
    }

    private void proceedGoToAmplifierTarget(int translationAutoPilotButton) {
        shooter.setShooterMode(Shooter.ShooterMode.AMPLIFY, this);
        arm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SCORE_AMPLIFIER, this);

        chassis.setTranslationalTask(new SwerveBasedChassis.ChassisTaskTranslation(SwerveBasedChassis.ChassisTaskTranslation.TaskType.GO_TO_POSITION,
                new Vector2D()), this); // TODO figure out the position to go
        chassis.setRotationalTask(new SwerveBasedChassis.ChassisTaskRotation(SwerveBasedChassis.ChassisTaskRotation.TaskType.FACE_DIRECTION,
                getAprilTagTargetRotation(VisionTargetClass.AMPLIFIER, amplifierTarget)), this);

        if (!launchStartedInCurrentTask && chassis.isCurrentRotationalTaskFinished() && chassis.isCurrentTranslationalTaskFinished()) {
            // start shooting
            intake.startLaunch(this);
            launchStartedInCurrentTask = true;
        }
        if (!intake.isNoteInsideIntake() || !pilotController.keyOnHold(translationAutoPilotButton))
            currentStatus = Status.MANUALLY_DRIVING; // finished or cancelled
    }

    private void initiateGrabNoteProcess() {
        chassisPositionWhenCurrentVisionTaskStarted = chassis.positionEstimator.getRobotPosition2D();
        currentStatus = Status.GRABBING_NOTE;
        final double length = getPathToNoteTarget().getLength(10);
        currentVisionTaskETA = length / chassisSpeedLimitWhenAutoAim;
        intake.startIntake(this);
        timeTaskStartedMillis = System.currentTimeMillis();
    }

    private BezierCurve getPathToNoteTarget() {
        return new BezierCurve(new Vector2D(), new Vector2D()); // TODO finish this logic
    }

    /* TODO put the following to robotConfig */
    private static final long objectUnseenTimeOut = 1000;
    private static final double speakerDefaultRotation = 0;
    private static final double amplifierDefaultRotation = Math.toRadians(90);
    private static final double noteDefaultRotation = Math.toRadians(90);
    private static final double chassisSpeedLimitWhenAutoAim = 5; // m/s
    @Override
    public void updateConfigs() {
        super.updateConfigs();
    }

    /**
     * activate this service to control the upper structure
     * */
    public void activateControlUpperStructure() {
        shooter.gainOwnerShip(this);
        intake.gainOwnerShip(this);
    }
}
