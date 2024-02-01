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
        REACHING_TO_NOTE,
        /**
         * we are just in front of the note, now we start the intake and grab it
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
    @Override
    public void periodic() {
        super.periodic();
        final int translationAutoPilotButton = (int)robotConfig.getConfig("control-" + super.controllerName, "translationAutoPilotButton"),
                rotationAutoPilotButton = (int)robotConfig.getConfig("control-" + super.controllerName, "rotationAutoPilotButton");
        final AprilTagReferredTarget currentAimingTarget = switch (targetChooser.getSelected()) {
            case SPEAKER -> speakerTarget;
            case AMPLIFIER -> amplifierTarget;
        };

        switch (currentStatus) {
            case MANUALLY_DRIVING -> {
                if (pilotController.keyOnPress(translationAutoPilotButton)) {
                    if (intake.isNoteInsideIntake())
                        currentStatus = Status.SEARCHING_FOR_SHOOT_TARGET;
                    else
                        currentStatus = Status.SEARCHING_FOR_NOTE;
                } else if (pilotController.keyOnHold(rotationAutoPilotButton)) {
                    // TODO rotation logic goes here
                }
            }
            case SEARCHING_FOR_SHOOT_TARGET -> {
                if (!pilotController.keyOnHold(translationAutoPilotButton))
                    currentStatus = Status.MANUALLY_DRIVING;
                else if (currentAimingTarget.getTargetFieldPositionWithVisibleAprilTags() != null)
                    initiateGoToAprilTagTargetProcess();
            }
            case SEARCHING_FOR_NOTE -> {
                if (!pilotController.keyOnHold(translationAutoPilotButton))
                    currentStatus = Status.MANUALLY_DRIVING;
                else if (noteTracker.isTargetVisible(0))
                    initiateGrabNoteProcess();
            }
            // TODO finish the following logic
            case REACHING_TO_SHOOT_TARGET -> {

            }
            case REACHING_TO_NOTE -> {

            }
            case GRABBING_NOTE -> {

            }
        }
    }

    private void initiateGoToAprilTagTargetProcess() {
        chassisPositionWhenCurrentVisionTaskStarted = chassis.positionEstimator.getRobotPosition2D();
        currentStatus = Status.REACHING_TO_SHOOT_TARGET;
        // TODO calculate ETA here
    }

    private void initiateGrabNoteProcess() {
        chassisPositionWhenCurrentVisionTaskStarted = chassis.positionEstimator.getRobotPosition2D();
        currentStatus = Status.REACHING_TO_NOTE;
        // TODO calculate ETA here
    }

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
