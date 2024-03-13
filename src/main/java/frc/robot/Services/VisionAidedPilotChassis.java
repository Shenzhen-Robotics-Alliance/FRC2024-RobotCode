package frc.robot.Services;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Modules.Chassis.SwerveBasedChassis;
import frc.robot.Modules.UpperStructure.Intake;
import frc.robot.Modules.UpperStructure.LEDStatusLights;
import frc.robot.Modules.UpperStructure.Shooter;
import frc.robot.Modules.UpperStructure.TransformableArm;
import frc.robot.Utils.ComputerVisionUtils.AprilTagReferredTarget;
import frc.robot.Utils.EasyShuffleBoard;
import frc.robot.Utils.MathUtils.*;
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
    private final AprilTagReferredTarget speakerTarget, amplifierTarget, noteTarget;
    private final XboxController copilotGamePad;
    private final DriverStation.Alliance alliance;
    private final LEDStatusLights red, green, blue;


    /**
     * @param chassis
     * @param shooter
     * @param intake
     * @param arm
     * @param copilotGamePad
     * @param robotConfig
     */
    public VisionAidedPilotChassis(SwerveBasedChassis chassis, Shooter shooter, Intake intake, TransformableArm arm, AprilTagReferredTarget speakerTarget, AprilTagReferredTarget amplifierTarget, AprilTagReferredTarget noteTarget, XboxController copilotGamePad, RobotConfigReader robotConfig, LEDStatusLights red, LEDStatusLights green, LEDStatusLights blue) {
        super(chassis, robotConfig);
        this.shooter = shooter;
        this.intake = intake;
        this.arm = arm;

        this.noteTarget = noteTarget;
        this.speakerTarget = speakerTarget;
        this.amplifierTarget = amplifierTarget;
        this.copilotGamePad = copilotGamePad;
        this.red = red;
        this.green = green;
        this.blue = blue;
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

        activateControlUpperStructure();

        red.gainOwnerShip(this); green.gainOwnerShip(this);  blue.gainOwnerShip(this);
    }


    /** the robot's position when the current task started, 0.4 seconds in advance of the current chassis position (predicted by velocity) */
    private Vector2D chassisPositionWhenCurrentVisionTaskStarted;
    /** the position of the current target, when it is last seen, this is always updated when a visual task started and is updated during the task process whenever target is seen */
    private Vector2D currentVisualTargetLastSeenPosition;
    /** only calculated once when the task is initiated */
    private double currentVisionTaskETA;
    private double currentIntakeTaskFacing;
    private long timeTaskStartedMillis;
    @Override
    public void periodic() {
        if (copilotGamePad.getStartButton()) {
            reset();
            activateControlUpperStructure();
        }

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
        if (copilotGamePad.getLeftStickButton() && copilotGamePad.getRightStickButton()) {
            reset();
            chassis.reset();
        }

        switch (currentStatus) {
            case MANUALLY_DRIVING -> {
                shooter.setShooterMode(Shooter.ShooterMode.DISABLED, this);
                if (copilotGamePad.getBButton()) {
                    arm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SPLIT, this);
                    if (arm.transformerInPosition())
                        intake.startSplit(this); // in case if the Note is stuck
                }
                else {
                    intake.turnOffIntake(this);
                    arm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.DEFAULT, this);
                }
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


                if (!currentAimingTarget.isVisible(aimingTimeUnseenToleranceMS))
                    System.out.println("<-- VAPC | waiting for " + currentAimingTargetClass + " to show up -->");

                final boolean speakerAutoApproach = pilotController.getRawAxis(5) > 0; // TODO pilot controller

                SmartDashboard.putNumber("speaker auto approach", speakerAutoApproach ? 1:0);
                if (currentAimingTarget.isVisible(aimingTimeUnseenToleranceMS) && speakerAutoApproach)
                    switch (currentAimingTargetClass) {
                        case SPEAKER -> initiateGoToSpeakerTargetProcess();
                        case AMPLIFIER -> initiateGoToAmplifierProcess();
                    }
                else if (!pilotController.keyOnHold(translationAutoPilotButton))
                    if ((arm.transformerInPosition() && shooter.shooterReady()) || currentAimingTargetClass == VisionTargetClass.AMPLIFIER)
                        intake.startLaunch(this);
                if (!intake.isNoteInsideIntake())
                    currentStatus = Status.MANUALLY_DRIVING;
            }
            case SEARCHING_FOR_NOTE -> {
                updateChassisPositionWhenTaskStarted();

                shooter.setShooterMode(Shooter.ShooterMode.DISABLED, this);
                arm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.INTAKE, this);
                startIntakeWheels();

                if (!pilotController.keyOnHold(translationAutoPilotButton) || intake.isNoteInsideIntake())
                    currentStatus = Status.MANUALLY_DRIVING;
                else if (noteTarget.isVisible())
                    initiateGrabNoteProcess(pilotController.keyOnHold(rotationAutoPilotButton));
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
//        System.out.println("<-- VAPC | current status: " + currentStatus + "-->");

        final Vector2D speakerPosition = shooter.aimingSystem.getRelativePositionToTarget(shooter.getProjectileSpeed());
        if (speakerPosition != null) {
            EasyShuffleBoard.putNumber("auto-aim", "aiming system relative pos (x)", speakerPosition.getX());
            EasyShuffleBoard.putNumber("auto-aim", "aiming system relative pos (y)", speakerPosition.getY());
        }

        red.setCurrentStatus(arm.malFunctioning() ?
                LEDStatusLights.LEDStatus.BLINK :
                (intake.malFunctioning() ? LEDStatusLights.LEDStatus.ON : LEDStatusLights.LEDStatus.OFF),
                this);

        blue.setCurrentStatus(intake.isNoteInsideIntake() ?
                        LEDStatusLights.LEDStatus.ON :
                        (noteTarget.isVisible(500) ? LEDStatusLights.LEDStatus.BLINK : LEDStatusLights.LEDStatus.OFF),
                this);

        green.setCurrentStatus(switch (currentStatus) {
            case MANUALLY_DRIVING, SEARCHING_FOR_SHOOT_TARGET, SEARCHING_FOR_NOTE, GRABBING_NOTE -> LEDStatusLights.LEDStatus.OFF;
            case REACHING_TO_SHOOT_TARGET -> (shooter.shooterReady() && arm.transformerInPosition()) ? LEDStatusLights.LEDStatus.ON :
                    (speakerTarget.isVisible(500) ? LEDStatusLights.LEDStatus.BLINK : LEDStatusLights.LEDStatus.OFF);
        }, this);
    }

    private void startIntakeWheels() {
        if (arm.transformerInPosition())
            intake.startIntake(this);
    }

    private void updateChassisPositionWhenTaskStarted() {
        chassisPositionWhenCurrentVisionTaskStarted = chassis.positionEstimator.getRobotPosition2D().addBy(chassis.positionEstimator.getRobotVelocity2D().multiplyBy(chassisReactionDelay));
    }

    /**
     * @return the facing of the chassis such that it faces the target, or the default value if unseen. In radian, zero is to front.
     * */
    private double getAprilTagTargetRotation(VisionTargetClass currentAimingTargetClass, AprilTagReferredTarget currentAimingTarget) {
        final Vector2D targetFieldPosition = currentAimingTarget.getTargetFieldPositionWithAprilTags(autoFaceTargetTimeUnseenToleranceMS);
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
        if (!updateTargetPositionIfSeen(speakerTarget)) return; // failed when unseen

        currentStatus = Status.REACHING_TO_SHOOT_TARGET;
        final double length = getPathToSpeakerTarget().getLength(10);
        currentVisionTaskETA = length / chassisSpeedLimitWhenAutoAim;
        timeTaskStartedMillis = System.currentTimeMillis();
    }

    private static final SpeedCurves.SpeedCurve autoApproachSpeedCurve = SpeedCurves.easeInOut;
    private void proceedGoToSpeakerTarget(int translationAutoPilotButton) {
        shooter.setShooterMode(Shooter.ShooterMode.SHOOT, this);
        arm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SHOOT_NOTE, this);

        /* if seen, update the position */
        // updateTargetPositionIfSeen(speakerTarget);

        final double timeSinceTaskStarted = (System.currentTimeMillis() - timeTaskStartedMillis) / 1000.0;
        final BezierCurve currentPath =
                getPathToSweetSpot();
//                getPathToSpeakerTarget();


        final Vector2D currentPathPositionWithLERP = currentPath.getPositionWithLERP(autoApproachSpeedCurve.getScaledT(timeSinceTaskStarted / currentVisionTaskETA)),
                displacementToSpeaker = Vector2D.displacementToTarget(currentPathPositionWithLERP, currentVisualTargetLastSeenPosition);
        final double yPositionLowerConstrain = Math.abs(displacementToSpeaker.getX()) <= speakerImpactSpacingWidth /2 ?
                distanceToWallConstrainInFrontOfSpeaker: distanceToWallConstrain;
        chassis.setTranslationalTask(new SwerveBasedChassis.ChassisTaskTranslation(SwerveBasedChassis.ChassisTaskTranslation.TaskType.GO_TO_POSITION,
                new Vector2D(new double[] {currentPathPositionWithLERP.getX(), Math.max(currentPathPositionWithLERP.getY(), currentVisualTargetLastSeenPosition.getY() + yPositionLowerConstrain)})), this);
        chassis.setRotationalTask(new SwerveBasedChassis.ChassisTaskRotation(SwerveBasedChassis.ChassisTaskRotation.TaskType.FACE_DIRECTION,
                shooter.aimingSystem.getRobotFacing(shooter.getProjectileSpeed(), currentVisualTargetLastSeenPosition)), this);

        if (intake.getCurrentStatus() != Intake.IntakeModuleStatus.LAUNCHING && shooter.shooterReady() && shooter.targetInRange() && arm.transformerInPosition() && chassis.isCurrentRotationalTaskFinished()) {
            // start shooting
            intake.startLaunch(this);
        }
        if (
                timeSinceTaskStarted > currentVisionTaskETA + aimingTimeUnseenToleranceMS/1000.0
                        || !intake.isNoteInsideIntake()
                        || !pilotController.keyOnHold(translationAutoPilotButton)
                        || !speakerTarget.isVisible(aimingTimeUnseenToleranceMS))
            currentStatus = Status.MANUALLY_DRIVING; // finished or cancelled
    }

    private static final double centerZoneAimHorizontalRange = 0.7;
    private static final int shootFromFarSpotControllerAxis = 2;
    private static final Vector2D centerSweetSpotNear = new Vector2D(new double[] {0, 2}),
            leftSweetSpotNear = new Vector2D(new double[] {-1.4, 1.4}),
            rightSweetSpotNear = new Vector2D(new double[] {1.4, 1.4}),
            centerSweetSpotFar = new Vector2D(new double[] {0, 2.5}),
            leftSweetSpotFar = new Vector2D(new double[] {-1.7, 1.7}),
            rightSweetSpotFar = new Vector2D(new double[] {1.7, 1.7});

    private BezierCurve getPathToSweetSpot() {
        final Vector2D relativePositionToSpeaker = Vector2D.displacementToTarget(currentVisualTargetLastSeenPosition, chassisPositionWhenCurrentVisionTaskStarted);
        return new BezierCurve(chassisPositionWhenCurrentVisionTaskStarted, currentVisualTargetLastSeenPosition.addBy(
                getSweetSpot(relativePositionToSpeaker)
        ));
    }

    private Vector2D getSweetSpot(Vector2D relativePositionToSpeaker) {
        final boolean useFarSweetSpot = pilotController.getRawAxis(shootFromFarSpotControllerAxis) > 0;
        if (relativePositionToSpeaker.getX() < -centerZoneAimHorizontalRange)
            return useFarSweetSpot ? leftSweetSpotFar : leftSweetSpotNear;
        if (relativePositionToSpeaker.getX() > centerZoneAimHorizontalRange)
            return useFarSweetSpot ? rightSweetSpotFar : rightSweetSpotNear;
        return useFarSweetSpot ? centerSweetSpotFar : centerSweetSpotNear;
    }

    @Deprecated
    private BezierCurve getPathToSpeakerTarget() {
        if (Vector2D.displacementToTarget(chassisPositionWhenCurrentVisionTaskStarted, currentVisualTargetLastSeenPosition.addBy(shootingSweetSpot)).getMagnitude() < 1000)
            return new BezierCurve(chassisPositionWhenCurrentVisionTaskStarted, currentVisualTargetLastSeenPosition.addBy(shootingSweetSpot)); // goes a straight line
        final boolean pilotSpecifyingShootingProcessEndPoint = pilotController.getTranslationalStickValue().getMagnitude() > 0.4; // only when significant movement
        final Vector2D
                /* the position of the ending point of the path of the shooting task, relative to the shooting sweet-spot  */
                shootingProcessEndPointFromSweetSpotDeviation = pilotSpecifyingShootingProcessEndPoint ?
                new Vector2D(new double[] {pilotController.getTranslationalStickValue().getX(), Math.max(0, pilotController.getTranslationalStickValue().getY())})
                        .multiplyBy(shootingProcessEndingPointUpdatableRange) :
                shootProcessEndingPointInReferenceToShootingSweetSpotByDefault,
                /* the position of the ending point of the path of the shooting task, relative to the speaker  */
                shootingProcessEndPoint = shootingSweetSpot.addBy(shootingProcessEndPointFromSweetSpotDeviation),
                /* the middle point of the path, relative to the shooting sweet-spot  */
                shootingProcessAnotherPoint = shootingSweetSpot.addBy(shootingProcessEndPointFromSweetSpotDeviation.multiplyBy(-0.7)), // TODO in config
                shootingProcessEndPointFieldPosition = shootingProcessEndPoint.addBy(currentVisualTargetLastSeenPosition),
                shootingProcessAnotherPointFieldPosition = shootingProcessAnotherPoint.addBy(currentVisualTargetLastSeenPosition);

        return new BezierCurve(
                chassisPositionWhenCurrentVisionTaskStarted, // starting from the chassis initial position during task
                chassisPositionWhenCurrentVisionTaskStarted.addBy(new Vector2D(
                        Vector2D.displacementToTarget(chassisPositionWhenCurrentVisionTaskStarted, shootingProcessAnotherPointFieldPosition).getHeading(),
                        Vector2D.displacementToTarget(shootingProcessAnotherPointFieldPosition, shootingProcessEndPointFieldPosition).getMagnitude()
                        )),
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

    private void initiateGrabNoteProcess(boolean useRotationAutoPilotRotationAsIntakeFacing) {
        if (!updateTargetPositionIfSeen(noteTarget)) return; // fails if note not seen

        /*
        * of the pilot is holding the rotation AP button, we use the rotation AP as the note facing
        * otherwise, we use the current chassis facing
        * the robot will face this direction during the whole moving process
         * */
        this.currentIntakeTaskFacing = useRotationAutoPilotRotationAsIntakeFacing ?
                getNoteRotation() :
                chassis.positionEstimator.getRobotRotation();
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
                currentPath.getPositionWithLERP(autoApproachSpeedCurve.getScaledT(timeSinceTaskStarted / currentVisionTaskETA))), this);
        chassis.setRotationalTask(new SwerveBasedChassis.ChassisTaskRotation(SwerveBasedChassis.ChassisTaskRotation.TaskType.FACE_DIRECTION,
                currentIntakeTaskFacing), this);

        if (!pilotController.keyOnHold(translationAutoPilotButton) || intake.isNoteInsideIntake())
            currentStatus = Status.MANUALLY_DRIVING;
    }

    private BezierCurve getPathToNoteTarget() {
        final Vector2D
                /* the position of the ending point of the path of the grabbing task, relative to the note target  */
                grabbingProcessEndPointFromNoteDeviation =
                new Vector2D(new double[] {intakeCenterHorizontalBiasFromCamera, -grabbingNoteDistance}).multiplyBy(new Rotation2D(currentIntakeTaskFacing)), // by default, we move backwards in relative to the robot, but we need to convert this to in relative to field by rotating it.
                /* the position of the ending point of the path of the grabbing task, relative to the speaker */
                grabbingProcessEndingPoint = currentVisualTargetLastSeenPosition.addBy(grabbingProcessEndPointFromNoteDeviation),
                grabbingProcessEndingAnotherPoint = currentVisualTargetLastSeenPosition.addBy(new Vector2D(new double[] {0, grabbingPathControlPointDistance}));

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
        final Vector2D targetFieldPosition = target.getTargetFieldPositionWithVisibleAprilTags();
        if (targetFieldPosition == null) return false;
        if (this.currentVisualTargetLastSeenPosition== null || Vector2D.displacementToTarget(this.currentVisualTargetLastSeenPosition, targetFieldPosition).getMagnitude() > robotConfig.getConfig("vision-autopilot", "updateTargetPositionTolerance"))
            this.currentVisualTargetLastSeenPosition = targetFieldPosition;
        return true;
    }

    private long autoFaceTargetTimeUnseenToleranceMS, aimingTimeUnseenToleranceMS;
    private double speakerDefaultRotation;
    private Vector2D amplifyingPositionToAmplifier;
    /** in radian, zero is front */
    private double amplifyingDefaultFacing;
    /** in radian, zero is front */
    private double grabbingNoteDefaultFacing;
    /** the amount of distance to travel when intake is sucking the note */
    private double grabbingNoteDistance, grabbingPathControlPointDistance, intakeCenterHorizontalBiasFromCamera, chassisSpeedLimitWhenAutoAim; // m/s

    /** avoid impact */
    private double speakerImpactSpacingWidth, distanceToWallConstrainInFrontOfSpeaker, distanceToWallConstrain, positionToSpeakerXConstrain;
    private Vector2D shootingSweetSpot;
    /** the pilot can specify the spot of shooting, by this amount of distance away from the sweet spot */
    private double shootingProcessEndingPointUpdatableRange;
    /** the default shoot process ending point, in reference to the shooting sweet spot */
    private Vector2D shootProcessEndingPointInReferenceToShootingSweetSpotByDefault;
    /** how many seconds does the chassis need to react */
    private double chassisReactionDelay;
    @Override
    public void updateConfigs() {
        super.updateConfigs();
        autoFaceTargetTimeUnseenToleranceMS = (long) robotConfig.getConfig("vision-autopilot", "autoFaceTargetTimeUnseenToleranceMS");
        aimingTimeUnseenToleranceMS = (long) robotConfig.getConfig("vision-autopilot", "aimingTimeUnseenToleranceMS");
        /* TODO read from robotConfig */
        this.intakeCenterHorizontalBiasFromCamera = -0.05;
        grabbingNoteDistance = 0.2;
        grabbingPathControlPointDistance = 0.4;
        chassisSpeedLimitWhenAutoAim = 2;
        shootingSweetSpot = new Vector2D(new double[] {0, 2});
        shootingProcessEndingPointUpdatableRange = 1.4;
        chassisReactionDelay = 0.4;

        speakerImpactSpacingWidth = 0.6;
        distanceToWallConstrainInFrontOfSpeaker = 1.5;
        distanceToWallConstrain = 0.8;
        switch (alliance) {
            case Red -> {
                amplifyingPositionToAmplifier = new Vector2D(new double[] {0, -0.4});
                speakerDefaultRotation = Math.toRadians(180);
                amplifyingDefaultFacing = Math.toRadians(-90);
                grabbingNoteDefaultFacing = Math.toRadians(-120);
                shootProcessEndingPointInReferenceToShootingSweetSpotByDefault = new Vector2D(new double[] {-shootingProcessEndingPointUpdatableRange, 0});
            }
            case Blue -> {
                amplifyingPositionToAmplifier = new Vector2D(new double[] {0, 0.4});
                speakerDefaultRotation = Math.toRadians(180);
                amplifyingDefaultFacing = Math.toRadians(90);
                grabbingNoteDefaultFacing = Math.toRadians(120);
                shootProcessEndingPointInReferenceToShootingSweetSpotByDefault = new Vector2D(new double[] {shootingProcessEndingPointUpdatableRange, 0});
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
