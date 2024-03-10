package frc.robot.Utils.ComputerVisionUtils;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Modules.Chassis.SwerveBasedChassis;
import frc.robot.Modules.UpperStructure.Intake;
import frc.robot.Modules.UpperStructure.Shooter;
import frc.robot.Modules.UpperStructure.TransformableArm;
import frc.robot.RobotCore;
import frc.robot.Utils.MathUtils.BezierCurve;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.SpeedCurves;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.SequentialCommandSegment;

public class AutoStageVisionAimBot {
    private final RobotCore robotCore;
    private final long timeUnseenToleranceMillis;
    public AutoStageVisionAimBot(RobotCore robotCore, long timeUnseenToleranceMillis) {
        this.robotCore = robotCore;
        this.timeUnseenToleranceMillis = timeUnseenToleranceMillis;
    }

    public SequentialCommandSegment grabNote(Vector2D assumedNotePosition, Rotation2D desiredRobotRotation, long timeOutMillis) {
        return grabNote(()->true, assumedNotePosition, desiredRobotRotation, timeOutMillis);
    }

    public SequentialCommandSegment grabNote(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D assumedNotePosition, Rotation2D desiredRobotRotation, long timeOutMillis) {
        return grabNote(initiateCondition, assumedNotePosition, desiredRobotRotation, timeOutMillis, false);
    }
    public SequentialCommandSegment grabNote(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D assumedNotePosition, Rotation2D desiredRobotRotation, long timeOutMillis, boolean accelerateShooters) {
        final Timer timer = new Timer();
        timer.start();
        final Vector2D noteLastSeenPosition = assumedNotePosition;
        return new SequentialCommandSegment(
                initiateCondition,
                () -> null,
                timer::reset,
                () -> {
                    robotCore.transformableArm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.INTAKE, null);
                    robotCore.shooter.setShooterMode(accelerateShooters ? Shooter.ShooterMode.PREPARE_TO_SHOOT : Shooter.ShooterMode.DISABLED, null);
                    if (robotCore.transformableArm.transformerInPosition())
                        robotCore.intake.startIntake(null);
                    robotCore.chassisModule.setOrientationMode(SwerveBasedChassis.OrientationMode.FIELD, null);

                    /* TODO: in robot config */
                    final double intakeDistance = 0.2, intakeTime = 0.6, positionDifferenceTolerance = 0.1;
                    final Vector2D
                            noteFieldPositionByCamera = robotCore.noteTarget.getTargetFieldPositionWithAprilTags(timeUnseenToleranceMillis),
                            intakeProcessPath = new Vector2D(new double[] {0, -intakeDistance}),
                            pathAnotherPoint = noteLastSeenPosition.addBy(intakeProcessPath.multiplyBy(desiredRobotRotation).multiplyBy(-1)),
                            pathEndPoint = noteLastSeenPosition.addBy(intakeProcessPath.multiplyBy(desiredRobotRotation));
                    final BezierCurve pathCurve = new BezierCurve(robotCore.positionReader.getRobotPosition2D(), pathAnotherPoint, pathEndPoint);
                    final Vector2D currentDesiredPosition = pathCurve.getPositionWithLERP(timer.get() / intakeTime);
                    robotCore.chassisModule.setTranslationalTask(new SwerveBasedChassis.ChassisTaskTranslation(SwerveBasedChassis.ChassisTaskTranslation.TaskType.GO_TO_POSITION, currentDesiredPosition), null);

                    if (noteFieldPositionByCamera != null && Vector2D.displacementToTarget(noteLastSeenPosition, noteFieldPositionByCamera).getMagnitude() > positionDifferenceTolerance)
                        noteLastSeenPosition.update(noteLastSeenPosition);
                },
                () -> robotCore.transformableArm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.DEFAULT, null),
                () -> timer.get() * 1000 > timeOutMillis || robotCore.intake.isNoteInsideIntake(),
                () -> desiredRobotRotation, () -> desiredRobotRotation
        );
    }

    public SequentialCommandSegment shootWhileMoving(BezierCurve chassisMovementPath, Vector2D assumedSpeakerPosition, long timeOutMillis) {
        return shootWhileMoving(robotCore.intake::isNoteInsideIntake, chassisMovementPath, assumedSpeakerPosition, timeOutMillis);
    }

    public SequentialCommandSegment shootWhileMoving(SequentialCommandSegment.InitiateCondition initiateCondition, BezierCurve chassisMovementPath, Vector2D assumedSpeakerPosition, long timeOutMillis) {
        final Timer timeSinceTaskStarted = new Timer(), timeSinceNoteGone = new Timer();
        timeSinceTaskStarted.start(); timeSinceNoteGone.start();
        return new SequentialCommandSegment(
                initiateCondition,
                () -> chassisMovementPath,
                timeSinceTaskStarted::reset,
                () -> {
                    robotCore.transformableArm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SHOOT_NOTE, null);
                    robotCore.shooter.setShooterMode(Shooter.ShooterMode.SHOOT, null);
                    robotCore.shooter.aimingSystem.defaultTargetFieldPosition = assumedSpeakerPosition;
                    robotCore.chassisModule.setRotationalTask(new SwerveBasedChassis.ChassisTaskRotation(
                            SwerveBasedChassis.ChassisTaskRotation.TaskType.FACE_DIRECTION, robotCore.shooter.aimingSystem.getRobotFacing(robotCore.shooter.getProjectileSpeed())), null);

                    if (robotCore.intake.getCurrentStatus() != Intake.IntakeModuleStatus.LAUNCHING && robotCore.shooter.shooterReady() && robotCore.shooter.targetInRange() && robotCore.transformableArm.transformerInPosition())
                        robotCore.intake.startLaunch(null);
                    if (robotCore.intake.isNoteInsideIntake())
                        timeSinceNoteGone.reset();
                },
                () -> {
                    robotCore.transformableArm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.DEFAULT, null);
                    robotCore.shooter.setShooterMode(Shooter.ShooterMode.DISABLED, null);
                    robotCore.intake.turnOffIntake(null);
                    robotCore.shooter.aimingSystem.defaultTargetFieldPosition = null;
                },
                () -> timeSinceTaskStarted.get() * 1000 > timeOutMillis
                        || timeSinceNoteGone.get() > 0.05,
                () -> null, () -> null,
                SpeedCurves.slowDown, 0.6 // TODO in robotConfig
        );
    }

    public SequentialCommandSegment splitForwardWhileMoving(BezierCurve chassisMovementPath, Rotation2D robotStartingRotation, Rotation2D robotEndingRotation, long timeOutMillis) {
        final Timer timeSinceTaskStarted = new Timer(),
                timeNoteLeftShooterModule = new Timer();
        timeSinceTaskStarted.start();
        timeNoteLeftShooterModule.start();
        return new SequentialCommandSegment(
                robotCore.intake::isNoteInsideIntake,
                () -> chassisMovementPath,
                () -> {
                    timeSinceTaskStarted.reset();
                    robotCore.transformableArm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.DEFAULT, null);
                    robotCore.shooter.setShooterMode(Shooter.ShooterMode.PREPARE_TO_SHOOT, null);
                },
                () -> {
                    if (robotCore.transformableArm.transformerInPosition() && robotCore.shooter.shooterReadyToSplit())
                        robotCore.intake.startLaunch(null);
                    if (robotCore.intake.isNoteInsideIntake())
                        timeNoteLeftShooterModule.reset();
                },
                () -> {
//                    robotCore.transformableArm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.DEFAULT, null);
//                    robotCore.shooter.setShooterMode(Shooter.ShooterMode.DISABLED, null);
                    robotCore.intake.turnOffIntake(null);
                },
                () -> timeSinceTaskStarted.get() * 1000 > timeOutMillis || timeNoteLeftShooterModule.get() > 0.5,
                () -> robotStartingRotation, () -> robotEndingRotation
        );
    }

    public Runnable prepareToShoot() {
        return () -> {
            robotCore.transformableArm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SHOOT_NOTE, null);
            robotCore.shooter.setShooterMode(Shooter.ShooterMode.PREPARE_TO_SHOOT, null);
        };
    }

    public Runnable prepareToShoot(Vector2D defaultTargetPosition) {
        return () -> {
            robotCore.shooter.aimingSystem.defaultTargetFieldPosition = defaultTargetPosition;
            prepareToShoot().run();
        };
    }

    public Runnable prepareToIntake() {
        return () -> robotCore.transformableArm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.INTAKE, null);
    }

    public SequentialCommandSegment waitForArmToLower() {
        return new SequentialCommandSegment(
                () -> true,
                () -> null,
                () -> robotCore.transformableArm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.DEFAULT, null),
                () -> {}, () -> {},
                robotCore.transformableArm::transformerInPosition,
                robotCore.positionReader::getRobotRotation2D, robotCore.positionReader::getRobotRotation2D
        );
    }
}
