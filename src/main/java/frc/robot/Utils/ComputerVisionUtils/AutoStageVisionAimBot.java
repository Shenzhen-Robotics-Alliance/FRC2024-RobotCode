package frc.robot.Utils.ComputerVisionUtils;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Modules.Chassis.SwerveBasedChassis;
import frc.robot.Modules.UpperStructure.Intake;
import frc.robot.Modules.UpperStructure.Shooter;
import frc.robot.Modules.UpperStructure.TransformableArm;
import frc.robot.RobotCore;
import frc.robot.Utils.MathUtils.BezierCurve;
import frc.robot.Utils.MathUtils.Rotation2D;
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
        final Timer timer = new Timer();
        timer.start();
        return new SequentialCommandSegment(
                initiateCondition,
                () -> null,
                timer::reset,
                () -> {
                    robotCore.intake.startIntake(null);
                    robotCore.transformableArm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.INTAKE, null);
                    robotCore.shooter.setShooterMode(Shooter.ShooterMode.DISABLED, null);
                },
                () -> {
                    robotCore.chassisModule.setOrientationMode(SwerveBasedChassis.OrientationMode.FIELD, null);
                    final Vector2D noteFieldPosition2D = robotCore.noteTarget.getTargetFieldPositionWithAprilTags(timeUnseenToleranceMillis),
                            desiredFieldPosition = noteFieldPosition2D == null ? assumedNotePosition : noteFieldPosition2D;
                    robotCore.chassisModule.setTranslationalTask(new SwerveBasedChassis.ChassisTaskTranslation(
                            SwerveBasedChassis.ChassisTaskTranslation.TaskType.GO_TO_POSITION, desiredFieldPosition
                    ), null);
                },
                () -> timer.get() * 1000 > timeOutMillis || robotCore.intake.isNoteInsideIntake(),
                () -> desiredRobotRotation, () -> desiredRobotRotation
        );
    }

    public SequentialCommandSegment shootWhileMoving(BezierCurve chassisMovementPath, Vector2D assumedSpeakerPosition, long timeOutMillis) {
        return shootWhileMoving(robotCore.intake::isNoteInsideIntake, chassisMovementPath, assumedSpeakerPosition, timeOutMillis);
    }
    public SequentialCommandSegment shootWhileMoving(SequentialCommandSegment.InitiateCondition initiateCondition, BezierCurve chassisMovementPath, Vector2D assumedSpeakerPosition, long timeOutMillis) {
        final Timer timer = new Timer();
        timer.start();
        return new SequentialCommandSegment(
                initiateCondition,
                () -> chassisMovementPath,
                timer::reset,
                () -> {
                    robotCore.transformableArm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SHOOT_NOTE, null);
                    robotCore.shooter.setShooterMode(Shooter.ShooterMode.SHOOT, null);
                    robotCore.shooter.aimingSystem.defaultTargetFieldPosition = assumedSpeakerPosition;
                    robotCore.chassisModule.setRotationalTask(new SwerveBasedChassis.ChassisTaskRotation(
                            SwerveBasedChassis.ChassisTaskRotation.TaskType.FACE_DIRECTION, robotCore.shooter.aimingSystem.getRobotFacing(robotCore.shooter.getProjectileSpeed())), null);

                    if (robotCore.shooter.shooterReady() && robotCore.shooter.targetInRange() && robotCore.transformableArm.transformerInPosition())
                        robotCore.intake.startLaunch(null);
                },
                () -> {
                    robotCore.transformableArm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.DEFAULT, null);
                    robotCore.shooter.setShooterMode(Shooter.ShooterMode.DISABLED, null);
                    robotCore.intake.turnOffIntake(null);
                    robotCore.shooter.aimingSystem.defaultTargetFieldPosition = null;
                },
                () -> timer.get() * 1000 > timeOutMillis || !robotCore.intake.isNoteInsideIntake(),
                () -> null, () -> null
        );
    }

    public Runnable prepareToShoot() {
        return () -> {
            robotCore.transformableArm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.SHOOT_NOTE, null);
            robotCore.shooter.setShooterMode(Shooter.ShooterMode.SHOOT, null);
        };
    }

    public Runnable prepareToShoot(Vector2D defaultTargetPosition) {
        return () -> {
            robotCore.shooter.aimingSystem.defaultTargetFieldPosition = defaultTargetPosition;
            prepareToShoot().run();
        };
    }

    public Runnable prepareToIntake() {
        return () -> {
            robotCore.transformableArm.setTransformerDesiredPosition(TransformableArm.TransformerPosition.INTAKE, null);
        };
    }
}
