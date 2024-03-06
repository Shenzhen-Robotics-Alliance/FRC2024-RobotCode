package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.CommandSequenceGenerator;
import frc.robot.Utils.ComputerVisionUtils.AutoStageVisionAimBot;
import frc.robot.Utils.MathUtils.BezierCurve;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.SpeedCurves;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.List;

public class RedDS3 implements CommandSequenceGenerator {
    public static final Vector2D
            robotStartingPosition = new Vector2D(new double[] {2.9, 0.3}),
            assumedSpeakerPosition = new Vector2D(new double[] {1.45, 0}),
            allianceFrontNotePosition = new Vector2D(new double[] {2.9, 2.9}),
            allianceFrontNoteIntakePosition = new Vector2D(new double[] {2.9, 2.4}),
            secondNote1 = new Vector2D(new double[]{2.9, 2.5}),
            secondNote2 = secondNote1.addBy(new Vector2D(Math.toRadians(-135), 1.35)),
            secondNote4 = new Vector2D(new double[]{2.1, 4}),
            secondNote3 = secondNote4.addBy(new Vector2D(Math.toRadians(-100), 1.4)),
            moveToThirdNoteMid = new Vector2D(new double[] {2.4, 5.8}),
            centerLineRighterNotePosition = new Vector2D(new double[] {1.68, 8.27}),
            moveToThirdNoteEnd = centerLineRighterNotePosition.addBy(new Vector2D(new double[] {0, -0.6})),
            moveFromThirdNoteBackMid = secondNote4.addBy(new Vector2D(Math.toRadians(30), 1.2)),
            shootThirdNoteMid = secondNote4.addBy(new Vector2D(Math.toRadians(-150), 1)),
            stageZoneCenter = new Vector2D(new double[] {0, 4.8}),
            moveToFourthNote2 = new Vector2D(new double[] {-0.7, 5.8}),
            moveToFourthNote3 = new Vector2D(new double[] {0.2, 6.2}),
            CenterLineLefterNotePosition = new Vector2D(new double[] {-1,68, 8.27}),
            fourthNoteIntakePosition = CenterLineLefterNotePosition.addBy(new Vector2D(new double[] {0.5, -0.5}));







    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final double timeScaleAutoIntake = 0.5;
        final long intakeTimeOut = 3000;
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, robotStartingPosition, new Rotation2D(Math.toRadians(90)));
        final AutoStageVisionAimBot aimBot = new AutoStageVisionAimBot(robotCore, intakeTimeOut);
        commandSegments.add(commandFactory.calibratePositionEstimator());

        /* move out, shoot the preloaded note */
        commandSegments.add(aimBot.shootWhileMoving(
                new BezierCurve(robotStartingPosition, allianceFrontNoteIntakePosition),
                assumedSpeakerPosition,
                4000
        ));

        /* the note in front */
        commandSegments.add(commandFactory.justDoIt(aimBot.prepareToIntake()));
        commandSegments.add(commandFactory.moveToPointIf(
                () -> true, allianceFrontNoteIntakePosition, ()->{}, () -> {}, () ->{}, new Rotation2D(Math.toRadians(180)))
        );
        commandSegments.add(aimBot.grabNote(
                () -> true,
                allianceFrontNotePosition,
                new Rotation2D(Math.toRadians(180)),
                intakeTimeOut,
                false));
        /* shoot */
        commandSegments.add(aimBot.shootWhileMoving(
                new BezierCurve(secondNote1, secondNote2, secondNote3, secondNote4),
                assumedSpeakerPosition,
                4000
        ));

        /* move to the third note */
        commandSegments.add(new SequentialCommandSegment(
                () -> true,
                () -> new BezierCurve(secondNote4, moveToThirdNoteMid, moveToThirdNoteEnd),
                aimBot.prepareToIntake(), ()->{}, ()->{},
                () -> true,
                () -> new Rotation2D(Math.toRadians(-180)), () -> new Rotation2D(Math.toRadians(-155)),
                SpeedCurves.easeOut,timeScaleAutoIntake
        ));
        commandSegments.add(aimBot.grabNote(centerLineRighterNotePosition, new Rotation2D(Math.toRadians(-155)), intakeTimeOut));
        // TODO over here
        return commandSegments;
    }
}
