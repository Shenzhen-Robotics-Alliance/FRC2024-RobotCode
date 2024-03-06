package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.CommandSequenceGenerator;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.List;

public class TestTurnToRotationAuto implements CommandSequenceGenerator {
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        SequentialCommandFactory sequentialCommandFactory = new SequentialCommandFactory(robotCore);
        commandSegments.add(sequentialCommandFactory.faceDirection(
                new Rotation2D(Math.toRadians(90)),
                () -> System.out.println("start"),
                () -> System.out.println("periodic"),
                () -> System.out.println("end")
        ));
        return commandSegments;
    }
}
