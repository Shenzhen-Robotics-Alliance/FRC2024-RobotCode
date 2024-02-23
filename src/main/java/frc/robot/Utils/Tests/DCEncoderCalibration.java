package frc.robot.Utils.Tests;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Utils.EasyShuffleBoard;
import frc.robot.Utils.MathUtils.AngleUtils;

public class DCEncoderCalibration implements SimpleRobotTest {
    final DutyCycleEncoder dutyCycleEncoder = new DutyCycleEncoder(1);
    @Override
    public void testStart() {

    }

    @Override
    public void testPeriodic() {
        final double encoderRawReading = dutyCycleEncoder.getAbsolutePosition(),
                encoderReadingRadians = encoderRawReading * Math.PI * 2;
        System.out.println("DcEncoder absolute position (rad): " + encoderReadingRadians);
        EasyShuffleBoard.putNumber("DCEncoder", "absolute position (radian)", encoderReadingRadians);
        EasyShuffleBoard.putNumber("DCEncoder", "distance", dutyCycleEncoder.getDistance());
        EasyShuffleBoard.putNumber("DCEncoder", "distance per rotation", dutyCycleEncoder.getDistancePerRotation());
    }
}
