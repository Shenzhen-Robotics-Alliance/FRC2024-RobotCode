package frc.robot.Drivers.Encoders;

import com.ctre.phoenix.sensors.CANCoder;
import frc.robot.Drivers.RobotDriverBase;
import frc.robot.Utils.MathUtils.AngleUtils;

public class CanCoder extends RobotDriverBase implements Encoder {
    private final CANCoder encoderInstance;
    private double encoderScaleFactor;
    private double encoderZeroPosition;

    public CanCoder(CANCoder encoderInstance) {
        this(encoderInstance, false);
    }

    public CanCoder(CANCoder encoderInstance, Boolean reversed) {
        this.encoderInstance = encoderInstance;
        this.encoderZeroPosition = 0;
        this.encoderScaleFactor = reversed ? -1 : 1;
    }

    @Override
    public void setZeroPosition(double zeroPosition) {
        this.encoderZeroPosition = zeroPosition;
    }

    @Override
    public int getPortID() {
        return encoderInstance.getDeviceID();
    }

    /**
     * get the reading of this encoder
     * returns the angle of the encoder, notice zero is to the direct right
     * positive is counter-clockwise, just as how we do it on the coordinate system
     */
    @Override
    public double getEncoderPosition() {
        final double differenceFromZeroPosition = AngleUtils.getActualDifference(encoderZeroPosition, getRawSensorReading());
        return AngleUtils.simplifyAngle(differenceFromZeroPosition * encoderScaleFactor);
    }

    /** the raw sensor reading, converted to radian */
    public double getRawSensorReading() {
        return Math.toRadians(encoderInstance.getAbsolutePosition());
    }

    @Override
    public double getEncoderVelocity() {
        return Math.toRadians(encoderInstance.getVelocity()) * encoderScaleFactor;
    }
}
