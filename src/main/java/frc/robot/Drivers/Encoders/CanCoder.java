package frc.robot.Drivers.Encoders;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.Drivers.RobotDriverBase;
import frc.robot.Utils.MathUtils.AngleUtils;

public class CanCoder extends RobotDriverBase implements Encoder {
    private final CANcoder encoderInstance;
    private final StatusSignal absolutePositionSignal, velocitySignal;
    private double encoderScaleFactor;
    private double encoderZeroPosition;

    public CanCoder(CANcoder encoderInstance) {
        this(encoderInstance, false);
    }

    public CanCoder(CANcoder encoderInstance, Boolean reversed) {
        this.encoderInstance = encoderInstance;
        this.encoderZeroPosition = 0;
        this.encoderScaleFactor = reversed ? -1 : 1;
        absolutePositionSignal = encoderInstance.getAbsolutePosition();
        velocitySignal = encoderInstance.getVelocity();
        absolutePositionSignal.setUpdateFrequency(200);
        velocitySignal.setUpdateFrequency(200);
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
        // TODO this value is probably incorrect
        return absolutePositionSignal.getValueAsDouble() * Math.PI * 2;
    }

    @Override
    public double getEncoderVelocity() {
        return velocitySignal.getValueAsDouble() * encoderScaleFactor * Math.PI * 2;
    }
}
