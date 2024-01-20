package frc.robot.Drivers.Encoders;

import frc.robot.Modules.RobotModuleBase;

public interface Encoder {
    /** set a selected position to be the referred origin */
    void setZeroPosition(double zeroPosition);

    /**
     * get the port of the current motor
     */
    int getPortID();

    /** get the current value of this encoder, converted to radian */
    double getEncoderPosition();

    /** get the velocity of this encoder, in radian per second */
    double getEncoderVelocity();

    /** gain ownership to this encoder */
    void gainOwnerShip(RobotModuleBase ownerModule);
}
