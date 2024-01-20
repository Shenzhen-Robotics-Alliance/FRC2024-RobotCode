package frc.robot.Drivers.IMUs;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/**
 * the NavX2IMU implement of the IMU interface
 * does not gather
 */

public class NavX2IMU implements RawGyro {
    private AHRS navx2Instance;

    public NavX2IMU() {
        this.navx2Instance = new AHRS(SPI.Port.kMXP);
    }

    @Override
    public void update() {

    }

    @Override
    public double[] getRawYawPitchRollAngle() {
        return new double[] {
                Math.toRadians(navx2Instance.getYaw()),
                Math.toRadians(navx2Instance.getPitch()),
                Math.toRadians(navx2Instance.getRoll())
        };
    }

    @Override
    public double[] getYawYawPitchRollVelocity() {
        return new double[] {
                Math.toRadians(navx2Instance.getRate()),
                0, 0 };
    }
}
