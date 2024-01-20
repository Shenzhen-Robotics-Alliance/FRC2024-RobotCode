package frc.robot.Drivers.IMUs;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.Timer;

public class PigeonsIMU implements RawGyro {
    private Pigeon2 pigeonInstance;
    private double lastYaw;
    private Timer dt;
    /** in degrees per second */
    private double yawVelocity;
    public PigeonsIMU(int portID) {
        this.pigeonInstance = new Pigeon2(portID);
        this.pigeonInstance.configFactoryDefault();
        this.dt = new Timer();
        dt.start();
        yawVelocity = 0;
    }

    @Override
    public void update() {
        double currentYaw = getRawYawPitchRollAngle()[0];
        this.yawVelocity = (currentYaw - lastYaw) / dt.get(); // the yaw value does not reset when going around 0, zero we can measure velocity as in linear motions

        lastYaw = currentYaw;
        dt.reset();
    }

    @Override
    public double[] getRawYawPitchRollAngle() {
        return new double[] {
                Math.toRadians(pigeonInstance.getYaw()),
                Math.toRadians(pigeonInstance.getPitch()),
                Math.toRadians(pigeonInstance.getRoll())
        };
    }

    @Override
    public double[] getYawYawPitchRollVelocity() {
        return new double[] {
                Math.toRadians(yawVelocity),
                0,
                0
        };
    }
}
