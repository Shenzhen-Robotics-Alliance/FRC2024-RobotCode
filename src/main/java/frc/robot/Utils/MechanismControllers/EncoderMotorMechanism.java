package frc.robot.Utils.MechanismControllers;

import frc.robot.Drivers.Encoders.Encoder;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Modules.RobotModuleBase;

public class EncoderMotorMechanism implements Encoder, Motor {
    private final Encoder encoder;
    private final Motor motor;
    private MechanismController controller = null;
    private static final double ticksPerRevolution = 2048; // TODO make all units the same and not convert it here

    public EncoderMotorMechanism(Encoder encoder, Motor motor) {
        this.encoder = encoder;
        this.motor = motor;
    }

    @Override
    public void setPower(double power, RobotModuleBase operatorModule) {
        motor.setPower(power, operatorModule);
    }

    @Override
    public double getCurrentPower() {
        return motor.getCurrentPower();
    }

    @Override
    public void setMotorZeroPowerBehavior(ZeroPowerBehavior behavior, RobotModuleBase operatorModule) {
        motor.setMotorZeroPowerBehavior(behavior, operatorModule);
    }

    @Override
    public void disableMotor(RobotModuleBase operatorModule) {
        motor.disableMotor(operatorModule);
    }

    @Override
    public void lockMotor(RobotModuleBase operatorModule) {
        motor.lockMotor(operatorModule);
    }

    @Override
    public void setZeroPosition(double zeroPosition) {
        encoder.setZeroPosition(zeroPosition);
    }

    @Override
    public double getEncoderPosition() {
        return encoder.getEncoderPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return encoder.getEncoderVelocity();
    }

    @Override
    public void gainOwnerShip(RobotModuleBase ownerModule) {
        motor.gainOwnerShip(ownerModule);
        encoder.gainOwnerShip(ownerModule);
    }

    @Override
    public void onDestroy() {
        motor.onDestroy();
    }

    @Override
    public int getPortID() {
        return motor.getPortID();
    }

    public void setController(MechanismController controller) {
        this.controller = controller;
    }

    public void updateWithController(RobotModuleBase operatorModule) {
        if (controller == null) motor.setPower(0, operatorModule);
        else setPower(controller.getMotorPower(encoder.getEncoderVelocity(), encoder.getEncoderPosition()), operatorModule);
    }

    @Deprecated
    public double getRotterRPM() {
        return this.getEncoderVelocity() / ticksPerRevolution * 60;
    }
}
