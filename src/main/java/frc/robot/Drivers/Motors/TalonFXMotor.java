package frc.robot.Drivers.Motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Drivers.Encoders.Encoder;
import frc.robot.Drivers.RobotDriverBase;
import frc.robot.Modules.RobotModuleBase;

public class TalonFXMotor extends RobotDriverBase implements Motor, Encoder {
    private final TalonFX talonFXInstance;
    private final int portID;
    /** encoder is built-in, so they reverse together */
    private double powerAndEncoderScaleFactor;
    private double encoderZeroPosition = 0;
    private double currentPower = 0;
    private boolean enabled;

    public TalonFXMotor(TalonFX talonFXInstance) {
        this(talonFXInstance, false);
    }

    public TalonFXMotor(TalonFX talonFXInstance, boolean reversed) {
        this.powerAndEncoderScaleFactor = reversed ? -1 : 1;
        this.talonFXInstance = talonFXInstance;
        this.portID = talonFXInstance.getBaseID();
        enabled = true;
    }

    @Override
    public int getPortID() {
        return portID;
    }

    @Override
    public void setPower(double power, RobotModuleBase operatorModule) {
        // System.out.println("<-- TalonFX | motor id " + portID + " set power by " + operatorModule + " -->");
        if (!isOwner(operatorModule))
            return;
        talonFXInstance.set(ControlMode.PercentOutput, power * this.powerAndEncoderScaleFactor);
        currentPower = power;
        enabled = true;
    }

    @Override
    public double getCurrentPower() {
        return currentPower;
    }

    @Deprecated
    public void setTargetedPosition(double targetedPosition, RobotModuleBase operatorModule) {
        if (!isOwner(operatorModule))
            return;
        talonFXInstance.set(ControlMode.Position, targetedPosition);
        talonFXInstance.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void onDestroy() {
        disableMotor(ownerModule); // disable the motor, operate as its owner
    }

    @Override
    public void setMotorZeroPowerBehavior(ZeroPowerBehavior behavior, RobotModuleBase operatorModule) {
        if (!isOwner(operatorModule))
            return;
        switch (behavior) {
            case BRAKE: {
                talonFXInstance.setNeutralMode(NeutralMode.Brake);
                break;
            }
            case RELAX: {
                talonFXInstance.setNeutralMode(NeutralMode.Coast);
            }
            default: {
                // unsupported zero power behavior
            }
        }
    }

    @Override
    public void lockMotor(RobotModuleBase operatorModule) {
        setMotorZeroPowerBehavior(ZeroPowerBehavior.BRAKE, operatorModule);
        setPower(0, operatorModule);
    }

    @Override
    public void disableMotor(RobotModuleBase operatorModule) {
        if (!enabled) return; // if already enabled, just skip
        // System.out.println("<-- TalonFX | motor id " + portID + " disabled -->");
        setMotorZeroPowerBehavior(ZeroPowerBehavior.RELAX, operatorModule);
        setPower(0, operatorModule);
        enabled = false;
    }

    public TalonFX getMotorInstance() {
        return talonFXInstance;
    }

    // TODO finish the encoder part of talon motors
    // features: convert the value into radian
    // two modes, the angle of the motor (0~360deg) and added-up value
    @Override
    public void setZeroPosition(double zeroPosition) {
        this.encoderZeroPosition = zeroPosition;
    }

    /** gets the current position, not in radian */
    @Override
    public double getEncoderPosition() {
        return talonFXInstance.getSelectedSensorPosition();
    }

    /** gets the current velocity, not in radian, but in per second */
    @Override
    public double getEncoderVelocity() {
        return talonFXInstance.getSelectedSensorVelocity() * 10;
    }
}
