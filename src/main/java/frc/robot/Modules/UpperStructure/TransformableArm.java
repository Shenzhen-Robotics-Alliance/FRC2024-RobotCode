package frc.robot.Modules.UpperStructure;

import frc.robot.Drivers.Encoders.Encoder;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Services.RobotServiceBase;
import frc.robot.Utils.MechanismControllers.ArmGravityController;
import frc.robot.Utils.MechanismControllers.EncoderMotorMechanism;
import frc.robot.Utils.RobotConfigReader;

import java.util.HashMap;
import java.util.Map;

public class TransformableArm extends RobotModuleBase {
    // TODO we have to finish this module after we test the arm gravity controller
    private final EncoderMotorMechanism armLifterMechanism;
    private final Encoder armEncoder;
    private final ArmGravityController armController;
    private double errorAsArmReady;
    private final double radianPerEncoderTick;
    private final RobotConfigReader robotConfig;
    public enum TransformerPosition {
        /** the position of the arm such that the robot is balanced */
        DEFAULT,
        /** the position where the arm is standing by for intake without touching the ground and can move to intake very fast */
        INTAKE_STANDBY,
        /** the position at which the intake spinner can touch the ground  */
        INTAKE,
        /** the position where the shooter points at the target, notice that the specific position is determined by the aiming system */
        SHOOT_NOTE,
        /** the position where the shooter points at the amplifier */
        SCORE_AMPLIFIER
    }
    private TransformerPosition desiredPosition;
    private Map<TransformerPosition, Double> desiredEncoderPositionTable = new HashMap<>();

    /**
     * crates a transformable arm instance
     * @param armLifterMotor the lifter motor, with positive direction is in harmony with the encoder
     * @param armEncoder the lifter encoder, with positive direction is in harmony with the motor, the zero position of the encoder will be set to the position when the module is initialized
     * @param robotConfig the config xml file
     */
    public TransformableArm(Motor armLifterMotor, Encoder armEncoder, RobotConfigReader robotConfig) {
        super("Transformable-Intake");
        super.motors.add(armLifterMotor);
        this.armEncoder = armEncoder;
        this.armLifterMechanism = new EncoderMotorMechanism(armEncoder, armLifterMotor);
        this.armController = new ArmGravityController(new ArmGravityController.ArmProfile(0,0,0,0,0 ,null));
        this.armLifterMechanism.setController(armController);
        this.robotConfig = robotConfig;

        this.radianPerEncoderTick = Math.PI * 2 / robotConfig.getConfig("arm", "overallGearRatio") / robotConfig.getConfig("arm", "encoderTicksPerRevolution");
    }

    @Override
    public void init() {
        armEncoder.setCurrentPositionAsZeroPosition();
        this.resetModule();
        updateConfigs();
    }

    @Override
    protected void periodic(double dt) {
        System.out.println("arm current position: " + desiredPosition);
    }

    @Override
    public void updateConfigs() {

    }

    @Override
    public void resetModule() {
        this.desiredPosition = TransformerPosition.DEFAULT;
        this.armController.reset(armEncoder.getEncoderPosition());
        this.armLifterMechanism.gainOwnerShip(this);
    }

    @Override
    public void onDestroy() {

    }

    @Override
    protected void onEnable() {

    }

    @Override
    protected void onDisable() {

    }

    public void setTransformerDesiredPosition(TransformerPosition desiredPosition, RobotServiceBase operatorService) {
        if (!isOwner(operatorService))
            return;
        this.desiredPosition = desiredPosition;
    }

    public boolean transformerInPosition() {
        return true;
    }
}
