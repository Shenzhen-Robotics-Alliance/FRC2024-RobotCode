package frc.robot.Modules.UpperStructure;

import frc.robot.Drivers.Encoders.Encoder;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Services.RobotServiceBase;
import frc.robot.Utils.EasyShuffleBoard;
import frc.robot.Utils.MathUtils.LookUpTable;
import frc.robot.Utils.MathUtils.StatisticsUtils;
import frc.robot.Utils.MechanismControllers.ArmGravityController;
import frc.robot.Utils.MechanismControllers.EncoderMotorMechanism;
import frc.robot.Utils.RobotConfigReader;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class TransformableArm extends RobotModuleBase {
    private final EncoderMotorMechanism armLifterMechanism;
    private final Encoder armEncoder;
    private final Shooter shooterModule;
    private final ArmGravityController armController;
    private double errorAsArmReady = 0;
    private final RobotConfigReader robotConfig;
    public enum TransformerPosition {
        /** the position of the arm such that the robot is balanced */
        DEFAULT,
        /** the position at which the intake spinner can touch the ground  */
        INTAKE,
        /** the position where the shooter points at the target, notice that the specific position is determined by the aiming system */
        SHOOT_NOTE,
        /** the position where the shooter points at the amplifier */
        SCORE_AMPLIFIER
    }
    private TransformerPosition desiredPosition;
    private double desiredEncoderPosition;
    /** in radian */
    private final Map<TransformerPosition, Double> desiredEncoderPositionTable = new HashMap<>();

    public TransformableArm(Motor armLifterMotor, Encoder armEncoder, RobotConfigReader robotConfig) {
        this(armLifterMotor, armEncoder, null, robotConfig);
    }

    /**
     * crates a transformable arm instance
     * @param armLifterMotor the lifter motor, with positive direction is in harmony with the encoder
     * @param armEncoder the lifter encoder, with positive direction is in harmony with the motor, the zero position of the encoder will be set to the position when the module is initialized
     * @param robotConfig the config xml file
     */
    public TransformableArm(Motor armLifterMotor, Encoder armEncoder, Shooter shooterModule, RobotConfigReader robotConfig) {
        super("Transformable-Intake", 128);
        this.shooterModule = shooterModule;
        super.motors.add(armLifterMotor);
        this.armEncoder = armEncoder;
        this.armLifterMechanism = new EncoderMotorMechanism(armEncoder, armLifterMotor);
        this.armController = new ArmGravityController(new ArmGravityController.ArmProfile(0, 0, 0, 0,0,0,0 ,0,0, null), armLifterMechanism.getEncoderPosition());
        this.armLifterMechanism.setController(armController);
        this.robotConfig = robotConfig;
    }

    @Override
    public void init() {
        this.onReset();
        updateConfigs();
    }

    @Override
    protected void periodic(double dt) {
        EasyShuffleBoard.putNumber("arm", "module dt", (int)(dt*1000));
        EasyShuffleBoard.putNumber("arm", "current desired position in module", desiredEncoderPosition);
        // System.out.println("arm current position: " + desiredPosition);

        long t;
        t = System.currentTimeMillis();

        armController.goToDesiredPosition(desiredEncoderPosition = desiredEncoderPositionTable.get(desiredPosition));

        if (System.currentTimeMillis() - t > 100)
            System.out.println("<-- update arm controller position complete, dt: " + (System.currentTimeMillis() - t) + " -->");

        /* disabled when shooter test */
        if (this.desiredPosition == TransformerPosition.SHOOT_NOTE && shooterModule != null) {
            desiredEncoderPosition = desiredEncoderPositionTable.get(TransformerPosition.SHOOT_NOTE) + shooterModule.getArmPositionWithAimingSystem();
            desiredEncoderPosition = Math.max(Math.toRadians(robotConfig.getConfig("arm", "lowerPositionLimit")), desiredEncoderPosition);
            desiredEncoderPosition = Math.min(Math.toRadians(robotConfig.getConfig("arm", "upperPositionLimit")), desiredEncoderPosition);
            armController.updateDesiredPosition(desiredEncoderPosition);
        }

        if (System.currentTimeMillis() - t > 100)
            System.out.println("<-- get shooter angle from aiming system complete, dt: " + (System.currentTimeMillis() - t) + " -->");
        t = System.currentTimeMillis();

        armLifterMechanism.updateWithController(this);

        if (System.currentTimeMillis() - t > 100)
            System.out.println("<-- update arm with controller complete, dt: " + (System.currentTimeMillis() - t) + " -->");
    }

    /**
     * for test only
     * @param aimingAngle the angle to aim, in reference to default shoot position and in radian
     * */
    @Deprecated
    public void updateShootingPosition(double aimingAngle, RobotServiceBase operatorService) {
        if (!isOwner(operatorService) || this.desiredPosition != TransformerPosition.SHOOT_NOTE) return;

        armController.updateDesiredPosition((desiredEncoderPositionTable.get(TransformerPosition.SHOOT_NOTE) + aimingAngle));
    }

    @Override
    public void updateConfigs() {
        final List<Double> encoderPositions = new ArrayList<>(), gravityTorques = new ArrayList<>();
        int i = 0; while (true) {
            try {
                encoderPositions.add(Math.toRadians(robotConfig.getConfig("arm", "encoderPosition" + i)));
                gravityTorques.add(robotConfig.getConfig("arm", "gravityTorque"+ i));
                i++;
            } catch (NullPointerException end) { break; }
        }
        final LookUpTable gravityTorqueLookUpTable = new LookUpTable(StatisticsUtils.toArray(encoderPositions), StatisticsUtils.toArray(gravityTorques));

        errorAsArmReady = Math.toRadians(robotConfig.getConfig("arm", "errorTolerance")) * robotConfig.getConfig("arm", "errorToleranceAsInPosition");
        this.armController.updateArmProfile(new ArmGravityController.ArmProfile(
                robotConfig.getConfig("arm", "maximumPower"),
                Math.toRadians(robotConfig.getConfig("arm", "errorStartDecelerate")),
                robotConfig.getConfig("arm", "minPowerToMove"),
                Math.toRadians(robotConfig.getConfig("arm", "errorTolerance")),
                robotConfig.getConfig("arm", "feedForwardTime"),
                robotConfig.getConfig("arm", "errorAccumulationProportion"),
                Math.toRadians(robotConfig.getConfig("arm", "maxAcceleration")),
                Math.toRadians(robotConfig.getConfig("arm", "maxVelocity")),
                robotConfig.getConfig("arm", "inAdvanceTime"),
                gravityTorqueLookUpTable
        ));

        for (TransformerPosition transformerPosition:TransformerPosition.values())
            desiredEncoderPositionTable.put(transformerPosition, Math.toRadians(robotConfig.getConfig("arm", "position-"+transformerPosition.name())));
    }

    @Override
    public void onReset() {
        updateConfigs();
        this.desiredPosition = TransformerPosition.DEFAULT;
        desiredEncoderPosition = desiredEncoderPositionTable.get(desiredPosition);
        this.armController.reset(armEncoder.getEncoderPosition());
        this.armLifterMechanism.gainOwnerShip(this);

        this.armEncoder.setZeroPosition(robotConfig.getConfig("arm", "encoderZeroPositionRadians"));
        this.armLifterMechanism.setMotorZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE, this);
        this.armLifterMechanism.setSoftEncoderLimit(Math.toRadians(robotConfig.getConfig("arm", "lowerPositionLimit")), Math.toRadians(robotConfig.getConfig("arm", "upperPositionLimit")));
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
        // TODO: bug over here, sometimes the required is 75, the default shooting angle
        System.out.println("transformer error: " + Math.toDegrees(Math.abs(armEncoder.getEncoderPosition() - desiredEncoderPosition)) + ", tolerance: " + Math.toDegrees(errorAsArmReady) + ", required: " + Math.toDegrees(desiredEncoderPosition) + ", actual: " + Math.toDegrees(armEncoder.getEncoderPosition()));
        return Math.abs(armEncoder.getEncoderPosition() - desiredEncoderPosition) < errorAsArmReady;
    }
}
