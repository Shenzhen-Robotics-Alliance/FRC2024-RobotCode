package frc.robot.Services;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Modules.Chassis.SwerveBasedChassis;
import frc.robot.Utils.EasyShuffleBoard;
import frc.robot.Utils.PilotController;
import frc.robot.Utils.RobotConfigReader;
import frc.robot.Utils.MathUtils.Vector2D;

public class PilotChassis extends RobotServiceBase {
    /** the module of the robot's chassis */
    protected final SwerveBasedChassis chassis;

    protected final RobotConfigReader robotConfig;

    /** the center of the translation axis of the stick, in vector */
    private Vector2D controllerTranslationStickCenter;
    /** the center of the rotation axis of the stick */
    private double controllerRotationStickCenter;

    /** the desired heading of the robot */
    private double desiredHeading;

    private final SendableChooser<SwerveBasedChassis.OrientationMode> orientationModeChooser= new SendableChooser<>();
    private final SwerveBasedChassis.OrientationMode defaultOrientationMode = SwerveBasedChassis.OrientationMode.FIELD;

    private final SendableChooser<SwerveBasedChassis.WheelOutputMode> wheelOutputModeChooser = new SendableChooser<>();
    private final SwerveBasedChassis.WheelOutputMode defaultSpeedOutputMode = SwerveBasedChassis.WheelOutputMode.PERCENT_POWER;

    private enum ControllerType {
        RM_BOXER,
        RM_POCKET,
        JOYSTICK,
        XBOX
    }
    private SendableChooser<ControllerType> controllerTypeSendableChooser = new SendableChooser<>();
    private static final ControllerType defaultControllerType = ControllerType.RM_POCKET;

    /**
     * creates a pilot chassis
     * @param chassis
     * @param robotConfig
     * */
    public PilotChassis(SwerveBasedChassis chassis, RobotConfigReader robotConfig) {
        super("pilotChassisService");
        this.chassis = chassis;
        this.robotConfig = robotConfig;
    }

    @Override
    public void init() {
        reset();

    }

    @Override
    public void reset() {
        /* calibrate its center */
//        this.controllerTranslationStickCenter = new Vector2D(
//                new double[] { pilotControllerStick.getX(), pilotControllerStick.getY() });
//        this.controllerRotationStickCenter = pilotControllerStick.getZ();
        this.controllerTranslationStickCenter = new Vector2D();
        this.controllerRotationStickCenter = 0;

        /* add mode choosers */
        for (SwerveBasedChassis.OrientationMode mode: SwerveBasedChassis.OrientationMode.values())
            this.orientationModeChooser.addOption(mode.name(), mode);
        this.orientationModeChooser.setDefaultOption(defaultOrientationMode.name(), defaultOrientationMode);
        SmartDashboard.putData("orientation mode", orientationModeChooser);

        for (SwerveBasedChassis.WheelOutputMode mode: SwerveBasedChassis.WheelOutputMode.values())
            this.wheelOutputModeChooser.addOption(mode.name(), mode);
        this.wheelOutputModeChooser.setDefaultOption(defaultSpeedOutputMode.name(), defaultSpeedOutputMode);
        SmartDashboard.putData("speed control", wheelOutputModeChooser);

        for (ControllerType controllerType: ControllerType.values())
            this.controllerTypeSendableChooser.addOption(controllerType.name(), controllerType);
        this.controllerTypeSendableChooser.setDefaultOption(defaultControllerType.name(), defaultControllerType);
        SmartDashboard.putData("pilot controller type", controllerTypeSendableChooser);

        addResetChassisCommandButtonToDashboard();

        this.chassis.reset();
        this.chassis.gainOwnerShip(this);
    }

    private void addResetChassisCommandButtonToDashboard() {
        SmartDashboard.putData("Reset Chassis", new InstantCommand(() -> {
            chassis.reset();
            addResetChassisCommandButtonToDashboard();
        }));
    }

    private ControllerType previousSelectedController = null;
    protected PilotController pilotController;
    protected String controllerName;
    @Override
    public void periodic() {
        controllerName = "control-" + controllerTypeSendableChooser.getSelected();

        /* if the controller type is switched, we create a new instance */
        if (controllerTypeSendableChooser.getSelected() != previousSelectedController)
            pilotController = new PilotController(robotConfig, controllerName);
        previousSelectedController = controllerTypeSendableChooser.getSelected();

        pilotController.updateKeys();

        /* set the control and orientation mode for wheel speed */
        chassis.setWheelOutputMode(wheelOutputModeChooser.getSelected(), this);
        chassis.setOrientationMode(orientationModeChooser.getSelected(), this);

        /* read and process pilot's translation input */
        final Vector2D translationInput = pilotController.getTranslationalStickValue();
        final int translationAutoPilotButton = (int)robotConfig.getConfig(controllerName, "translationAutoPilotButton");
        SwerveBasedChassis.ChassisTaskTranslation chassisTranslationalTask = new SwerveBasedChassis.ChassisTaskTranslation(
                SwerveBasedChassis.ChassisTaskTranslation.TaskType.SET_VELOCITY,
                /* if autopilot is on, we scale the input down by a factor so that we can search for the target */
                translationInput.multiplyBy(pilotController.keyOnHold(translationAutoPilotButton) ? robotConfig.getConfig("chassis", "lowSpeedModeTranslationalCommandScale"):1)
        );

        /* read and process the pilot's rotation inputs */
        final double rotationInput = pilotController.getRotationalStickValue();
        /* turn it into a task */
        SwerveBasedChassis.ChassisTaskRotation chassisRotationalTask = new SwerveBasedChassis.ChassisTaskRotation(
                SwerveBasedChassis.ChassisTaskRotation.TaskType.SET_VELOCITY,
                rotationInput
        );
        if (pilotController.getRotationalStickValue() != 0) /* when there is rotational input, we record the current heading of the chassis */
            desiredHeading = chassis.getChassisHeading();
        else /* when there is no rotational input, we stay at the previous rotation */
            chassisRotationalTask = new SwerveBasedChassis.ChassisTaskRotation(
                    SwerveBasedChassis.ChassisTaskRotation.TaskType.FACE_DIRECTION,
                    desiredHeading
            );

        EasyShuffleBoard.putNumber("chassis", "rotation maintenance heading", Math.toDegrees(desiredHeading));

        /* calls to the chassis module and pass the desired motion */
        chassis.setTranslationalTask(chassisTranslationalTask, this);
        chassis.setRotationalTask(chassisRotationalTask, this);

        /* lock the chassis if needed */
        final int lockChassisButtonPort = (int) robotConfig.getConfig(controllerName, "lockChassisButtonPort");
        chassis.setChassisLocked(pilotController.keyOnHold(lockChassisButtonPort), this);
    }

    @Override
    public void onDestroy() {
        throw new UnsupportedOperationException("Unimplemented method 'onDestroy'");
    }
}
