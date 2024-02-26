package frc.robot.Modules.UpperStructure;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.RobotModuleOperatorMarker;

public class LEDStatusLights extends RobotModuleBase {
    public enum LEDStatus {
        OFF,
        SEEING_NOTE,
        GRABBING_NOTE,
        HOLDING_NOTE,
        SHOOTING_NOTE,
        VISION_MALFUNCTION,
        NOTE_SENSOR_MALFUNCTION
    }
    private static final double hz = 6;

    private LEDStatus currentStatus;
    final Timer t = new Timer();
    private final Solenoid
            redLight = new Solenoid(PneumaticsModuleType.CTREPCM, 0),
            greenLight = new Solenoid(PneumaticsModuleType.CTREPCM, 1),
            blue = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
    public LEDStatusLights() {
        super("LED-Status-Lights");
    }

    @Override
    public void init() {
        onReset();
    }

    @Override
    protected void periodic(double dt) {
        switch (currentStatus) {
            case OFF -> {
                redLight.set(false);
                greenLight.set(false);
                blue.set(false);
            }
            case SEEING_NOTE -> {
                redLight.set(false);
                greenLight.set(false);
                blue.set(true);
            }
            case GRABBING_NOTE -> {
                redLight.set(false);
                greenLight.set(false);
                blue.set(blink());
            }
            case HOLDING_NOTE -> {
                redLight.set(false);
                greenLight.set(true);
                blue.set(false);
            }
            case SHOOTING_NOTE -> {
                redLight.set(false);
                greenLight.set(blink());
                blue.set(false);
            }
            case VISION_MALFUNCTION -> {
                redLight.set(true);
                greenLight.set(false);
                blue.set(false);
            }
            case NOTE_SENSOR_MALFUNCTION -> {
                redLight.set(blink());
                greenLight.set(false);
                blue.set(false);
            }
        }
    }

    private boolean blink() {
        return Math.sin(t.get() * hz * Math.PI) > 0;
    }

    @Override
    public void onReset() {
        this.currentStatus = LEDStatus.OFF;
        t.start();
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

    public void setCurrentStatus(LEDStatus ledStatus, RobotModuleOperatorMarker operator) {
        if (!isOwner(operator))
            return;
        switch (ledStatus) {
            case OFF, SEEING_NOTE, GRABBING_NOTE, HOLDING_NOTE, SHOOTING_NOTE -> {
                if (this.currentStatus == LEDStatus.NOTE_SENSOR_MALFUNCTION || this.currentStatus == LEDStatus.VISION_MALFUNCTION)
                    return;
            }
            case NOTE_SENSOR_MALFUNCTION -> {
                if (this.currentStatus == LEDStatus.VISION_MALFUNCTION)
                    return;
            }
        }
        this.currentStatus = ledStatus;
    }
}
