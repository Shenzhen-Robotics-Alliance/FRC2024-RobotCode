package frc.robot.Modules.UpperStructure;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.RobotModuleOperatorMarker;

public class LEDStatusLights extends RobotModuleBase {
    public enum LEDStatus {
        OFF,
        BLINK,
        ON
    }
    private static final double hz = 10;

    private LEDStatus currentStatus = LEDStatus.OFF;
    final Timer t = new Timer();
    private final Solenoid light;
    public LEDStatusLights(int channel) {
        super("LED-Status-Lights");
        this.light = new Solenoid(PneumaticsModuleType.CTREPCM, channel);
    }

    @Override
    public void init() {
        onReset();
    }

    @Override
    protected void periodic(double dt) {
        switch (currentStatus) {
            case ON -> light.set(true);
            case OFF -> light.set(false);
            case BLINK -> light.set(blink());
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
        this.currentStatus = ledStatus;
    }
}
