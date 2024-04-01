package frc.robot.Modules.UpperStructure;

import edu.wpi.first.wpilibj.*;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.LEDAnimation;
import frc.robot.Utils.RobotModuleOperatorMarker;

public class LEDStatusLights extends RobotModuleBase {
    final Timer t = new Timer();
    final AddressableLED led;
    final AddressableLEDBuffer buffer;

    LEDAnimation animation;
    double hz;
    public LEDStatusLights(AddressableLED led, AddressableLEDBuffer buffer) {
        super("LED-Status-Lights");
        this.led = led;
        this.buffer = buffer;
    }

    @Override
    public void init() {
        onReset();
    }

    @Override
    protected void periodic(double dt) {
        final double t_scaled = (t.get() * hz) % 1;

        animation.play(buffer, t_scaled);
        led.setData(buffer);
    }

    @Override
    public void onReset() {
        t.start();
        onDisable();
    }

    @Override
    protected void onEnable() {
        animation = new LEDAnimation.Rainbow();
        hz = 0.6;
    }

    @Override
    protected void onDisable() {
        // TODO make it update even when disabled
        animation = new LEDAnimation.Breathe(0, 200, 255);
        hz = 0.4;
    }

    public void setAnimation(LEDAnimation animation, double hz, RobotModuleOperatorMarker operator) {
        if (!isOwner(operator)) return;
        this.animation = animation;
        this.hz = hz;
    }
}
