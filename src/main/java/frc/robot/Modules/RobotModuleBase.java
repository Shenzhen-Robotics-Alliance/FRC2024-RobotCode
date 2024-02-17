package frc.robot.Modules;

import frc.robot.Drivers.Motors.Motor;
import frc.robot.RobotShell;
import frc.robot.Utils.RobotModuleOperatorMarker;
import frc.robot.Utils.TimeUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeoutException;

/**
 * The template for the classes that controls the different modules of the
 * robot, from chassis to arm and to aim
 * 
 * @author Sam
 * @version 0.1
 */
public abstract class RobotModuleBase extends RobotModuleOperatorMarker {
    private final ExecutorService periodicExecutor = Executors.newSingleThreadExecutor();
    /** the name of the module */
    public final String moduleName;

    /** the desired update frequency, in hz */
    public final int desiredUpdateFrequency;

    /** if the current module is enabled */
    private boolean enabled = true;

    /** the current owner services or modules of the module */
    protected RobotModuleOperatorMarker owner = null;

    /** always add motors to this list */
    protected List<Motor> motors = new ArrayList<>();

    private long previousTimeMillis;

    protected RobotModuleBase(String moduleName) {
        this(moduleName, 64);
    }
    /**
     * public RobotModule(HashMap<String, RobotModule> dependenciesModules,
     * dependency object 1, dependency object 2, ...)
     */
    protected RobotModuleBase(String moduleName, int desiredUpdateFrequency) {
        this.moduleName = moduleName;
        this.desiredUpdateFrequency = desiredUpdateFrequency;
        previousTimeMillis = System.currentTimeMillis();
    }

    public abstract void init();

    /** called during every loop */
    protected abstract void periodic(double dt);

    public void periodic() {
        // System.out.println("<-- base periodic of " + moduleName + ", enabled: " + enabled + "-->");
        if (!enabled) {
            for (Motor motor:motors)
                motor.disableMotor(getMarker());
            TimeUtils.sleep(50);
            return;
        }
        long newTimeMillis = System.currentTimeMillis();
        if (newTimeMillis == previousTimeMillis) {
            /* in case of dt=0 */
            TimeUtils.sleep(1);
            newTimeMillis = System.currentTimeMillis();
        }
        updateConfigs();
        // System.out.println("executing periodic");
        periodic((newTimeMillis - previousTimeMillis) / 1000.0f);
        this.previousTimeMillis = newTimeMillis;
        // System.out.println("<-- end of base periodic -->");
    }

    public Thread getPeriodicUpdateThread() {
        return new Thread(new Runnable() {
            private long t = System.currentTimeMillis();
            private final double periodMS = 1000.0f / desiredUpdateFrequency;
            @Override
            public void run() {
                System.out.println("<-- start of the update thread of " + moduleName + "-->");
                while (true) {
                    if (RobotShell.isFormalCompetition)
                        try {
                            TimeUtils.executeWithTimeOut(periodicExecutor, () -> periodic(), 500);
                        } catch (TimeoutException e) {
                            System.out.println("<-- RobotModuleBase | WARNING!!! Module " + moduleName + " timeout while updating, restarting... ->");
                            onReset();
                            continue;
                        }
                    else
                        periodic();
                    periodic();

                    while (System.currentTimeMillis() - t< periodMS)
                        TimeUtils.sleep(1);
                    // System.out.println("module <" + moduleName + "> update frequency: " + 1000.0f/(System.currentTimeMillis() - t));
                    t = System.currentTimeMillis();
                }
            }
        });
    }

    private RobotModuleBase getMarker() {
        return this;
    }

    /** update robot configs from robotConfigReader, used when debugging the robot override or nothing will be done */
    public void updateConfigs() {}

    /** called to reset module to initial state, you can also call it by the end of init() */
    public abstract void onReset();

    public void reset() {
        this.previousTimeMillis = System.currentTimeMillis();
        onReset();
        enable();
    }

    /** called when the program ends */
    public abstract void onDestroy();
    protected abstract void onEnable();
    protected abstract void onDisable();

    public void enable() {
        if (enabled)
            return;
        System.out.println("<-- Module Base | enabling module " + moduleName + " -->");
        onEnable();
        this.enabled = true;
    }

    public void disable() {
        if (!enabled)
            return;
        System.out.println("<-- Module Base | disabling module " + moduleName + " -->");
        for (Motor motor:motors)
            motor.disableMotor(getMarker());
        onDisable();
        this.enabled = false;
    }

    /**
     * make a service or module the only owner of this module
     * @param owner the robot service or module that is desired to be the owner
     */
    public void gainOwnerShip(RobotModuleOperatorMarker owner) {
        this.owner = owner;
    }

    /**
     * check if a service or module has ownership to this module
     * @param operator the service or module that needs to be checked
     * @return whether it is the only owner of this module
     */
    public boolean isOwner(RobotModuleOperatorMarker operator) {
        return operator == null || operator == owner;
    }

    public long getPreviousUpdateTimeMillis() {
        return previousTimeMillis;
    }
}
