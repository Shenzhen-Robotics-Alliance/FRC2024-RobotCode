package frc.robot.Modules;

import frc.robot.Drivers.Motors.Motor;
import frc.robot.Utils.RobotModuleOperatorMarker;
import frc.robot.Utils.Time;

import java.util.ArrayList;
import java.util.List;

/**
 * The template for the classes that controls the different modules of the
 * robot, from chassis to arm and to aim
 * 
 * @author Sam
 * @version 0.1
 */
public abstract class RobotModuleBase extends RobotModuleOperatorMarker {
    /** the name of the module */
    public final String moduleName;

    /** the desired update frequency, in hz */
    public final int desiredUpdateFrequency;

    /** if the current module is enabled */
    private boolean enabled = true;

    /** the current owner services or modules of the module */
    protected List<RobotModuleOperatorMarker> owners = new ArrayList<>(1);

    /** always add motors to this list */
    protected List<Motor> motors = new ArrayList<>();

    private long previousTimeMillis;

    protected RobotModuleBase(String moduleName) {
        this(moduleName, 50);
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
        if (!enabled) {
            for (Motor motor:motors)
                motor.disableMotor(getMarker());
            Time.sleep(50);
            return;
        }
        final long newTimeMillis = System.currentTimeMillis();
        if (newTimeMillis == previousTimeMillis) {
            /* in case of dt=0 */
            Time.sleep(1);
            periodic();
        }
        updateConfigs();
        periodic((newTimeMillis - previousTimeMillis) / 1000.0f);
        this.previousTimeMillis = newTimeMillis;
    }

    public Thread getPeriodicUpdateThread() {
        return new Thread(new Runnable() {
            private long t = System.currentTimeMillis();
            private final double periodMS = 1000.0f / desiredUpdateFrequency;
            @Override
            public void run() {
                while (true) {
                    periodic();

                    while (System.currentTimeMillis() - t< periodMS)
                        Time.sleep(1);
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
    public abstract void resetModule();

    public void reset() {
        this.previousTimeMillis = System.currentTimeMillis();
        resetModule();
        enabled = true;
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
        onDisable();
        this.enabled = false;
    }

    /**
     * cancel the ownership of all services that currently owns to this module
     * */
    public void clearOwners() {
        this.owners = new ArrayList<>(1);
    }

    /**
     * make a service or module one of the owners (does not cancel ownerships of other services)
     * @param owner the robot service or module that is desired to be one of the owner
     */
    public void addOwnerShip(RobotModuleOperatorMarker owner) {
        this.owners.add(owner);
    }

    /**
     * make a service or module the only owner of this module
     * @param owner the robot service or module that is desired to be the owner
     */
    public void gainOwnerShip(RobotModuleOperatorMarker owner) {
        clearOwners();
        addOwnerShip(owner);
    }

    /**
     * check if a service or module has ownership to this module
     * @param operator the service or module that needs to be checked
     * @return whether it is one of or the only owner of this module
     */
    public boolean isOwner(RobotModuleOperatorMarker operator) {
        return owners.contains(operator);
    }
}
