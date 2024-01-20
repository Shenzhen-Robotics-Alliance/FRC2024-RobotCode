package frc.robot;

import frc.robot.Services.PilotChassis;
import frc.robot.Utils.PilotController;

public class RemoteControlledRobot extends Robot {
    private final PilotChassis pilotChassis;
    public RemoteControlledRobot() {
        super();

        pilotChassis = new PilotChassis(chassisModule, robotConfig);
        services.add(pilotChassis);
    }

    @Override
    public void updateRobot(boolean isEnabled) {
        // TODO here we need to update some of the controllers
        //  notice that pilot controller is updated in pilot chassis service which is the only place it is used
        super.updateRobot(isEnabled);
    }
}
