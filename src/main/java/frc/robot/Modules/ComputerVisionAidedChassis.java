package frc.robot.Modules;

import frc.robot.Drivers.Visions.TargetFieldPositionTracker;
import frc.robot.Modules.Chassis.SwerveBasedChassis;

/**
 * TODO: here, we write a status machine that connects the pilot service to the robot chassis and the vision module
 *
 * */
public class ComputerVisionAidedChassis extends RobotModuleBase{
    private final SwerveBasedChassis chassis;
    private final TargetFieldPositionTracker aprilTagTracker, gamePieceTracker;
    private enum AimBotStatus {
        /** not requested to do auto aim, following pilot's command */
        UNUSED,
        /** moving to the shooting position, following the Bézier curve */
        AIMING,
        /** moving through the shooting spot,  */
    }
    public ComputerVisionAidedChassis(SwerveBasedChassis chassis, TargetFieldPositionTracker aprilTagTracker, TargetFieldPositionTracker gamePieceTracker) {
        super("Vision-Aided-Chassis", 100);
        this.chassis = chassis;
        this.aprilTagTracker = aprilTagTracker;
        this.gamePieceTracker = gamePieceTracker;
    }


    @Override
    public void init() {

    }

    @Override
    protected void periodic(double dt) {

    }

    @Override
    public void resetModule() {

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
}
