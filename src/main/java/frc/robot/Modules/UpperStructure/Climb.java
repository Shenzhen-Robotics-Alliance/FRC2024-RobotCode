package frc.robot.Modules.UpperStructure;

import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.MechanismControllers.EncoderMotorMechanism;
import frc.robot.Utils.MechanismControllers.MechanismController;
import frc.robot.Utils.MechanismControllers.SimpleArmController;
import frc.robot.Utils.RobotConfigReader;
import frc.robot.Utils.RobotModuleOperatorMarker;

public class Climb extends RobotModuleBase {
    /**
     * positive is up
     */
    private final EncoderMotorMechanism leftClimb, rightClimb;
    private final RobotConfigReader robotConfig;
    private ClimbTask leftClimbTask, rightClimbTask;

    private SimpleArmController controller;
    public Climb(EncoderMotorMechanism leftClimb, EncoderMotorMechanism rightClimb, RobotConfigReader robotConfig) {
        super("climb");
        this.leftClimb = leftClimb;
        this.rightClimb = rightClimb;
        this.robotConfig = robotConfig;
    }

    @Override
    public void init() {
        onReset();
    }

    @Override
    protected void periodic(double dt) {
        leftClimb.setController(controller);
        rightClimb.setController(controller);
        switch (leftClimbTask.taskType) {
            case SET_POWER ->
                leftClimb.setPower(leftClimbTask.taskValue, this);
            case SET_POSITION -> {
                controller.desiredPosition = leftClimbTask.taskValue;
                leftClimb.updateWithController(this);
            }
        }

        updateClimbGivenTask(leftClimb, leftClimbTask);
        updateClimbGivenTask(rightClimb, rightClimbTask);
    }

    private void updateClimbGivenTask(EncoderMotorMechanism climb, ClimbTask task) {
        switch (task.taskType) {
            case SET_POWER ->
                    climb.setPower(task.taskValue, this);
            case SET_POSITION -> {
                controller.desiredPosition = task.taskValue;
                climb.updateWithController(this);
            }
        }
    }

    @Override
    public void updateConfigs() {
        controller = new SimpleArmController(
                robotConfig.getConfig("climb", "movingPower"),
                robotConfig.getConfig("climb", "movingPower"),
                robotConfig.getConfig("climb", "tolerance"),
                robotConfig.getConfig("climb", "movingPower"),
                robotConfig.getConfig("climb", "movingPower"),
                robotConfig.getConfig("climb", "tolerance"),
                false
        );
    }

    @Override
    public void onReset() {
        leftClimb.gainOwnerShip(this);
        rightClimb.gainOwnerShip(this);

        leftClimb.setCurrentPositionAsZeroPosition();
        rightClimb.setCurrentPositionAsZeroPosition();

        leftClimbTask = rightClimbTask = new ClimbTask(ClimbTask.TaskType.SET_POWER, 0);
    }

    public void setLeftClimbTask(ClimbTask task, RobotModuleOperatorMarker operator) {
        if (isOwner(operator)) this.leftClimbTask = task;
    }

    public void setRightClimbTask(ClimbTask task, RobotModuleOperatorMarker operator) {
        if (isOwner(operator)) this.rightClimbTask = task;
    }

    public static final class ClimbTask {
        public enum TaskType {
            SET_POWER,
            SET_POSITION
        }

        public final TaskType taskType;
        public final double taskValue;
        public ClimbTask(TaskType taskType, double taskValue) {
            this.taskType = taskType;
            this.taskValue = taskValue;
        }
    }
}
