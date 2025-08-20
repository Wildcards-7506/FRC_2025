package frc.robot.players.drivers;

import frc.robot.Constants.IOConstants;
import frc.robot.Robot;
import frc.robot.players.PlayerConfigs;

public class Ricardo extends PlayerConfigs {
    @Override
    public void getDriverConfig() {
        // Constants
        fullTurnSpeed = 0.50;
        fullDriveSpeed = 0.50;
        fineTurnSpeed = 0.3; // current default state
        fineDriveSpeed = 0.2; // current default state
        boostDriveSpeed = 1;
        boostTurnSpeed = 1;
        
        // Driving and rotation
        xMovement = applyAxisDeadband(Robot.controller0.getLeftX());
        yMovement = applyAxisDeadband(-Robot.controller0.getLeftY());
        turnMovement = applyAxisDeadband(-Robot.controller0.getRightX());
        boostToggle = Robot.controller0.getRightTriggerAxis() > IOConstants.TRIGGER_DEADBAND;
        fineControlToggle = Robot.controller0.getLeftTriggerAxis() > IOConstants.TRIGGER_DEADBAND;

        robotRelative = Robot.controller0.getRightBumperButton();
        
        // Gyro Reset
        zeroGyro = Robot.controller0.getBButton();

        // Climber toggle
        // TODO: Disabled temporarily until climber code is done
        climberOnline = Robot.controller0.getStartButton(); // Climber engage
    }

    @Override
    public void getOperatorConfig() {
        //Not Applicable
    }
}
