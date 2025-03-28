package frc.robot.players.drivers;

import frc.robot.Constants.IOConstants;
import frc.robot.Robot;
import frc.robot.players.PlayerConfigs;

public class Dessie extends PlayerConfigs {
    @Override
    public void getDriverConfig() {
        // Constants
        fullTurnSpeed = 0.40;
        fullDriveSpeed = 0.30;
        fineTurnSpeed = 0.3;
        fineDriveSpeed = 0.1;
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
        fineStrafe = 0.05;
        stationPickup = Robot.controller1.getRightBumperButton();
        shelfReef = Robot.controller1.getXButton();
        lowReef = Robot.controller1.getAButton();
        midReef = Robot.controller1.getBButton();
        highReef = Robot.controller1.getYButton();
        algaeHigh = Robot.controller1.getPOV() == IOConstants.DPAD_UP;
        algaeLow = Robot.controller1.getPOV() == IOConstants.DPAD_DOWN;
        suckerIntake = Robot.controller1.getLeftTriggerAxis() > IOConstants.TRIGGER_DEADBAND;
        suckerEject = Robot.controller1.getLeftBumperButton();
        fineControlWrist = applyAxisDeadband(Robot.controller1.getRightY());
        fineControlElbow = applyAxisDeadband(-Robot.controller1.getLeftY()); // Inverted because joystick y up is negative
        moveAnchor = applyAxisDeadband(-Robot.controller1.getRightY()); // Inverted because joystick y up is negative
        moveWinch = applyAxisDeadband(-Robot.controller1.getLeftY());

        strafeLeft = Robot.controller1.getPOV() == IOConstants.DPAD_LEFT;
        strafeRight = Robot.controller1.getPOV() == IOConstants.DPAD_RIGHT;
        
        // 2 control schemes, switches when climberOnline is pressed on driver controller
        fineControlCraneEnable = Robot.controller1.getRightTriggerAxis() > IOConstants.TRIGGER_DEADBAND; // Fine control enable
        fineControlClimberEnable = Robot.controller1.getLeftTriggerAxis() > IOConstants.TRIGGER_DEADBAND; // Fine control enable
    }
}
