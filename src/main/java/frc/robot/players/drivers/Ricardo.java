package frc.robot.players.drivers;

import frc.robot.Constants.IOConstants;
import frc.robot.Robot;
import frc.robot.players.PlayerConfigs;

public class Ricardo extends PlayerConfigs {
    @Override
    public void getDriverConfig() {
        // Constants
        fullTurnSpeed = 0.40;
        fullDriveSpeed = 0.40;
        fineTurnSpeed = 0.3;
        fineDriveSpeed = 0.2;
        boostDriveSpeed = 1;
        boostTurnSpeed = 1;
        
        // Driving and rotation
        xMovement = applyAxisDeadband(Robot.controller0.getLeftX());
        yMovement = applyAxisDeadband(-Robot.controller0.getLeftY());
        turnMovement = applyAxisDeadband(-Robot.controller0.getRightX());
        boostToggle = Robot.controller0.getRightTriggerAxis() > IOConstants.TRIGGER_DEADBAND;
        fineControlToggle = Robot.controller0.getLeftTriggerAxis() > IOConstants.TRIGGER_DEADBAND;

        // Reef snaps
        snapUp = Robot.controller0.getPOV() == IOConstants.DPAD_UP;
        snapUpRight = Robot.controller0.getPOV() == IOConstants.DPAD_UP_RIGHT;
        snapRight = Robot.controller0.getPOV() == IOConstants.DPAD_RIGHT;
        snapDownRight = Robot.controller0.getPOV() == IOConstants.DPAD_DOWN_RIGHT;
        snapDown = Robot.controller0.getPOV() == IOConstants.DPAD_DOWN;
        snapDownLeft = Robot.controller0.getPOV() == IOConstants.DPAD_DOWN_LEFT;
        snapLeft = Robot.controller0.getPOV() == IOConstants.DPAD_LEFT;
        snapUpLeft = Robot.controller0.getPOV() == IOConstants.DPAD_UP_LEFT;

        robotRelative = Robot.controller0.getRightBumperButton();

        // Auto Align
        autoAlignLeft = Robot.controller0.getXButtonReleased();
        autoAlignRight = Robot.controller0.getAButtonReleased();
        
        // Gyro Reset
        zeroGyro = Robot.controller0.getBButton();

        // Climber toggle
        climberOnline = Robot.controller0.getStartButton(); // Climber engage
    }

    @Override
    public void getOperatorConfig() {
        fineStrafe = 0.05;
        stationPickup = Robot.controller1.getPOV() == IOConstants.DPAD_UP;
        lowPickup = Robot.controller1.getPOV() == IOConstants.DPAD_DOWN;
        shelfReef = Robot.controller1.getXButton();
        lowReef = Robot.controller1.getAButton();
        midReef = Robot.controller1.getBButton();
        highReef = Robot.controller1.getYButton();
        suckerIntake = Robot.controller1.getLeftTriggerAxis() > IOConstants.TRIGGER_DEADBAND;
        suckerEject = Robot.controller1.getLeftBumperButton();
        fineControlWrist = applyAxisDeadband(Robot.controller1.getRightX());
        fineControlElbow = applyAxisDeadband(-Robot.controller1.getLeftY()); // Inverted because joystick y up is negative
        fineControlRotator = applyAxisDeadband(-Robot.controller1.getLeftY()); // Inverted because joystick y up is negative
        moveClimber = applyAxisDeadband(-Robot.controller1.getRightY()); // Inverted because joystick y up is negative

        strafeLeft = Robot.controller1.getPOV() == IOConstants.DPAD_LEFT;
        strafeRight = Robot.controller1.getPOV() == IOConstants.DPAD_RIGHT;
        
        // 2 control schemes, switches when climberOnline is pressed on driver controller
        fineControlCraneEnable = Robot.controller1.getRightTriggerAxis() > IOConstants.TRIGGER_DEADBAND; // Fine control enable
        fineControlClimberEnable = Robot.controller1.getLeftTriggerAxis() > IOConstants.TRIGGER_DEADBAND; // Fine control enable
    }
}
