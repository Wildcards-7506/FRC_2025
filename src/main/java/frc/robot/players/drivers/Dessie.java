package frc.robot.players.drivers;

import frc.robot.Constants.IOConstants;
import frc.robot.Robot;
import frc.robot.players.PlayerConfigs;

public class Dessie extends PlayerConfigs {
    @Override
    public void getDriverConfig() {
        // Not Applicable
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
