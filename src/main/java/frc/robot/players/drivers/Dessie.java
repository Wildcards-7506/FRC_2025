package frc.robot.players.drivers;

import frc.robot.Constants.IOConstants;
import frc.robot.Robot;

public class Dessie extends PlayerConfigs {

    @Override
    public void getOperatorConfig() {

        fineControlWrist = applyAxisDeadband(Robot.controller1.getRightY());
        fineControlElbow = applyAxisDeadband(-Robot.controller1.getLeftY()); // Inverted because joystick y up is negative
        
        moveAnchor = applyAxisDeadband(-Robot.controller1.getRightY()); // Inverted because joystick y up is negative
        moveWinch = applyAxisDeadband(-Robot.controller1.getLeftY());
        
        // 2 control schemes, switches when climberOnline is pressed on driver controller
        fineControlCraneEnable = Robot.controller1.getRightTriggerAxis() > IOConstants.TRIGGER_DEADBAND; // Fine control enable
        fineControlClimberEnable = Robot.controller1.getLeftTriggerAxis() > IOConstants.TRIGGER_DEADBAND; // Fine control enable
    }
}
