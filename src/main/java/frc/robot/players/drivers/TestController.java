package frc.robot.players.drivers;

import frc.robot.Constants.IOConstants;
import frc.robot.Robot;
import frc.robot.players.PlayerConfigs;

public class TestController extends PlayerConfigs {
    @Override
    public void getDriverConfig() {
        //Constants
        fullTurnSpeed = 0.40;
        fullDriveSpeed = 0.30;
        boostTurnSpeed = 1;
        boostDriveSpeed = 1;
        fineTurnSpeed = 0.1;
        fineDriveSpeed = 0.1;
        
        //Driving and rotation
        xMovement = applyAxisDeadband(Robot.controller0.getLeftX());
        yMovement = applyAxisDeadband(-Robot.controller0.getLeftY());
        turnMovement = applyAxisDeadband(-Robot.controller0.getRightX());
        boostToggle = Robot.controller0.getRightTriggerAxis() > IOConstants.TRIGGER_DEADBAND;
        fineControlToggle = Robot.controller0.getLeftTriggerAxis() > IOConstants.TRIGGER_DEADBAND;
        snapUp = Robot.controller0.getPOV() == IOConstants.DPAD_UP;
        snapRight = Robot.controller0.getPOV() == IOConstants.DPAD_RIGHT;
        snapDown = Robot.controller0.getPOV() == IOConstants.DPAD_DOWN;
        snapLeft = Robot.controller0.getPOV() == IOConstants.DPAD_LEFT;
        robotRelative = Robot.controller0.getRightBumperButton();
        
        //Gyro Reset
        zeroGyro = Robot.controller0.getBButton();
    }

    @Override
    public void getOperatorConfig() {
        lowPickup = Robot.controller1.getPOV() == IOConstants.DPAD_DOWN;
        station = Robot.controller1.getAButton(); // Likely same button as lowReef
        lowReef = station; // Likely same button as station
        midReef = Robot.controller1.getBButton();
        highReef = Robot.controller1.getYButton();
        shelfReef = Robot.controller1.getXButton();
        gripperOpen = Robot.controller1.getLeftTriggerAxis() > IOConstants.TRIGGER_DEADBAND;
        fineControlWrist = applyAxisDeadband(Robot.controller1.getRightX()); // Fine control wrist
        fineControlElbow = applyAxisDeadband(Robot.controller1.getLeftY()); // Fine control elbow
        climberActivate = Robot.controller1.getLeftBumperButton();
        climberDeactivate = Robot.controller1.getRightBumperButton();
        
        fineControlEnable = Robot.controller1.getRightTriggerAxis() > IOConstants.TRIGGER_DEADBAND; // Fine control enable
        climberOnline = Robot.controller1.getPOV() == IOConstants.DPAD_LEFT; // Climber engage
    }
}
