package frc.robot.players.drivers;

import frc.robot.Robot;
import frc.robot.players.PlayerConfigs;

public class Ricardo extends PlayerConfigs {
    public void getDriverConfig() {
        //Constants
        PlayerConfigs.fullTurnSpeed = 0.40;
        PlayerConfigs.fullDriveSpeed = 0.30;
        PlayerConfigs.setupTurnSpeed = 1;
        PlayerConfigs.setupDriveSpeed = 1;
        PlayerConfigs.fineTurnSpeed = 0.1;
        PlayerConfigs.fineDriveSpeed = 0.1;

        //Driving and rotation
        PlayerConfigs.xMovement = -Robot.controller0.getLeftX();
        PlayerConfigs.yMovement = Robot.controller0.getLeftY();
        PlayerConfigs.turnMovement = -Robot.controller0.getRightX();
        PlayerConfigs.setupControlToggle = Robot.controller0.getRightTriggerAxis() > 0.2;
        PlayerConfigs.fineControlToggle = Robot.controller0.getLeftTriggerAxis() > 0.2;
        PlayerConfigs.snapUp = Robot.controller0.getPOV() == 0;
        PlayerConfigs.snapRight = Robot.controller0.getPOV() == 90;
        PlayerConfigs.snapDown = Robot.controller0.getPOV() == 180;
        PlayerConfigs.snapLeft = Robot.controller0.getPOV() == 270;
        PlayerConfigs.robotRelative = Robot.controller0.getRightBumperButton();

        //Scoring
        // PlayerConfigs.fire = Robot.controller0.getRightBumper();
        
        //Gyro Reset
        PlayerConfigs.zeroGyro = Robot.controller0.getAButton();
    } 

    public void getOperatorConfig() {
        //Intake
        // PlayerConfigs.ground = Robot.controller1.getPOV() == 0;
        // PlayerConfigs.amp = Robot.controller1.getPOV() == 90;
        // PlayerConfigs.stow = Robot.controller1.getPOV() == 180;
        
        // Grabbing and rejecting objects
        // PlayerConfigs.intake = Robot.controller1.getXButton();
        // PlayerConfigs.reject = Robot.controller1.getBButton();

        //Intake Fine Control
        PlayerConfigs.fineControlEnable = Robot.controller1.getStartButton();
        PlayerConfigs.fineControlElbow = Robot.controller1.getLeftY();
        PlayerConfigs.fineControlWrist = Robot.controller1.getRightY();

        //Shooter Spin up
        // PlayerConfigs.armScoringMechanism = Robot.controller1.getAButton();
        // PlayerConfigs.shooterActive = Robot.controller1.getBackButton();
        
        //Climbers
        // PlayerConfigs.climberEngage = Robot.controller1.getRightTriggerAxis();
    }
}
