package frc.robot.players.drivers;

import frc.robot.Robot;
import frc.robot.players.PlayerConfigs;

public class Ricardo extends PlayerConfigs {
    @Override
    public void getDriverConfig() {
        //Constants
        fullTurnSpeed = 0.40;
        fullDriveSpeed = 0.30;
        setupTurnSpeed = 1;
        setupDriveSpeed = 1;
        fineTurnSpeed = 0.1;
        fineDriveSpeed = 0.1;
        
        //Driving and rotation
        // TODO: Update these to match the actual controls
        xMovement = -Robot.controller0.getLeftX();
        yMovement = Robot.controller0.getLeftY();
        turnMovement = -Robot.controller0.getRightX();
        setupControlToggle = Robot.controller0.getRightTriggerAxis() > 0.2;
        fineControlToggle = Robot.controller0.getLeftTriggerAxis() > 0.2;
        snapUp = Robot.controller0.getPOV() == 0;
        snapRight = Robot.controller0.getPOV() == 90;
        snapDown = Robot.controller0.getPOV() == 180;
        snapLeft = Robot.controller0.getPOV() == 270;
        robotRelative = Robot.controller0.getRightBumperButton();

        //Scoring
        // fire = Robot.controller0.getRightBumper();
        
        //Gyro Reset
        zeroGyro = Robot.controller0.getAButton();
    } 

    @Override
    public void getOperatorConfig() {
        //Intake
        // ground = Robot.controller1.getPOV() == 0;
        // amp = Robot.controller1.getPOV() == 90;
        // stow = Robot.controller1.getPOV() == 180;
        
        // Grabbing and rejecting objects
        // intake = Robot.controller1.getXButton();
        // reject = Robot.controller1.getBButton();

        //Intake Fine Control
        fineControlEnable = Robot.controller1.getStartButton();
        fineControlElbow = Robot.controller1.getLeftY();
        fineControlWrist = Robot.controller1.getRightY();

        //Shooter Spin up
        // armScoringMechanism = Robot.controller1.getAButton();
        // shooterActive = Robot.controller1.getBackButton();
        
        //Climbers
        // climberEngage = Robot.controller1.getRightTriggerAxis();
    }
}
