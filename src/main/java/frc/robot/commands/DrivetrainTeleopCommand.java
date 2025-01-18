package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IOConstants;
import frc.robot.Robot;
import frc.robot.players.PlayerConfigs;

public class DrivetrainTeleopCommand extends Command {
    double inputRot, yInputSpeed, xInputSpeed;
    String payload;
    // private static final DecimalFormat df = new DecimalFormat("0.00");

    public DrivetrainTeleopCommand() {
        addRequirements(Robot.drivetrain);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //Reset Gyro
        if(PlayerConfigs.zeroGyro){
            Robot.drivetrain.zeroHeading();
        }

        //Joystick Inputs
        xInputSpeed = PlayerConfigs.fineControlToggle ? 
            PlayerConfigs.fineDriveSpeed * PlayerConfigs.xMovement :
            PlayerConfigs.setupControlToggle ?
            PlayerConfigs.setupDriveSpeed * PlayerConfigs.xMovement :
            PlayerConfigs.fullDriveSpeed * PlayerConfigs.xMovement;
        yInputSpeed = PlayerConfigs.fineControlToggle ? 
            -PlayerConfigs.fineDriveSpeed * PlayerConfigs.yMovement : 
            PlayerConfigs.setupControlToggle ?
            -PlayerConfigs.setupDriveSpeed * PlayerConfigs.yMovement :
            -PlayerConfigs.fullDriveSpeed * PlayerConfigs.yMovement;
        inputRot = PlayerConfigs.fineControlToggle ? 
            PlayerConfigs.fineTurnSpeed * PlayerConfigs.turnMovement : 
            PlayerConfigs.setupControlToggle ?
            PlayerConfigs.setupTurnSpeed * PlayerConfigs.turnMovement :
            PlayerConfigs.fullTurnSpeed * PlayerConfigs.turnMovement;

        //Snap or align if needed, set drive if joystick inputs available, otherwise X
        if(PlayerConfigs.snapUp){
            double angle = Robot.teamColor.get() == Alliance.Red ? 180 : 0;
            Robot.drivetrain.snap(angle);
        } else if(PlayerConfigs.snapRight) {
            double angle = Robot.teamColor.get() == Alliance.Red ? 90 : -90;
            Robot.drivetrain.snap(angle);
        } else if(PlayerConfigs.snapDown) {
            double angle = Robot.teamColor.get() == Alliance.Red ? 0 : 90;
            Robot.drivetrain.snap(angle);
        } else if(PlayerConfigs.snapLeft){
            double angle = Robot.teamColor.get() == Alliance.Red ? -90 : 90;
            Robot.drivetrain.snap(angle);
        } else if (Math.abs(PlayerConfigs.xMovement) > IOConstants.DEADZONE || 
                   Math.abs(PlayerConfigs.yMovement) > IOConstants.DEADZONE || 
                   Math.abs(PlayerConfigs.turnMovement) > IOConstants.DEADZONE) {
            // (!)PlayerConfigs.robotRelative instead of fieldRelative because fieldRelative is default
            // When robotRelative is true, the robot will drive relative to the robot's current heading
            // When robotRelative is false, the robot will drive relative to the field's current heading
            Robot.drivetrain.drive(yInputSpeed, xInputSpeed, inputRot, !PlayerConfigs.robotRelative);
        } else {
            // Robot.drivetrain.setX();
        }
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
