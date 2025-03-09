package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.players.PlayerConfigs;

public class DrivetrainTeleopCommand extends Command {
    double inputRot, yInputSpeed, xInputSpeed;

    public DrivetrainTeleopCommand() {
        addRequirements(Robot.drivetrain);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
    
    // Called every time the scheduler runs (every ~20 ms) while the command is scheduled.
    @Override
    public void execute() {        
        //Reset Gyro
        if(PlayerConfigs.zeroGyro) {
            Robot.drivetrain.zeroHeading();
        }

        // (!)PlayerConfigs.robotRelative instead of fieldRelative because fieldRelative is default
        // When robotRelative is true, the robot will drive relative to the robot's current heading
        // When robotRelative is false, the robot will drive relative to the field's current heading
        if(PlayerConfigs.strafeRight || (PlayerConfigs.robotRelative && PlayerConfigs.snapRight)){
            Robot.drivetrain.drive(PlayerConfigs.fineStrafe, 0, 0, false);
        }
        else if(PlayerConfigs.strafeLeft || (PlayerConfigs.robotRelative && PlayerConfigs.snapLeft)){
            Robot.drivetrain.drive(-PlayerConfigs.fineStrafe, 0, 0, false);
        }
        else {
            // Joystick Inputs
            xInputSpeed = getDriveSpeed(PlayerConfigs.xMovement);
            yInputSpeed = getDriveSpeed(PlayerConfigs.yMovement);
            inputRot = getTurnSpeed(PlayerConfigs.turnMovement);
            Robot.drivetrain.drive(xInputSpeed, yInputSpeed, inputRot, true);
        }
    }

    private double getDriveSpeed(double input) {
        return PlayerConfigs.fineControlToggle ? 
        PlayerConfigs.fineDriveSpeed * input :
        PlayerConfigs.boostToggle ?
        PlayerConfigs.boostDriveSpeed * input :
        PlayerConfigs.fullDriveSpeed * input;
    }

    private double getTurnSpeed(double input) {
        return  PlayerConfigs.fineControlToggle ? 
                    PlayerConfigs.fineTurnSpeed * input : 
                PlayerConfigs.boostToggle ?
                    PlayerConfigs.boostTurnSpeed * input :
                    PlayerConfigs.fullTurnSpeed * input;
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
