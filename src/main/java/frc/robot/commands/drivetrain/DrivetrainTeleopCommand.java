package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.players.PlayerConfigs;

public class DrivetrainTeleopCommand extends Command {
    double inputRot, yInputSpeed, xInputSpeed;

    public DrivetrainTeleopCommand() {
        //Prevents the drivetrain from trying to run conflicting commands at the same time
        addRequirements(Robot.drivetrain);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
    
    // Called every time the scheduler runs (every ~20 ms) while the command is scheduled.
    @Override
    public void execute() {        
        //Reset Gyro if driver is pressing reset button
        if(PlayerConfigs.zeroGyro) {
            Robot.drivetrain.zeroHeading();
        }

        // If operator is attempting to strafe, take control from driver and turn off field relative control
        // Allows the operator to strafe across any surface, regardless of what direction the robot is facing
        if(PlayerConfigs.strafeRight){
            Robot.drivetrain.drive(PlayerConfigs.fineStrafe, 0, 0, false);
        }
        else if(PlayerConfigs.strafeLeft){
            Robot.drivetrain.drive(-PlayerConfigs.fineStrafe, 0, 0, false);
        }
        else {
            // If Operator is not strafing, driver controls movement under field oriented control
            // Up on the joystick moves the robot away from the driver station 
            // no matter what direction the robot is facing.
            xInputSpeed = PlayerConfigs.xLimiter.calculate(getDriveSpeed(PlayerConfigs.xMovement));
            yInputSpeed = PlayerConfigs.yLimiter.calculate(getDriveSpeed(PlayerConfigs.yMovement));
            inputRot = getTurnSpeed(PlayerConfigs.turnMovement);
            Robot.drivetrain.drive(xInputSpeed, yInputSpeed, inputRot, true);
            SmartDashboard.putNumber("XSpeed", xInputSpeed);
            SmartDashboard.putNumber("YSpeed", yInputSpeed);
        }
    }

    //If driver is engaging fine control (slow) mode or boost mode, adjust the drive speed accordingly
    private double getDriveSpeed(double input) {
        return PlayerConfigs.fullControlToggle ? 
        PlayerConfigs.fullDriveSpeed * input :
        PlayerConfigs.boostToggle ?
        PlayerConfigs.boostDriveSpeed * input :
        PlayerConfigs.fineDriveSpeed * input;
    }

    //If driver is engaging fine control (slow) mode or boost mode, adjust the rotational speed accordingly
    private double getTurnSpeed(double input) {
        return  PlayerConfigs.fullControlToggle ? 
                    PlayerConfigs.fullTurnSpeed * input : 
                PlayerConfigs.boostToggle ?
                    PlayerConfigs.boostTurnSpeed * input :
                    PlayerConfigs.fineTurnSpeed * input;
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
    
    // Returns true when the command should end.
    // Since we run this every cycle in teleop, we don't want this command to end 
    // or we wouldn't be able to move.
    @Override
    public boolean isFinished() {
        return false;
    }
}
