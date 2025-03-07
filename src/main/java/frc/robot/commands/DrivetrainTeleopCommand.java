package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
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
    
    // Called every time the scheduler runs (every ~20 ms) while the command is scheduled.
    @Override
    public void execute() {        
        //Reset Gyro
        if(PlayerConfigs.zeroGyro) {
            Robot.drivetrain.zeroHeading();
        }

        // Joystick Inputs
        xInputSpeed = getDriveSpeed(PlayerConfigs.xMovement);
        yInputSpeed = getDriveSpeed(PlayerConfigs.yMovement);
        inputRot = getTurnSpeed(PlayerConfigs.turnMovement);

        // (!)PlayerConfigs.robotRelative instead of fieldRelative because fieldRelative is default
        // When robotRelative is true, the robot will drive relative to the robot's current heading
        // When robotRelative is false, the robot will drive relative to the field's current heading
        // System.out.println(Robot.drivetrain.m_gyro.getAngle());
        if(joystickHasInput())
            // Robot.drivetrain.drive(xInputSpeed, yInputSpeed, inputRot, !PlayerConfigs.robotRelative);
            Robot.drivetrain.drive(xInputSpeed, yInputSpeed, inputRot, true);
        else if(PlayerConfigs.strafeRight || (PlayerConfigs.robotRelative && PlayerConfigs.snapRight)){
            Robot.drivetrain.drive(PlayerConfigs.fineStrafe, 0, 0, false);
        }
        else if(PlayerConfigs.strafeLeft || (PlayerConfigs.robotRelative && PlayerConfigs.snapLeft)){
            Robot.drivetrain.drive(-PlayerConfigs.fineStrafe, 0, 0, false);
        }
        // else if(PlayerConfigs.robotRelative) {} // Do nothing if robot relative is on and no input
        // Snap to angles
        else if(PlayerConfigs.snapUp) Robot.drivetrain.snap(xInputSpeed, yInputSpeed, IOConstants.DPAD_UP);
        else if(PlayerConfigs.snapRight) Robot.drivetrain.snap(xInputSpeed, yInputSpeed, IOConstants.DPAD_RIGHT);
        else if(PlayerConfigs.snapDown) Robot.drivetrain.snap(xInputSpeed, yInputSpeed, IOConstants.DPAD_DOWN);
        else if(PlayerConfigs.snapLeft) Robot.drivetrain.snap(xInputSpeed, yInputSpeed, IOConstants.DPAD_LEFT);

        // Reef snaps
        else if(PlayerConfigs.snapUpRight) Robot.drivetrain.snap(xInputSpeed, yInputSpeed, DriveConstants.SNAP_UP_RIGHT);
        else if(PlayerConfigs.snapDownRight) Robot.drivetrain.snap(xInputSpeed, yInputSpeed, DriveConstants.SNAP_DOWN_RIGHT);
        else if(PlayerConfigs.snapDownLeft) Robot.drivetrain.snap(xInputSpeed, yInputSpeed, DriveConstants.SNAP_DOWN_LEFT);
        else if(PlayerConfigs.snapUpLeft) Robot.drivetrain.snap(xInputSpeed, yInputSpeed, DriveConstants.SNAP_UP_LEFT);
        else {
            Robot.drivetrain.stop();
        }

        //Cancel normal command if auto align is called
        // if(PlayerConfigs.autoAlignLeft) {
        //     new AutoAlign(Robot.drivetrain, true).schedule();
        //     this.cancel();
        // } else if(PlayerConfigs.autoAlignRight){
        //     new AutoAlign(Robot.drivetrain, false).schedule();
        //     this.cancel();
        // }
    }

    private boolean joystickHasInput() {
        return Math.abs(PlayerConfigs.xMovement)    > IOConstants.XY_DEADBAND || 
               Math.abs(PlayerConfigs.yMovement)    > IOConstants.XY_DEADBAND || 
               Math.abs(PlayerConfigs.turnMovement) > IOConstants.XY_DEADBAND;
    }

    private double getDriveSpeed(double input) {
        return PlayerConfigs.fineControlToggle ? 
        PlayerConfigs.fineDriveSpeed * input :
        PlayerConfigs.boostToggle ?
        PlayerConfigs.boostDriveSpeed * input :
        PlayerConfigs.fullDriveSpeed * input;
    }

    private double getTurnSpeed(double input) {
        return PlayerConfigs.fineControlToggle ? 
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
