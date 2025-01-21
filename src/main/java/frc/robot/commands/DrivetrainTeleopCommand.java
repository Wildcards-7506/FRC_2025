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

        // Default is stop, input priority: joystick > snap > stop
        // Where joystick is the final decision maker
        Robot.drivetrain.stop();

        // Snap to angles
        if(PlayerConfigs.snapUp) Robot.drivetrain.snap(IOConstants.DPAD_UP);
        if(PlayerConfigs.snapRight) Robot.drivetrain.snap(IOConstants.DPAD_RIGHT);
        if(PlayerConfigs.snapDown) Robot.drivetrain.snap(IOConstants.DPAD_DOWN);
        if(PlayerConfigs.snapLeft) Robot.drivetrain.snap(IOConstants.DPAD_LEFT);

        // Reef snaps
        if(PlayerConfigs.snapUpRight) Robot.drivetrain.snap(DriveConstants.SNAP_UP_RIGHT);
        if(PlayerConfigs.snapDownRight) Robot.drivetrain.snap(DriveConstants.SNAP_DOWN_RIGHT);
        if(PlayerConfigs.snapDownLeft) Robot.drivetrain.snap(DriveConstants.SNAP_DOWN_LEFT);
        if(PlayerConfigs.snapUpLeft) Robot.drivetrain.snap(DriveConstants.SNAP_UP_LEFT);

        // (!)PlayerConfigs.robotRelative instead of fieldRelative because fieldRelative is default
        // When robotRelative is true, the robot will drive relative to the robot's current heading
        // When robotRelative is false, the robot will drive relative to the field's current heading
        if(joystickHasInput())
            Robot.drivetrain.drive(yInputSpeed, xInputSpeed, inputRot, !PlayerConfigs.robotRelative);
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
