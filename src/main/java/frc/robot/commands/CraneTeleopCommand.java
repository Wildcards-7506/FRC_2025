package frc.robot.commands;

import com.pathplanner.lib.pathfinding.LocalADStar.GridPosition;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.players.PlayerConfigs;

public class CraneTeleopCommand extends Command {
    public CraneTeleopCommand() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.crane);
        // Configure additional PID settings here
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // if(PlayerConfigs.fineControlEnable) {
        //     // Robot.crane.setWristPosition(....);
        // }

        if(PlayerConfigs.gripperOpen)
            Robot.crane.setGripperPosition(-90);
        else
            Robot.crane.setGripperPosition(0);

        if(PlayerConfigs.lowPickup) {
            Robot.crane.setWristPosition(0);
        } else if(PlayerConfigs.midReef) {
            Robot.crane.setWristPosition(0);
        } else if(PlayerConfigs.highReef) {
            Robot.crane.setWristPosition(0);
        } else if(PlayerConfigs.shelfReef) {
            Robot.crane.setWristPosition(0);
        } else if(PlayerConfigs.stationOrLowReef) {
            Robot.crane.setWristPosition(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}