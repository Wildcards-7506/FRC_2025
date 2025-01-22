package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.players.PlayerConfigs;

public class CraneTeleopCommand extends Command {
    public CraneTeleopCommand() {
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID settings here
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // if(PlayerConfigs.fineControlEnable) {
        //     // Robot.crane.setWristPosition(....);
        // }

        if(PlayerConfigs.lowPickup) {
            Robot.crane.setWristPosition(0);
        } else {
            Robot.crane.setWristPosition(90);
        }
        // Implement the teleop control for the claw here
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