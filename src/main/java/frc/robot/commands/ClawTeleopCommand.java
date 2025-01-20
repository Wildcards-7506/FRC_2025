package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class ClawTeleopCommand extends Command {
    public ClawTeleopCommand() {
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