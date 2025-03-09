package frc.robot.commands.drivetrain;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.Command;

public class DrivetrainXCommand extends Command{

    /** Creates a new Auto Pitch Correction Command. */
    public DrivetrainXCommand() {}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {    
        Robot.drivetrain.setX();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}