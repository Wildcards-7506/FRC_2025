package frc.robot.commands.autonomous.subsystem;

import frc.robot.Robot;
import frc.robot.utils.Logger;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoDrivetrainX extends Command{

    /** Creates a new Auto Pitch Correction Command. */
    public AutoDrivetrainX() {}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Logger.info("DRVEX","X Started");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {    
        if(!Robot.skipNonPath) {Robot.drivetrain.setX();}
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Logger.info("DRVEX","Robot Anchored"); 
        Robot.drivetrain.setX();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}