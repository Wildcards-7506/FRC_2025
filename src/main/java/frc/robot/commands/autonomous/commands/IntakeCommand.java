package frc.robot.commands.autonomous.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    private final Timer timer = new Timer();
    private double volts;
    private double duration;

    /**
     * Creates a new AutoSpinSucker command. Stops the sucker after a given duration.
     * 
     * @param duration Time in seconds to run the command.
     * @param volts Voltage to run the sucker.
     */
    public IntakeCommand(double duration, double volts) {
        this.duration = duration;
        this.volts = volts;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Robot.crane.spinSucker(volts);
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.crane.spinSucker(0);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        // Timeout in seconds
        return timer.get() > duration;
    }
}
