package frc.robot.commands.autonomous.commands;

import frc.robot.Robot;
import frc.robot.Constants.CraneConstants;
import frc.robot.utils.Logger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSpinSucker extends Command {
    private final Timer timer = new Timer();
    private double volts;
    private double duration;

    /**
     * Creates a new AutoSpinSucker command. Stops the sucker after a given duration.
     * 
     * @param duration Time in seconds to run the command.
     * @param volts Voltage to run the sucker.
     */
    public AutoSpinSucker(double duration, double volts) {
        this.duration = duration;
        this.volts = volts;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(volts == CraneConstants.kSuckerIntake) Logger.info("SUCKER", "Sucker state is Suck");
        else if(volts == CraneConstants.kSuckerEject) Logger.info("SUCKER", "Sucker state is EJECT");
        else Logger.info("SUCKER", "Sucker state is STOP");
        timer.reset();
        timer.start();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(Robot.skipNonPath) return;
        Robot.crane.spinSucker(volts);
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Logger.info("SUCKER", "Sucker is done");
        Robot.crane.holdSucker();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        // Timeout in seconds
        return timer.get() > duration;
    }
}
