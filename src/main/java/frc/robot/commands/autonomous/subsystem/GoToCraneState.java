package frc.robot.commands.autonomous.subsystem;

import frc.robot.Robot;
import frc.robot.Constants.CraneConstants;
import frc.robot.Constants.CraneState;
import frc.robot.utils.Logger;
import edu.wpi.first.wpilibj2.command.Command;

public class GoToCraneState extends Command {
    private CraneState state;
    private double errorMargin;

    /** 
     * Creates a new move to Crane state Command.
     * 
     * @param errorMargin Degree error margin for the command to finish.
    */
    public GoToCraneState(CraneState state, double errorMargin) {
        this.state = state;
        this.errorMargin = errorMargin;
    }
    
    /**
     * Creates a new move to Crane state Command. Default error margin is 1.0 degrees.
     * 
     * @param state Crane state to move to.
     * */
    public GoToCraneState(CraneState state) {
        this(state, CraneConstants.kDefaultErrorMargin);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Logger.info("CRANE", "Crane moving to " + state.toString());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(Robot.skipNonPath) return;
        Robot.crane.goToCraneState(state);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Logger.info("CRANE", "Crane in position at " + state.toString());
    }

    @Override
    public boolean isFinished() {
        // Return true when the robot is at the crane shelf
        return Robot.skipNonPath || 
            Math.abs(Robot.crane.getElbowPosition() - CraneConstants.kElbowShelf) < errorMargin;
    }
}
