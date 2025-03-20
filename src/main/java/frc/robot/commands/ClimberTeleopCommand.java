package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.players.PlayerConfigs;

public class ClimberTeleopCommand extends Command {
    // Holds previous state for toggle functionality
    private boolean prevState = false;

    public ClimberTeleopCommand() {
        addRequirements(Robot.climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time (~20 ms) the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // If driver has engaged climb mode, set a boolean status to indicate.
        // Pressing this button will lock the robot into climb mode to prevent 
        //the crane from moving while trying to climb
        if(PlayerConfigs.climberOnline && !prevState) {
            Robot.climber.onClimberControl = !Robot.climber.onClimberControl;
        }
        prevState = PlayerConfigs.climberOnline;

        SmartDashboard.putBoolean("Climber Toggle", Robot.climber.onClimberControl);

        // DO NOT EXECUTE CLIMBER TELEOP WHILE NOT ON CLIMBER CONTROL
        if(!Robot.climber.onClimberControl) {
            return;
        }

        //If climber mode is engaged, the climber position is set by the operator's joysticks
        Robot.climber.setAnchorVoltage(12 * PlayerConfigs.moveAnchor);
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