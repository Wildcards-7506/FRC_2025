package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.players.PlayerConfigs;

public class ClimberTestModeCommand extends Command {
    // Holds previous state for toggle functionality

    public ClimberTestModeCommand() {
        addRequirements(Robot.climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time (~20 ms) the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        //If climber mode is engaged, the climber position is set by the operator's joysticks
        Robot.climber.setAnchorVoltage(-12 * PlayerConfigs.moveAnchor);
        Robot.climber.setWinchPosition(Robot.climber.getWinchPosition() + PlayerConfigs.moveWinch * 10, false); // * 7.2 to get 360 deg per sec
        SmartDashboard.putNumber("AnchorPos", Robot.climber.getAnchorPosition());
        SmartDashboard.putNumber("WinchPos", Robot.climber.getWinchPosition());
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