package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.players.PlayerConfigs;

public class ClimberTeleopCommand extends Command {

    public ClimberTeleopCommand() {
        addRequirements(Robot.climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time (~20 ms) the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(PlayerConfigs.fineControlClimberEnable) { // fine control
            Robot.climber.setRotatorPosition(Robot.climber.rotatorSetpoint + PlayerConfigs.fineControlRotator * 0.1);
            Robot.climber.setAnchorPosition(Robot.climber.anchorSetpoint + PlayerConfigs.fineControlAnchor * 0.01);
        }

        SmartDashboard.putNumber("Climber State", Robot.climber.climberState);
        SmartDashboard.putBoolean("Climber FC", PlayerConfigs.fineControlClimberEnable);
        SmartDashboard.putNumber("FC Rotator", PlayerConfigs.fineControlRotator);
        SmartDashboard.putNumber("FC Anchor", PlayerConfigs.fineControlAnchor);
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