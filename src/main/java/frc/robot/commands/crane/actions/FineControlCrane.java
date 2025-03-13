// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.crane.actions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.players.PlayerConfigs;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FineControlCrane extends Command {
  /** Creates a new FineControlCrane. */
  public FineControlCrane() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.crane);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.crane.setWristPosition(Robot.crane.wristSetpoint + PlayerConfigs.fineControlWrist * 1);
    Robot.crane.setElbowPosition(Robot.crane.elbowSetpoint + PlayerConfigs.fineControlElbow * 1);
    Robot.led.solidSection(0,8,(int)Math.round(30*PlayerConfigs.fineControlWrist + 30));
    Robot.led.solidSection(8,14,(int)Math.round(30*PlayerConfigs.fineControlElbow + 30));
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
