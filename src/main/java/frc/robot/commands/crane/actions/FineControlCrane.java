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
    //Takes control of crane subsystem and prevents other
    //commands from being scheduled over the top of this one
    addRequirements(Robot.crane);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*Sets target for the motors to half a degree above or below 
      the current setpoint of the wrist and/or elbow.
      Functionally, this causes the motor to chase a moving setpoint, and the movement of that setpoint
      can be controlled by the operator controller */
    Robot.crane.setWristPosition(Robot.crane.wristSetpoint + PlayerConfigs.fineControlWrist * 1);
    Robot.crane.setElbowPosition(Robot.crane.elbowSetpoint + PlayerConfigs.fineControlElbow * 1);

    //Sets LEDs to green/red depending on what direction fine control is moving the wrist or elbow
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
