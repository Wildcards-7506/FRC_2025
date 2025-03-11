// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.crane;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.CraneConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElbowCommand extends Command {
  /** Creates a new SetElbowCommand. */
  double setpoint;
  public SetElbowCommand(double setpoint) {
    this.setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Set LEDs to red to indicate the command is running
    Robot.led.solidSection(0,5,0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.crane.setElbowPosition(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //set LEDs to green to indicate the command has ended and we have hit our setpoint
    Robot.led.solidSection(0,5,60);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // If we are within 8 degrees of the setpoint, end the command.
    // Because of the control system used, the motor will continue to move the elbow 
    // to the setpoint even after the command ends
    return Math.abs(Robot.crane.getElbowPosition() - setpoint) < CraneConstants.rotationMargin;
  }
}
