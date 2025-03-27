// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.crane.actions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.players.PlayerConfigs;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FineControlCrane extends Command {
  double filteredWrist = 0.0;
  double wristPos = 0.0;
  double filteredElbow = 0.0;
  double elbowPos = 0.0;
  /** Creates a new FineControlCrane. */
  public FineControlCrane() {
    //Takes control of crane subsystem and prevents other
    //commands from being scheduled over the top of this one
    addRequirements(Robot.crane);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.led.enableStreamer = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*Sets target for the motors to half a degree above or below 
      the current setpoint of the wrist and/or elbow.
      Functionally, this causes the motor to chase a moving setpoint, and the movement of that setpoint
      can be controlled by the operator controller */
    // Robot.crane.setWristPosition(Robot.crane.getWristPosition() + PlayerConfigs.fineControlWrist * 20);
    // Robot.crane.setElbowPosition(Robot.crane.getElbowPosition() + PlayerConfigs.fineControlElbow * 20);

    // Keep wrist setpoint from exploding
    filteredWrist = Robot.crane.wristSetpoint + PlayerConfigs.fineControlWrist * 1;
    wristPos = Robot.crane.getWristPosition();
    filteredWrist = filterSetPoint(filteredWrist, wristPos - 10, wristPos + 10);
    
    // Keep elbow setpoint from exploding
    filteredElbow = Robot.crane.elbowSetpoint + PlayerConfigs.fineControlElbow * 1;
    elbowPos = Robot.crane.getElbowPosition();
    filteredElbow = filterSetPoint(filteredElbow, elbowPos - 10, elbowPos + 10);

    Robot.crane.setWristPosition(filteredWrist);
    Robot.crane.setElbowPosition(filteredElbow);
  }
    
    /**
     * Returns the ceiling if the setpoint is above it, or the hard deck if the setpoint is below it.
     * Otherwise, returns the setpoint.
     * 
     * @param setPoint The desired state of the crane.
     * @param hardDeck The hard deck of the crane.
     */
    private double filterSetPoint(double setPoint, double hardDeck, double ceiling) {
        if(setPoint < hardDeck)
            setPoint = hardDeck;
        if(setPoint > ceiling)
            setPoint = ceiling;
        return setPoint;
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
