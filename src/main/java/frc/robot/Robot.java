// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.autonomous.AutoRoutines;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  /* Declare systems critical to non-robot operation in this file.
   * This file contains instructions for each operational mode:
   * Disabled, Autonomous, Teleoperated, Test
   * Each operational mode is made of an initialize step 
   * (runs once at the beginning of each mode) and a periodic step 
   * (runs continuously while the mode is enabled)
   * Save robot-specific systems in the RobotContainer file. 
   * Adding everything to Robot.java does work, but can make code hard to read with 
   * multiple hundreds of lines in one file.
  */
  
  //Creates our robot in memory on the RoboRIO.
  public static RobotContainer robotContainer = new RobotContainer();
  //Creates a graphical representation of the field in the Shuffleboard app.
  public static Field2d field = new Field2d();
  //Creates an object that can poll the driverstation for alliance color.
  public static Optional<Alliance> teamColor;
  //Creates an object that holds our autonomous routine.
  public AutoRoutines autoMode = new AutoRoutines();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {   
    //Put the graphical field on the shuffleboard app
    SmartDashboard.putData("Field", field); 
  }
  
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    /*Update where the robot is on the field every clock cycle.
      If pose estimation is enabled with a limelight, this will 
      also work when the robot is disabled.
    */
    field.setRobotPose(robotContainer.drivetrain.getPose());
  }

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();

    /*  These lines do the following:
    Poll the driver station for color
    Reset the robot's pose to the starting position of the chosen autonomous routine
    Starts the autonomous routine
    Sets the drivetrain to brake mode so we can't be shoved if not moving
    */
    teamColor = DriverStation.getAlliance();
    autoMode.resetAutoHeading();
    autoMode.getAutonomousCommand().schedule();
    robotContainer.drivetrain.idleSwerve(IdleMode.kBrake);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    /*Every clock cycle in autonomous, continue running the autonomous routine
    and run the led rainbow function*/
    CommandScheduler.getInstance().run();
    robotContainer.led.rainbow();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    teamColor = DriverStation.getAlliance();

    // Default subsystem states
    robotContainer.drivetrain.idleSwerve(IdleMode.kBrake);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    //Set the drivetrain to coast mode so it can be moved by hand
    robotContainer.drivetrain.idleSwerve(IdleMode.kCoast);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    teamColor = DriverStation.getAlliance();
    robotContainer.led.allianceFlow();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    //Disable climber software limits so it can be retracted under motor power
    robotContainer.climber.testModeConfig();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
