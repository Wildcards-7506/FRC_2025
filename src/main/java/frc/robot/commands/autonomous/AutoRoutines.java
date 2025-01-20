// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.commands.autonomous.subsystem.AutoDrivetrainX;

@SuppressWarnings("unused")
public final class AutoRoutines {

  private HashMap<String, Command> eventMap;
  private final AutoBuilder autoBuilder = new AutoBuilder();

  // Autonomous selector on dashboard
  private final SendableChooser<Command> autoChooser;
  private double kAutoStartDelaySeconds;
    
  // Load the RobotConfig from the GUI settings. You should probably
  // store this in your Constants file
  RobotConfig config;
  
  public AutoRoutines() {
    setMarkers();

    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    AutoBuilder.configure(
        Robot.drivetrain::getPose, // Robot pose supplier
        Robot.drivetrain::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        Robot.drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> Robot.drivetrain.driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
              return alliance.get() == DriverStation.Alliance.Red;
            return false;
        },
        Robot.drivetrain // Reference to this subsystem to set requirements
    );

    // Autonomous selector options
    kAutoStartDelaySeconds = SmartDashboard.getNumber("Auto Delay",0.0);
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser",autoChooser);

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) 
    -> Robot.m_field.getObject("path").setPoses(poses));
  }

  private void setMarkers() {
    // NamedCommands.registerCommand("AutoShoot", new AutoShoot());
    // NamedCommands.registerCommand("AutoIntake", new AutoIntake_Trigger(5, false,12,2.4));
    // NamedCommands.registerCommand("AutoShooterSpinup", new AutoShooterSpinUp(ShooterConstants.kLArmedRPM, ShooterConstants.kRArmedRPM));
    // NamedCommands.registerCommand("AutoIntakeStowToGround", new AutoIntakeStowToGround());
    // NamedCommands.registerCommand("AutoIntakeGroundToStow", new AutoIntakeGroundToStow());
    // NamedCommands.registerCommand("AutoIntakeGroundToAmp", new AutoIntakeGroundToAmp());
    // NamedCommands.registerCommand("AutoIntakeAmpToGround", new AutoIntakeAmpToGround());
    // NamedCommands.registerCommand("AutoIntakeStowToAmp", new AutoIntakeStowToAmp());
  }

      /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.sequence(
      Commands.waitSeconds(kAutoStartDelaySeconds),
      autoChooser.getSelected(),
      new AutoDrivetrainX());
  }

  public void resetAutoHeading() {
    Robot.drivetrain.zeroHeading();
  }
  }