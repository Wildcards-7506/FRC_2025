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
import frc.robot.Constants.CraneConstants;

@SuppressWarnings("unused")
public final class AutoRoutines {

  private HashMap<String, Command> eventMap;
  private final AutoBuilder autoBuilder = new AutoBuilder();

  // Autonomous selector on dashboard
  private final SendableChooser<Command> autoChooser;
    
  // Load the RobotConfig from the GUI settings.
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
      Robot.robotContainer.drivetrain::getPose, // Robot pose supplier
      Robot.robotContainer.drivetrain::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      Robot.robotContainer.drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> Robot.robotContainer.drivetrain.driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
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
      Robot.robotContainer.drivetrain // Reference to this subsystem to set requirements
    );
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser",autoChooser);

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) 
    -> Robot.field.getObject("path").setPoses(poses));
  }

  private void setMarkers() {
    //Registers commands to run in autonomous. The Pathplanner application can take these
    //pre-defined commands and place them at specific points while moving.
    NamedCommands.registerCommand("AutoCraneStation", Robot.robotContainer.craneCommands.stationCommand);
    NamedCommands.registerCommand("AutoCraneShelf", Robot.robotContainer.craneCommands.shelfCommand);
    NamedCommands.registerCommand("AutoCraneStow", Robot.robotContainer.craneCommands.stowCommand);
    NamedCommands.registerCommand("AutoEject", Commands.run(() -> Robot.robotContainer.crane.spinIntake(-12)).withTimeout(0.25));
    NamedCommands.registerCommand("AutoIntake", Commands.run(() -> Robot.robotContainer.crane.spinIntake(6)).withTimeout(1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void resetAutoHeading() {
    Robot.robotContainer.drivetrain.zeroHeading();
  }
}