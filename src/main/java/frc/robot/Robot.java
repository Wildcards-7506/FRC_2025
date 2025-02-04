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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ClimberTeleopCommand;
import frc.robot.commands.CraneTeleopCommand;
import frc.robot.commands.DrivetrainTeleopCommand;
import frc.robot.commands.autonomous.AutoRoutines;
import frc.robot.players.PlayerConfigs;
import frc.robot.players.drivers.Ricardo;
import frc.robot.players.drivers.TestController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Logger;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  // Modes & people
  private AutoRoutines autoMode;
  public PlayerConfigs driver;
  public PlayerConfigs operator;
  public static boolean skipNonPath;

  // Field Information
  public static Optional<Alliance> teamColor;
  public final static Field2d m_field = new Field2d();

  // Controllers
  public static final XboxController controller0 = new XboxController(Constants.IOConstants.DRIVER_CONTROLLER_0);
  public static final XboxController controller1 = new XboxController(Constants.IOConstants.DRIVER_CONTROLLER_1);

  // Drivers
  public static SendableChooser<PlayerConfigs> driver_chooser = new SendableChooser<>();
  public static SendableChooser<PlayerConfigs> operator_chooser = new SendableChooser<>();
  public static PlayerConfigs ricardo = new Ricardo();
  public static PlayerConfigs test = new TestController();

  // Subsystems
  public final static Drivetrain drivetrain = new Drivetrain();
  public final static Crane crane = new Crane();
  public final static Climber climber = new Climber();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    Logger.info("SYSTEM","Robot Started");
    
    //Auto Chooser
    autoMode = new AutoRoutines();

    // // Driver choosers
    driver_chooser.setDefaultOption("Ricardo", ricardo);
    driver_chooser.addOption("Test", test);       

    // Operator choosers
    operator_chooser.setDefaultOption("Test", test);
    operator_chooser.addOption("Ricardo", ricardo);

    // Put the choosers on the dashboard
    SmartDashboard.putData("Driver",driver_chooser);
    SmartDashboard.putData("Operator",operator_chooser);
    SmartDashboard.putBoolean("Skip Non-Path Commands", false);
    SmartDashboard.putData(m_field);
  }
  
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    Logger.info("SYSTEM","Autonomous Program Started");
    CommandScheduler.getInstance().cancelAll();

    // Set robot state
    teamColor = DriverStation.getAlliance();
    autoMode.resetAutoHeading();
    autoMode.getAutonomousCommand().schedule();
    drivetrain.idleSwerve(IdleMode.kBrake);
    skipNonPath = SmartDashboard.getBoolean("Skip Non-Path Commands", false);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    Logger.info("SYSTEM","Teleop Started");
    CommandScheduler.getInstance().cancelAll();

    // Get the selected drivers
    driver = driver_chooser.getSelected();
    operator = operator_chooser.getSelected();
    teamColor = DriverStation.getAlliance();

    // Subsystem default commands
    drivetrain.setDefaultCommand(new DrivetrainTeleopCommand());
    crane.setDefaultCommand(new CraneTeleopCommand());

    // Default subsystem states
    drivetrain.idleSwerve(IdleMode.kBrake);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    driver.getDriverConfig();
    operator.getOperatorConfig();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    Logger.info("SYSTEM", "Robot Disabled");
    Logger.flush();
    CommandScheduler.getInstance().cancelAll();
    drivetrain.idleSwerve(IdleMode.kCoast);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    Logger.info("SYSTEM","Test Program Started");
    CommandScheduler.getInstance().cancelAll();
    driver = driver_chooser.getSelected();
    operator = operator_chooser.getSelected();
    // crane.setDefaultCommand(new CraneTeleopCommand());
    climber.setDefaultCommand(new ClimberTeleopCommand());
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    CommandScheduler.getInstance().run();
    operator.getOperatorConfig();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
