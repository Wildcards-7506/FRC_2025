// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.CraneConstants;
import frc.robot.commands.ClimberTeleopCommand;
import frc.robot.commands.ClimberTestModeCommand;
import frc.robot.commands.autonomous.AutoRoutines;
import frc.robot.commands.crane.CraneTeleopCommand;
import frc.robot.commands.crane.actions.ClimbPresetCommand;
import frc.robot.commands.crane.actions.FineControlCrane;
import frc.robot.commands.crane.actions.ReefStationCommand;
import frc.robot.commands.crane.actions.StowCommand;
import frc.robot.commands.drivetrain.DrivetrainTeleopCommand;
import frc.robot.players.PlayerConfigs;
import frc.robot.players.drivers.Ricardo;
import frc.robot.players.drivers.Dessie;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LED;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  // Modes & people
  private AutoRoutines autoMode;
  public PlayerConfigs driver = new Ricardo();
  public PlayerConfigs operator = new Dessie();
  public static boolean skipNonPath;

  // Field Information
  public static Optional<Alliance> teamColor;
  public final static Field2d m_field = new Field2d();

  // Controllers
  public static final XboxController controller0 = new XboxController(Constants.IOConstants.DRIVER_CONTROLLER_0);
  public static final XboxController controller1 = new XboxController(Constants.IOConstants.DRIVER_CONTROLLER_1);

  // Subsystems
  public final static Drivetrain drivetrain = new Drivetrain();
  public final static Crane crane = new Crane();
  public final static Climber climber = new Climber();
  public final static LED led = new LED(0,14);

  //Commands
  public final static ClimbPresetCommand climbPresetCommand = new ClimbPresetCommand();
  public final static StowCommand stowCommand = new StowCommand();
  public final static ReefStationCommand stationCommand = new ReefStationCommand(
      CraneConstants.kElbowStation,
      CraneConstants.kExtenderStation,
      CraneConstants.kWristStation,
    160);
  public final static ReefStationCommand shelfCommand = new ReefStationCommand(
      CraneConstants.kElbowShelf,
      CraneConstants.kExtenderShelf,
      CraneConstants.kWristShelf,
      15);
  public final static ReefStationCommand lowCommand = new ReefStationCommand(
    CraneConstants.kElbowLow,
    CraneConstants.kExtenderLow,
    CraneConstants.kWristLow,
    120);
  public final static ReefStationCommand midCommand = new ReefStationCommand(
    CraneConstants.kElbowMid,
    CraneConstants.kExtenderMid,
    CraneConstants.kWristMid,
    150);
  public final static ReefStationCommand highCommand = new ReefStationCommand(
    CraneConstants.kElbowHigh,
    CraneConstants.kExtenderHigh,
    CraneConstants.kWristHigh,
    0);
  public final static ReefStationCommand algaeHighCommand = new ReefStationCommand(
    CraneConstants.kElbowAlgaeHigh,
    CraneConstants.kExtenderAlgaeHigh,
    CraneConstants.kWristAlgaeHigh,
    90);
  public final static ReefStationCommand algaeLowCommand = new ReefStationCommand(
    CraneConstants.kElbowAlgaeLow,
    CraneConstants.kExtenderAlgaeLow,
    CraneConstants.kWristAlgaeLow,
    70);
  public final static FineControlCrane fineControlCrane = new FineControlCrane();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {    
    //Auto Chooser
    autoMode = new AutoRoutines();
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
  }

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
    CommandScheduler.getInstance().cancelAll();

    // Set robot state
    teamColor = DriverStation.getAlliance();
    autoMode.resetAutoHeading();
    autoMode.getAutonomousCommand().schedule();
    drivetrain.idleSwerve(IdleMode.kBrake);
    led.enableStreamer = true;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
    led.rainbow();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();

    // Get the selected drivers
    teamColor = DriverStation.getAlliance();
    led.enableStreamer = true;

    // Subsystem default commands
    drivetrain.setDefaultCommand(new DrivetrainTeleopCommand());
    new CraneTeleopCommand().schedule();;
    climber.setDefaultCommand(new ClimberTeleopCommand());

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
    CommandScheduler.getInstance().cancelAll();
    drivetrain.idleSwerve(IdleMode.kCoast);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    led.allianceFlow();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    Robot.climber.testModeConfig();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    CommandScheduler.getInstance().run();
    driver.getDriverConfig();
    operator.getOperatorConfig();
    new ClimberTestModeCommand().schedule();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
