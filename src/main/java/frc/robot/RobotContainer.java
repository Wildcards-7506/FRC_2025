package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CraneConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.autonomous.AutoRoutines;
import frc.robot.commands.crane.actions.ClimbPresetCommand;
import frc.robot.commands.crane.actions.FineControlCrane;
import frc.robot.commands.crane.actions.StowCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LED;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Modes & people
    private AutoRoutines autoMode;

    // Controllers
    public final CommandXboxController controller0 = new CommandXboxController(Constants.IOConstants.DRIVER_CONTROLLER_0);
    public final CommandXboxController controller1 = new CommandXboxController(Constants.IOConstants.DRIVER_CONTROLLER_1);
    private static SlewRateLimiter slewLimiter = new SlewRateLimiter(1);
    private boolean climbMode = false;

    // Subsystems
    public final Drivetrain drivetrain;
    public final Crane crane;
    public final Climber climber;
    public final LED led;

    //Commands
    public final ClimbPresetCommand climbPresetCommand;
    public final StowCommand stowCommand;
    public final Command stationCommand;
    public final Command shelfCommand;
    public final Command lowCommand;
    public final Command midCommand;
    public final Command highCommand;
    public final Command algaeHighCommand;
    public final Command algaeLowCommand;
    public final FineControlCrane fineControlCrane;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    //Auto Chooser
    autoMode = new AutoRoutines();

    drivetrain = new Drivetrain();
    crane = new Crane();
    climber = new Climber();
    led = new LED(0,14);

    climbPresetCommand = new ClimbPresetCommand();

    stowCommand = new StowCommand();

    stationCommand = new crane.ReefStationCommand(
      CraneConstants.kElbowStation,
      CraneConstants.kExtenderStation,
      CraneConstants.kWristStation);

    shelfCommand = new ReefStationCommand(
      CraneConstants.kElbowShelf,
      CraneConstants.kExtenderShelf,
      CraneConstants.kWristShelf,
      15);

    lowCommand = new ReefStationCommand(
    CraneConstants.kElbowLow,
    CraneConstants.kExtenderLow,
    CraneConstants.kWristLow,
    120);

    midCommand = new ReefStationCommand(
    CraneConstants.kElbowMid,
    CraneConstants.kExtenderMid,
    CraneConstants.kWristMid,
    150);

    highCommand = new ReefStationCommand(
    CraneConstants.kElbowHigh,
    CraneConstants.kExtenderHigh,
    CraneConstants.kWristHigh,
    0);

    algaeHighCommand = new ReefStationCommand(
    CraneConstants.kElbowAlgaeHigh,
    CraneConstants.kExtenderAlgaeHigh,
    CraneConstants.kWristAlgaeHigh,
    90);

    algaeLowCommand = new ReefStationCommand(
    CraneConstants.kElbowAlgaeLow,
    CraneConstants.kExtenderAlgaeLow,
    CraneConstants.kWristAlgaeLow,
    70);

    fineControlCrane = new FineControlCrane();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drivetrain
    // Default command, normal field-relative drive
    drivetrain.setDefaultCommand(
        Commands.runOnce(
            () -> {
                double xSpeed = applyAxisDeadband(controller0.getLeftX()) * Constants.DriveConstants.standardSpeed * (climbMode ? 0.25 : 1.0);
                double ySpeed = applyAxisDeadband(controller0.getLeftY()) * Constants.DriveConstants.standardSpeed * (climbMode ? 0.25 : 1.0);
                drivetrain.drive(
                slewLimiter.calculate(xSpeed), 
                slewLimiter.calculate(ySpeed), 
                applyAxisDeadband(-controller0.getRightX()), 
                true);
            }
    ));

    controller0.rightTrigger().whileTrue(
        Commands.runOnce(
            () -> drivetrain.drive(
                slewLimiter.calculate(
                    applyAxisDeadband(controller0.getLeftX())), 
                slewLimiter.calculate(
                    applyAxisDeadband(controller0.getLeftX())), 
                applyAxisDeadband(-controller0.getRightX()), 
                true)
    ));

    controller0.leftTrigger().whileTrue(
        Commands.runOnce(
            () -> drivetrain.drive(
                slewLimiter.calculate(
                    applyAxisDeadband(controller0.getLeftX()) * Constants.DriveConstants.fineSpeed), 
                slewLimiter.calculate(
                    applyAxisDeadband(controller0.getLeftX()) * Constants.DriveConstants.fineSpeed), 
                applyAxisDeadband(-controller0.getRightX()), 
                true)
    ));

    controller1.povLeft().whileTrue(
        Commands.runOnce(
            () -> drivetrain.drive(-Constants.DriveConstants.microSpeed, 0, 0, false)
        )
    );

    controller1.povRight().whileTrue(
        Commands.runOnce(
            () -> drivetrain.drive(Constants.DriveConstants.microSpeed, 0, 0, false)
        )
    );

    controller0.b().onTrue(
        Commands.runOnce(
            () -> drivetrain.zeroHeading()
        )
    );

    controller0.x().onTrue(
        Commands.runOnce(() -> drivetrain.setX())
    );

    //Crane
    controller0.start().onTrue(
        Commands.runOnce(() -> climbMode = true)
        .andThen(climbPresetCommand)
    );

    stationPickup = Robot.controller1.getRightBumperButton();
        shelfReef = Robot.controller1.getXButton();
        lowReef = Robot.controller1.getAButton();
        midReef = Robot.controller1.getBButton();
        highReef = Robot.controller1.getYButton();
        algaeHigh = Robot.controller1.getPOV() == IOConstants.DPAD_UP;
        algaeLow = Robot.controller1.getPOV() == IOConstants.DPAD_DOWN;
        suckerIntake = Robot.controller1.getLeftTriggerAxis() > IOConstants.TRIGGER_DEADBAND;
        suckerEject = Robot.controller1.getLeftBumperButton();

  }

      /**
     * This helper method is used to get the joystick value after deadbanding.
     * 
     * @param axis The joystick axis to apply the deadband to.
     * @return A double representing the new axis value. 0.0 if old axis value <= {@code IOConstants.XY_DEADBAND}.
     */
    public double applyAxisDeadband(double axis) {
        return Math.abs(axis) > IOConstants.XY_DEADBAND ? axis : 0.0;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    // public Command getAutonomousCommand() {
    //     return autoRoutineBuilder.build();
    // }
}