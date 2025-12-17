package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.CraneCommands;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LED;

public class RobotContainer {
    // Controllers - Note the change to CommandXboxController
    // These controller object allow binding of commands to individual buttons.
    private final CommandXboxController driveController = new CommandXboxController(Constants.IOConstants.DRIVER_CONTROLLER_0);
    private final CommandXboxController opController = new CommandXboxController(Constants.IOConstants.DRIVER_CONTROLLER_1);
    private static SlewRateLimiter driveSlewLimiter = new SlewRateLimiter(1);
    private boolean climbMode = false;

    // Subsystems
    public final Drivetrain drivetrain;
    public final Crane crane;
    public final Climber climber;
    public final LED led;

    //Commands - Note the seperate object from the crane subsystem object.
    public final CraneCommands craneCommands;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    drivetrain = new Drivetrain();
    crane = new Crane();
    craneCommands = new CraneCommands(crane);
    climber = new Climber();
    led = new LED(0,14);

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
    /*
     * This file uses a data object called a lambda to save blocks of code for use at a later time.
     * Anything in the format () -> {code} is saving the code in the brackets for use in a command step.
     * This format allows the engineer to chain steps together to create individual commands and sequences.
     * Users may use methods such as repeatingSequence, either, andThen, or along with to create custom 
     * logic loops and bind them to sontroller buttons.
    */

    // Drivetrain
    // Default command, normal field-relative drive
    drivetrain.setDefaultCommand(
        //runOnce creates a command that runs a lambda once and then exits.
        Commands.runOnce(
            () -> {
                double xSpeed = applyAxisDeadband(driveController.getLeftX()) * Constants.DriveConstants.standardSpeed * (climbMode ? 0.25 : 1.0);
                double ySpeed = applyAxisDeadband(driveController.getLeftY()) * Constants.DriveConstants.standardSpeed * (climbMode ? 0.25 : 1.0);
                drivetrain.drive(
                    driveSlewLimiter.calculate(xSpeed), 
                    driveSlewLimiter.calculate(ySpeed), 
                    applyAxisDeadband(-driveController.getRightX()), 
                    true
                );
            },
            drivetrain
    ));

    /*
    * Controller triggers have a variety of button detection cases such as the
    * press, release, change or pure button status to determine when a command is run.
    * Use the .and method to chain buttons together
    * example: pressing the right bumper to shoot a ball only works if the right trigger is also being
    * held down to spin up the shooter.
    */
    driveController.rightTrigger().whileTrue(
        Commands.runOnce(
            () -> drivetrain.drive(
                driveSlewLimiter.calculate(
                    applyAxisDeadband(driveController.getLeftX())), 
                driveSlewLimiter.calculate(
                    applyAxisDeadband(driveController.getLeftY())), 
                applyAxisDeadband(-driveController.getRightX()), 
                true)
                )
    );

    driveController.leftTrigger().whileTrue(
        Commands.runOnce(
            () -> drivetrain.drive(
                driveSlewLimiter.calculate(
                    applyAxisDeadband(driveController.getLeftX()) * Constants.DriveConstants.fineSpeed), 
                driveSlewLimiter.calculate(
                    applyAxisDeadband(driveController.getLeftY()) * Constants.DriveConstants.fineSpeed), 
                applyAxisDeadband(-driveController.getRightX()), 
                true)
    ));

    driveController.b().onTrue(
        Commands.runOnce(
            () -> drivetrain.zeroHeading()
        )
    );

    driveController.x().onTrue(
        Commands.runOnce(() -> drivetrain.setX())
    );

    opController.povLeft().whileTrue(
        Commands.runOnce(
            () -> drivetrain.drive(-Constants.DriveConstants.microSpeed, 0, 0, false)
        )
    );

    opController.povRight().whileTrue(
        Commands.runOnce(
            () -> drivetrain.drive(Constants.DriveConstants.microSpeed, 0, 0, false)
        )
    );

    // Intake - runEnd runs a command until an end condition is triggered, then runs a second command once.
    // In this case, the second command stops the intake wheels when the button is released.
    opController.leftTrigger().whileTrue(
        Commands.runEnd(() -> crane.spinIntake(6), () -> crane.spinIntake(0))
        .alongWith(Commands.runEnd(() -> led.allianceFlow(), () -> led.solid(0, 0, 255)))
    );
    opController.leftBumper().whileTrue(
        Commands.runEnd(() -> crane.spinIntake(-12), () -> crane.spinIntake(0))
        .alongWith(Commands.runEnd(() -> led.allianceFlow(), () -> led.solid(150, 255, 255)))
    );

    //Crane
    driveController.start().onTrue(
        Commands.runOnce(() -> climbMode = true)
        .andThen(craneCommands.climbPrepCommand)
    );
    opController.rightBumper().onTrue(craneCommands.stationCommand);
    opController.x().onTrue(craneCommands.shelfCommand);
    opController.a().onTrue(craneCommands.lowCommand);
    opController.b().onTrue(craneCommands.midCommand);
    opController.y().onTrue(craneCommands.highCommand);
    opController.start().onTrue(craneCommands.stowCommand);
    opController.povUp().onTrue(craneCommands.algaeHighCommand);
    opController.povDown().onTrue(craneCommands.algaeLowCommand);

    //Climber and Crane Fine Control
    opController.rightTrigger().whileTrue(
        //either runs one of two commands depending on a supplied boolean
        Commands.either(
            Commands.runOnce(() -> {
                climber.setAnchorVoltage(12 * opController.getRightY());
                climber.setWinchPosition(climber.getWinchPosition() - opController.getLeftY() * 15); 
                climber.setPivotVoltage(4);
            }).alongWith(
                //repeating sequence runs a sequence of commands repeatedly until the end condition is met.
                Commands.repeatingSequence(
                    Commands.either(
                        Commands.runOnce(() -> led.solid(60, 255, 255)),
                        Commands.runOnce(() -> led.solid(0, 255, 255)), 
                        () -> climber.getWinchPosition() > 380 && climber.getWinchPosition() < 400
                    ),
                    new WaitCommand(0.5),
                    Commands.runOnce(() -> led.solid(0,0,0)),
                    new WaitCommand(0.5))
            ),
            Commands.runOnce(() -> {
                crane.setBoomRotatorPosition(crane.getBoomPosition() - opController.getLeftY() * 20);
                crane.setWristRotatorPosition(crane.getWristPosition() + opController.getRightY() * 20);
            }),
            () -> climbMode)
    );
    
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
}