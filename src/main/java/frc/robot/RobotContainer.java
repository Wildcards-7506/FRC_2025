package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CraneConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.CraneCommands;
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
    // Controllers - Note the change to CommandXboxController
    // These controller object allow binding of commands to individual buttons.
    public final CommandXboxController controller0 = new CommandXboxController(Constants.IOConstants.DRIVER_CONTROLLER_0);
    public final CommandXboxController controller1 = new CommandXboxController(Constants.IOConstants.DRIVER_CONTROLLER_1);
    private static SlewRateLimiter slewLimiter = new SlewRateLimiter(1);
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
                double xSpeed = applyAxisDeadband(controller0.getLeftX()) * Constants.DriveConstants.standardSpeed * (climbMode ? 0.25 : 1.0);
                double ySpeed = applyAxisDeadband(controller0.getLeftY()) * Constants.DriveConstants.standardSpeed * (climbMode ? 0.25 : 1.0);
                drivetrain.drive(
                slewLimiter.calculate(xSpeed), 
                slewLimiter.calculate(ySpeed), 
                applyAxisDeadband(-controller0.getRightX()), 
                true);
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
    controller0.rightTrigger().whileTrue(
        Commands.runOnce(
            () -> drivetrain.drive(
                slewLimiter.calculate(
                    applyAxisDeadband(controller0.getLeftX())), 
                slewLimiter.calculate(
                    applyAxisDeadband(controller0.getLeftX())), 
                applyAxisDeadband(-controller0.getRightX()), 
                true)
                )
    );

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

    // Intake - runEnd runs a command until an end condition is triggered, then runs a second command once.
    // In this case, the second command stops the intake wheels when the button is released.
    controller1.leftTrigger().whileTrue(Commands.runEnd(() -> crane.spinSucker(CraneConstants.kSuckerIntake), () -> crane.spinSucker(0)));
    controller1.leftBumper().whileTrue(Commands.runEnd(() -> crane.spinSucker(CraneConstants.kSuckerEject), () -> crane.spinSucker(0)));

    //Crane
    controller0.start().onTrue(
        Commands.runOnce(() -> climbMode = true)
        .andThen(craneCommands.climbPrepCommand)
    );
    controller1.rightBumper().onTrue(craneCommands.stationCommand);
    controller1.x().onTrue(craneCommands.shelfCommand);
    controller1.a().onTrue(craneCommands.lowCommand);
    controller1.b().onTrue(craneCommands.midCommand);
    controller1.y().onTrue(craneCommands.highCommand);
    controller1.start().onTrue(craneCommands.stowCommand);
    controller1.povUp().onTrue(craneCommands.algaeHighCommand);
    controller1.povDown().onTrue(craneCommands.algaeLowCommand);

    //Climber and Crane Fine Control
    controller1.rightTrigger().whileTrue(
        //either runs one of two commands depending on a supplied boolean
        Commands.either(
            Commands.runOnce(() -> {
                climber.setAnchorVoltage(12 * controller1.getRightY());
                climber.setWinchPosition(climber.getWinchPosition() - controller1.getLeftY() * 15); 
                climber.setTensionerVoltage(4);
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
                crane.setElbowPosition(crane.getElbowPosition() - controller1.getLeftY() * 20);
                crane.setWristPosition(crane.getWristPosition() + controller1.getRightY() * 20);
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