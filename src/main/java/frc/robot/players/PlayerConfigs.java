package frc.robot.players;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.IOConstants;

/**
 * This class is used to store the configurations for the drivers and operators.
 * Values are set in the getDriverConfig and getOperatorConfig methods.
 * IOConstants deadbands are applied to the joystick and trigger values by default. 
 */
public abstract class PlayerConfigs {
    // Constants
    public static double fullTurnSpeed;
    public static double fullDriveSpeed;
    public static double boostTurnSpeed;
    public static double boostDriveSpeed;
    public static double fineTurnSpeed;
    public static double fineDriveSpeed;
    
    // Drivetrain
    public static double xMovement;
    public static double yMovement;
    public static double turnMovement;
    public static boolean fullControlToggle;
    public static boolean boostToggle;

    //Slew Rate Limiters
    public static SlewRateLimiter xLimiter = new SlewRateLimiter(1);
    public static SlewRateLimiter yLimiter = new SlewRateLimiter(1);

    //Operator Drivetrain control
    public static double fineStrafe;
    public static boolean strafeRight;
    public static boolean strafeLeft;

    public static boolean zeroGyro;
    // robotRelative instead of fieldRelative because fieldRelative is default
    // When robotRelative is true, the robot will drive relative to the robot's current heading
    // When robotRelative is false, the robot will drive relative to the field's current heading
    public static boolean robotRelative;

    // Arm
    public static boolean stationPickup;
    public static boolean lowPickup;
    public static boolean shelfReef;
    public static boolean lowReef; // Likely same button as lowReef
    public static boolean midReef;
    public static boolean highReef;
    public static boolean algaeHigh;
    public static boolean algaeLow;
    public static boolean suckerIntake;
    public static boolean suckerEject;
    public static double fineControlWrist;
    public static double fineControlElbow;
    // public static double fineControlExtender;
    // public static boolean climberActivate; // Original purpose was to have setpoints that this steps through
    // public static boolean climberDeactivate;
    
    public static boolean fineControlCraneEnable; // Fine control enable
    public static boolean fineControlClimberEnable; // Fine control enable
    public static boolean climberOnline; // Climber toggle is specifically controlled by driver (not operator)

    // ClimbersfineControlRotator
    public static double fineControlRotator;
    public static double moveClimber;

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
     * This method updates the controller values for the driver.
     */
    public void getDriverConfig() {}

    /**
     * This method updates the controller values for the operator.
     */
    public void getOperatorConfig() {}
}