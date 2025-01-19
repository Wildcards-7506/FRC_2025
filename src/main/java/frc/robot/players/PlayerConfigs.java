package frc.robot.players;

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
    public static boolean fineControlToggle;
    public static boolean boostToggle;
    public static boolean snapUp;
    public static boolean snapRight;
    public static boolean snapDown;
    public static boolean snapLeft;
    public static boolean zeroGyro;
    // robotRelative instead of fieldRelative because fieldRelative is default
    // When robotRelative is true, the robot will drive relative to the robot's current heading
    // When robotRelative is false, the robot will drive relative to the field's current heading
    public static boolean robotRelative;

    // Arm
    public static boolean lowPickup;
    public static boolean station; // Likely same button as lowReef
    public static boolean lowReef; // Likely same button as station
    public static boolean midReef;
    public static boolean highReef;
    public static boolean shelfReef;
    public static boolean gripperOpen;
    public static double fineControlWrist; // Fine control wrist
    public static double fineControlElbow; // Fine control elbow
    public static boolean climberActivate;
    public static boolean climberDeactivate;
    
    public static boolean fineControlEnable; // Fine control enable
    public static boolean climberOnline; // Climber engage
    
    // Shooter
    // public static boolean armScoringMechanism;
    // public static boolean shooterActive;
    // public static boolean fire;
    // public static boolean reject;
    // public static boolean intake;

    // Climbers
    // public static double climberEngage;

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