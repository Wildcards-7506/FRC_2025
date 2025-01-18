package frc.robot.players;

public abstract class PlayerConfigs {
    // Constants
    public static double fullTurnSpeed;
    public static double fullDriveSpeed;
    public static double setupTurnSpeed;
    public static double setupDriveSpeed;
    public static double fineTurnSpeed;
    public static double fineDriveSpeed;
    
    // Drivetrain
    public static double xMovement;
    public static double yMovement;
    public static double turnMovement;
    public static boolean fineControlToggle;
    public static boolean setupControlToggle;
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
    public static boolean shelfReef;
    public static boolean highReef;
    public static boolean midReef;
    public static boolean lowReef; // Likely same button as station
    public static boolean station; // Likely same button as lowReef
    public static boolean fineControlEnable; // Fine control enable
    public static double fineControlElbow; // Fine control elbow
    public static double fineControlWrist; // Fine control wrist
    
    // Shooter
    // public static boolean armScoringMechanism;
    // public static boolean shooterActive;
    // public static boolean fire;
    // public static boolean reject;
    // public static boolean intake;

    // Climbers
    // public static double climberEngage;

    public void getDriverConfig(){}

    public void getOperatorConfig(){}
}