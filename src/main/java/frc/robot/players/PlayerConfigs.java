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
    public static boolean fcToggle; // Fine control toggle
    public static boolean scToggle; // Setup control toggle
    public static boolean snapUp;
    public static boolean snapRight;
    public static boolean snapDown;
    public static boolean snapLeft;
    public static boolean zeroGyro;

    // Arm
    public static boolean shelfReef;
    public static boolean highReef;
    public static boolean midReef;
    public static boolean lowReef; // Likely same button as station
    public static boolean station; // Likely same button as lowReef
    public static boolean fcEnable; // Fine control enable
    public static double fcElbow; // Fine control elbow
    public static double fcWrist; // Fine control wrist
    
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