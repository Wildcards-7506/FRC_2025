
//This will be the control for the claw on the robot
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.CraneConstants;

public class Crane extends SubsystemBase {
    // Crane vars
    public int craneState = 0; // 0 is stow/default/starting configuration

    // Gripper
    private final SparkMax gripperMotor;
    private final SparkMaxConfig gripperConfig;
    public final SparkClosedLoopController gripperPID;
    public double gripperSetpoint;
    // public boolean gripState = true; // gripToggle = true: means gripping coral, false: means open

    // Wrist
    private final SparkMax wristMotor;
    private final SparkMaxConfig wristConfig;
    public final SparkClosedLoopController wristPID;
    public double wristSetpoint;
    // public boolean wristScoreState = false;

    // Arm: Elbow and Extender

    // Elbow
    private final SparkMax elbowMotor;
    private final SparkMaxConfig elbowConfig;
    public final SparkClosedLoopController elbowPID;
    public double elbowSetpoint;
    
    //Extender
    private final SparkMax extenderMotor;
    private final SparkMaxConfig extenderConfig;
    public final SparkClosedLoopController extenderPID;
    public double extenderSetpoint;

    public Crane() {
        // Initializing the Gripper motorSparkMax max = new SparkMax(1, MotorType.kBrushless);
        gripperMotor = new SparkMax(CANIDS.GRIPPER, MotorType.kBrushless);
        gripperConfig = new SparkMaxConfig();
        gripperPID = gripperMotor.getClosedLoopController();

        wristMotor = new SparkMax(CANIDS.WRIST, MotorType.kBrushless);
        wristConfig = new SparkMaxConfig();
        wristPID = wristMotor.getClosedLoopController();

        elbowMotor = new SparkMax(CANIDS.ELBOW, MotorType.kBrushless);
        elbowConfig = new SparkMaxConfig();
        elbowPID = elbowMotor.getClosedLoopController();

        extenderMotor = new SparkMax(CANIDS.EXTENDER, MotorType.kBrushless);
        extenderConfig = new SparkMaxConfig();
        extenderPID = extenderMotor.getClosedLoopController();

        extenderSetpoint = CraneConstants.kExtenderCeiling;

        gripperConfig
            .idleMode(IdleMode.kBrake);
        gripperConfig.encoder
            // .inverted(true)
        // TODO: Ratio needs to be changed
            .positionConversionFactor(CraneConstants.kGripperEncoderDistancePerPulse)
            .velocityConversionFactor(CraneConstants.kGripperEncoderDistancePerPulse);
        gripperConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.001, 0.0, 0.0);
            
        gripperMotor.configure(gripperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        wristConfig
            .idleMode(IdleMode.kBrake);
        wristConfig.encoder
        // TODO: Ratio needs to be changed
            .positionConversionFactor(CraneConstants.kWristEncoderDistancePerPulse)
            .velocityConversionFactor(CraneConstants.kWristEncoderDistancePerPulse);
        wristConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.01, 0.0, 0.0);
            
        wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elbowConfig
            .idleMode(IdleMode.kBrake);
        elbowConfig.encoder
            // .inverted(true)
            .positionConversionFactor(CraneConstants.kElbowEncoderDistancePerPulse)
            .velocityConversionFactor(CraneConstants.kElbowEncoderDistancePerPulse);
        elbowConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // TODO: PID values changed temporarily for testing, 1/25/2025 was: 0.01, 0.01, 0.5 
            .pid(0.01, 0.0, 0.5);
            
        elbowMotor.configure(elbowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        extenderConfig
            .idleMode(IdleMode.kBrake);
        extenderConfig.encoder
            .positionConversionFactor(CraneConstants.kExtenderEncoderDistancePerPulse)
            .velocityConversionFactor(CraneConstants.kExtenderEncoderDistancePerPulse);
        extenderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // TODO: PID values changed temporarily for testing, 1/25/2025 was: 0.01, 0.01, 0.1
            .pid(0.01, 0.0, 0.1);
            
        extenderMotor.configure(extenderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the position of the gripper relative to its starting position, CW+.
     * 
     * @param setPoint The desired position of the gripper Motor
     */
    public void setGripperPosition(double setPoint) {
        gripperSetpoint = filterSetPoint(setPoint,
                                         CraneConstants.kGripperHardDeck,
                                         CraneConstants.kGripperCeiling);
        gripperPID.setReference(-gripperSetpoint, ControlType.kPosition);
        SmartDashboard.putNumber("Gripper Setpoint", setPoint);
    }

    /**
     * Sets the position of the wrist relative to its starting position
     * 
     * @param setPoint The desired position of the wrist motor
     */
    public void setWristPosition(double setPoint) {
        wristSetpoint = filterSetPoint(setPoint, 
                                       CraneConstants.kWristHardDeck, 
                                       CraneConstants.kWristCeiling);
        wristPID.setReference(wristSetpoint, ControlType.kPosition);
        SmartDashboard.putNumber("Wrist Setpoint", setPoint);
    }

    /**
     * Sets the position of the elbow relative to its starting position, CW+.
     * 
     * @param setPoint The desired position of the elbow motor
     */
    public void setElbowPosition(double setPoint) {
        elbowSetpoint = filterSetPoint(setPoint, 
                                       CraneConstants.kElbowHardDeck, 
                                       CraneConstants.kElbowCeiling);
        System.out.println(elbowSetpoint);
        elbowPID.setReference(-elbowSetpoint, ControlType.kPosition);
        SmartDashboard.putNumber("Elbow Setpoint", elbowSetpoint);
    }

    /**
     * Sets the position of the extender relative to its starting position.
     * Full extension is setpoint = ceiling, motor = 0.
     * Full retraction is setpoint = 0, motor = ceiling.
     * 
     * @param setPoint The desired position of the extender motor
     */
    public void setExtenderPosition(double setPoint) {
        // Full extension is setpoint = ceiling, motor = 0
        // Full retraction is setpoint = 0, motor = ceiling
        extenderSetpoint = filterSetPoint(setPoint, 
                                          CraneConstants.kExtenderHardDeck, 
                                          CraneConstants.kExtenderCeiling);
        setPoint = CraneConstants.kExtenderCeiling - extenderSetpoint;
        System.out.println("Extender: " + extenderSetpoint);
        setPoint = inchesToDegrees(setPoint);
        extenderPID.setReference(setPoint, ControlType.kPosition);
        SmartDashboard.putNumber("Extender Setpoint", extenderSetpoint);
    }

    private double inchesToDegrees(double inches) {
        return inches * 360 / CraneConstants.kPullyCircumferenceInches;
    }
    
    /**
     * Returns the ceiling if the setpoint is above it, or the hard deck if the setpoint is below it.
     * Otherwise, returns the setpoint.
     * 
     * @param setPoint The desired state of the crane.
     * @param hardDeck The hard deck of the crane.
     */
    private double filterSetPoint(double setPoint, double hardDeck, double ceiling) {
        if(setPoint < hardDeck)
            setPoint = hardDeck;
        if(setPoint > ceiling)
            setPoint = ceiling;
        return setPoint;
    }

    /** Relative to the chassis up from starting is positive. */
    public double getElbowPosition() {
        return -elbowMotor.getEncoder().getPosition();
    }

    /** Relative to the elbow joint, 0 is fully retracted, ceiling is fully extended. */
    public double getExtenderPosition() {
        return CraneConstants.kExtenderCeiling - extenderMotor.getEncoder().getPosition();
    }

    public double getGripperMotor() {
        return -gripperMotor.getEncoder().getPosition();
    }

    public double getWristPosition() {
        return wristMotor.getEncoder().getPosition();
    }
}
